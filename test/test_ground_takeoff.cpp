// Kiểm tra logic thuần C++: không tạo ROS node, không arm hoặc điều khiển PX4.
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

#include "uav_research/p01_ground_session.hpp"
#include "uav_research/p01_takeoff_reference.hpp"
#include "uav_research/x500_control.hpp"

namespace
{
using Session = uav_research::P01GroundSession;
using Inputs = uav_research::GroundInputs;

void require(bool ok, const char *message)
{
    if (!ok) throw std::runtime_error(message);
}

Inputs ground()
{
    Inputs in;
    in.enabled = in.feedback_ok = in.land_ok = in.ready_ground = in.landed = true;
    return in;
}

void prime(Session &session, Inputs &in)
{
    const auto first = session.step(0.0, in);
    require(first.heartbeat && first.motors && first.control_blend == 0.0,
        "Priming must stream only low motor commands");
    require(!first.request_arm && !first.request_offboard, "No premature command");
    require(!session.step(1.19, in).request_offboard, "Offboard requested too soon");
    require(session.step(1.21, in).request_offboard, "Missing Offboard request");
    require(!session.step(1.22, in).request_offboard, "Offboard request repeated");
}

void check_session()
{
    // Disabled means absolutely no commands, even with otherwise valid inputs.
    Session dry;
    auto in = ground();
    in.enabled = false;
    const auto off = dry.step(0.0, in);
    require(!off.heartbeat && !off.motors && !off.request_arm &&
        !off.request_offboard && !off.request_hold, "Dry-run issued an action");

    Session session;
    in = ground();
    prime(session, in);
    in.offboard = true;
    require(session.step(1.30, in).request_arm, "Arm requires Offboard confirmation");
    require(!session.step(1.31, in).request_arm, "Arm request repeated");
    in.armed = true;
    in.ready_ground = false; // Readiness requires disarmed, but flight must continue.
    require(session.step(1.40, in).control_blend == 0.0, "Spin-up must start at idle");
    const auto middle = session.step(2.40, in);
    require(std::abs(middle.control_blend - 0.5) < 1e-12, "Incorrect spin-up blend");
    const auto lift = session.step(3.41, in);
    require(lift.start_takeoff && lift.control_blend == 1.0, "Missing takeoff event");
    require(!session.step(3.42, in).start_takeoff, "Takeoff event repeated");
    in.landed = false;
    in.takeoff_complete = true;
    require(session.step(10.0, in).motors && session.stage() == Session::Stage::Active,
        "Cannot enter Active after confirmed takeoff");
    require(static_cast<int>(session.stage()) == 3, "CSV Active stage changed");

    in.offboard = false;
    const auto release = session.step(10.1, in);
    require(!release.motors && !release.heartbeat && !release.request_hold &&
        session.stage() == Session::Stage::Stopped, "Manual exit must release control");
    in.offboard = true;
    require(!session.step(11.0, in).motors, "Stopped session restarted");

    Session mode_timeout;
    in = ground();
    prime(mode_timeout, in);
    require(!mode_timeout.step(4.22, in).motors &&
        mode_timeout.stage() == Session::Stage::Stopped, "Missing Offboard timeout");

    Session arm_timeout;
    in = ground();
    prime(arm_timeout, in);
    in.offboard = true;
    arm_timeout.step(1.30, in);
    require(!arm_timeout.step(4.31, in).motors &&
        arm_timeout.stage() == Session::Stage::Stopped, "Missing arm timeout");

    Session stale;
    in = ground();
    prime(stale, in);
    in.offboard = true;
    stale.step(1.30, in);
    in.armed = true;
    stale.step(1.40, in);
    in.land_ok = false;
    const auto lost = stale.step(1.50, in);
    require(!lost.motors && lost.request_hold, "Stale feedback did not release to Hold");

    Session conflict;
    in = ground();
    in.exclusive = false;
    const auto duplicate = conflict.step(0.0, in);
    require(!duplicate.motors && !duplicate.request_offboard && !duplicate.request_hold,
        "Competing publisher must prevent control");

    Session airborne;
    in = ground();
    in.armed = true;
    require(!airborne.step(0.0, in).motors && airborne.stage() == Session::Stage::Stopped,
        "New ground session must reject an already armed vehicle");

    Session no_liftoff;
    in = ground();
    prime(no_liftoff, in);
    in.offboard = true;
    no_liftoff.step(1.30, in);
    in.armed = true;
    no_liftoff.step(1.40, in);
    no_liftoff.step(3.41, in);
    const auto expired = no_liftoff.step(23.42, in);
    require(!expired.motors && expired.request_hold &&
        no_liftoff.stage() == Session::Stage::Stopped, "Missing takeoff timeout");

    Session failsafe;
    in = ground();
    prime(failsafe, in);
    in.failsafe = true;
    const auto failed = failsafe.step(1.30, in);
    require(!failed.motors && !failed.heartbeat && !failed.request_arm &&
        !failed.request_hold, "Failsafe must stop commands without overriding PX4");
}

void check_reference()
{
    const Eigen::Vector3d origin(1.0, -2.0, 0.1);
    const Eigen::Vector3d heading = Eigen::Vector3d::UnitX();
    const auto reference = [&](double t) {
        return uav_research::reference_takeoff(t, origin, heading, 2.0, 6.0);
    };
    const auto start = reference(0.0);
    const auto end = reference(6.0);
    require((start.x - origin).norm() < 1e-12, "Takeoff starts at wrong origin");
    require(std::abs(end.x.z() - (origin.z() - 2.0)) < 1e-12, "Wrong NED endpoint");
    require(end.v.norm() + end.a.norm() + end.j.norm() + end.snap.norm() == 0.0,
        "Endpoint must hold with zero derivatives");
    require((reference(8.0).x - end.x).norm() == 0.0, "Endpoint does not hold");
    require(std::abs(reference(3.0).x.z() - (origin.z() - 1.0)) < 1e-12,
        "Incorrect midpoint");

    double previous_z = origin.z();
    for (int i = 0; i <= 600; ++i)
    {
        const auto r = reference(i * 0.01);
        require(r.x.allFinite() && r.v.allFinite() && r.a.allFinite() &&
            r.j.allFinite() && r.snap.allFinite(), "Non-finite reference");
        require(r.x.z() <= previous_z + 1e-10 && r.x.z() >= end.x.z() - 1e-10,
            "Takeoff must rise monotonically without overshoot");
        require(r.x.x() == origin.x() && r.x.y() == origin.y(), "Horizontal target drifted");
        previous_z = r.x.z();
    }

    // Đối chiếu các đạo hàm giải tích với sai phân, không chép lại hệ số đa thức.
    const double dt = 1e-4;
    for (const double t : {0.5, 1.7, 3.0, 4.1, 5.5})
    {
        const auto before = reference(t - dt);
        const auto at = reference(t);
        const auto after = reference(t + dt);
        require(((after.x - before.x) / (2 * dt) - at.v).norm() < 1e-6,
            "Velocity does not differentiate position");
        require(((after.v - before.v) / (2 * dt) - at.a).norm() < 1e-6,
            "Acceleration does not differentiate velocity");
        require(((after.a - before.a) / (2 * dt) - at.j).norm() < 1e-6,
            "Jerk does not differentiate acceleration");
        require(((after.j - before.j) / (2 * dt) - at.snap).norm() < 1e-6,
            "Snap does not differentiate jerk");
    }
    bool rejected = false;
    try { uav_research::reference_takeoff(0, origin, heading, 2, 0); }
    catch (const std::invalid_argument &) { rejected = true; }
    require(rejected, "Zero duration accepted");
    rejected = false;
    try { reference(std::numeric_limits<double>::quiet_NaN()); }
    catch (const std::invalid_argument &) { rejected = true; }
    require(rejected, "NaN time accepted");
}

void check_rotor_transition()
{
    const auto parameters = uav_research::x500_parameters();
    const uav_research::X500Actuators motors;
    const double idle = 4.0 * motors.min_thrust();
    const double hover = parameters.m * parameters.g;
    double previous = 0.0;
    for (const double blend : {0.0, 0.25, 0.5, 0.75, 1.0})
    {
        const auto out = uav_research::allocate_x500(
            idle + blend * (hover - idle), Eigen::Vector3d::Zero(), motors);
        require(std::abs(out.achieved_f - (idle + blend * (hover - idle))) < 1e-10,
            "Rotor allocation lost requested collective");
        require(out.achieved_f >= previous && out.achieved_M.norm() < 1e-12,
            "Transition must increase collective without torque");
        for (const double u : out.commands)
            require(std::isfinite(u) && u >= 0.0 && u <= 1.0, "Invalid motor command");
        previous = out.achieved_f;
    }
}
} // namespace

int main()
{
    try
    {
        check_session();
        check_reference();
        check_rotor_transition();
        std::cout << "Ground session, takeoff reference and rotor transition: PASS\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << "Ground takeoff checks: FAIL: " << error.what() << '\n';
        return 1;
    }
}
