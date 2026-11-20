// NODE P02 ĐƠN GIẢN — PID HÌNH HỌC (arXiv:1304.6765v1), CHỈ DÙNG TRONG GAZEBO
//
// Chép từ p01_simple, chỉ đổi lõi toán: P01 → P02Controller (thêm tích phân vị trí e_i, tư thế e_I).
// Tích phân cộng dồn từ lúc bắt đầu cất cánh tới hết hover/Case I (stage 7 và 3); trước cất cánh và lúc hạ thì giữ nguyên.
// Phân bổ rotor, tham chiếu cất cánh, Case I, hạ cánh giống hệt p01_simple.
// Cũng BỎ các lớp an toàn như p01_simple: không máy trạng thái, không dry-run, không chờ PX4 xác nhận,
// không kiểm tra dữ liệu, không trộn lực khi khởi động motor.
// Mọi giai đoạn chuyển theo THỜI GIAN, tự động từ đầu tới cuối, không cần QGC:
//
//   t (s, tính từ lúc bắt đầu lên):
//   hover: 0–6 lên 2 m │ giữ hover_time_s (mặc định 20) │ 6 s hạ cánh │ +2 s disarm │ +1 s node tự tắt
//   case1: 0–6 lên 2 m │ 7–10 chờ │ 10–40 bay Case I │ 40–45 giữ │ 6 s hạ cánh │ disarm │ tự tắt
//   Trước t=0 (2 s đầu của node): heartbeat, lệnh Offboard ở 1 s, lệnh arm ở 1,5 s.
//
// Ctrl+C giữa chừng: PX4 mất heartbeat → tự Return và hạ cánh (failsafe).
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/actuator_motors.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <chrono>
#include <fstream>
#include <limits>
#include <memory>
#include <string>
#include "uav_research/px4_topic.hpp"
#include "uav_research/p01_takeoff_reference.hpp"
#include "uav_research/x500_reference.hpp"
#include "uav_research/x500_control.hpp"
#include "uav_research/p01_telemetry.hpp"
#include "uav_research/p02_controller.hpp"

using namespace uav_research;
using Odom = px4_msgs::msg::VehicleOdometry;
using Motors = px4_msgs::msg::ActuatorMotors;
using Mode = px4_msgs::msg::OffboardControlMode;
using Command = px4_msgs::msg::VehicleCommand;

// ---- Các mốc thời gian (giây, tính từ lúc node chạy) ----
const double T_OFFBOARD = 1.0;   // gửi lệnh chuyển Offboard (PX4 cần thấy heartbeat trước)
const double T_ARM = 1.5;        // gửi lệnh arm
const double T_CLIMB = 2.0;      // bắt đầu tham chiếu cất cánh
const double HEIGHT = 2.0;       // lên thêm 2 m
const double CLIMB_TIME = 6.0;   // trong 6 s
const double HOLD_AFTER_CLIMB = 1.0; // giữ 1 s rồi mới tính là "Active"
const double SETTLE = 3.0;       // case1: chờ thêm 3 s trước khi bay
const double CASE1_END = 40.0;   // case1: t lúc hết quỹ đạo (7 + 3 + 30)
const double HOLD_END = 5.0;     // case1: giữ điểm cuối 5 s rồi hạ
const double LAND_TIME = 6.0;    // hạ cánh trong 6 s
const double BELOW_GROUND = 0.1; // đích hạ thấp hơn mặt đất 10 cm để P01 tự giảm lực

class P02Simple : public rclcpp::Node
{
public:
    P02Simple() : Node("p02_simple")
    {
        // Chỉ 2 tham số: chọn quỹ đạo và tên file CSV.
        mode_ = declare_parameter<std::string>("reference_mode", "hover");   // hover | case1
        hover_time_ = declare_parameter<double>("hover_time_s", 20.0);       // hover giữ bao lâu
        const auto csv = declare_parameter<std::string>("log_csv", "");     // rỗng = không ghi CSV

        // Tham số vật lý X500 và hệ số điều khiển: lấy nguyên bộ đã bay tốt.
        params_ = x500_parameters();
        // mass_kg: khối lượng BỘ ĐIỀU KHIỂN tin là đúng (UAV thật trong Gazebo vẫn 2,064 kg).
        // Khai sai = tạo nhiễu lực cố định để thử P01 / P02. Hệ số kx, kv tính theo khối lượng khai.
        params_.m = declare_parameter<double>("mass_kg", params_.m);
        params_.kx = 4.0 * params_.m;
        params_.kv = 3.2 * params_.m;

        // Hệ số P02 (mặc định xem P02Gains). ki mặc định = 1.0 * mass_kg khai báo.
        gains_.ki = declare_parameter<double>("ki", 1.0 * params_.m);
        gains_.c1 = declare_parameter<double>("c1", gains_.c1);
        gains_.sigma = declare_parameter<double>("sigma", gains_.sigma);
        gains_.c2 = declare_parameter<double>("c2", gains_.c2);
        const double kI_xy = declare_parameter<double>("kI_xy", gains_.kI.x());
        const double kI_z = declare_parameter<double>("kI_z", gains_.kI.z());
        gains_.kI = Vector3(kI_xy, kI_xy, kI_z);
        // use_integral:=false → tắt cộng dồn tích phân (còn lại khác P01 ở dạng mômen (26)).
        use_integral_ = declare_parameter<bool>("use_integral", true);

        // 3 publisher gửi vào PX4 (/fmu/in) + telemetry cho rqt_plot/RViz.
        motor_pub_ = create_publisher<Motors>(px4_topic<Motors>("/fmu/in/actuator_motors"), rclcpp::SensorDataQoS());
        mode_pub_ = create_publisher<Mode>(px4_topic<Mode>("/fmu/in/offboard_control_mode"), 10);
        cmd_pub_ = create_publisher<Command>(px4_topic<Command>("/fmu/in/vehicle_command"), 10);
        telemetry_ = std::make_unique<P01Telemetry>(*this);

        // 1 subscriber: odometry. Callback CHỈ lưu trạng thái, không tính gì.
        odom_sub_ = create_subscription<Odom>(px4_topic<Odom>("/fmu/out/vehicle_odometry"),
            rclcpp::SensorDataQoS(), [this](Odom::SharedPtr msg) { save_state(*msg); });

        if (!csv.empty())
        {
            log_.open(csv);
            log_ << "t,x,y,z,xd,yd,zd,position_error,Psi,f_requested,f_achieved,"
                    "Mx,My,Mz,Mx_achieved,My_achieved,Mz_achieved,u0,u1,u2,u3,saturated,stage,control_blend,"
                    "ei_x,ei_y,ei_z,eI_x,eI_y,eI_z\n";
        }

        // 1 timer 100 Hz: TOÀN BỘ việc điều khiển nằm trong loop().
        start_ = std::chrono::steady_clock::now();
        last_loop_ = start_;
        timer_ = create_wall_timer(std::chrono::milliseconds(10), [this]() { loop(); });
        RCLCPP_INFO(get_logger(), "p02_simple: reference_mode=%s mass_kg=%.3f ki=%.2f c1=%.2f sigma=%.2f kI=(%.3f,%.3f) c2=%.2f integral=%d (CHỈ DÙNG TRONG MÔ PHỎNG)",
            mode_.c_str(), params_.m, gains_.ki, gains_.c1, gains_.sigma, kI_xy, kI_z, gains_.c2, use_integral_);
    }

private:
    // Đổi bản tin PX4 sang trạng thái P01/P02: x, v (NED), omega (thân FRD), R.
    void save_state(const Odom &msg)
    {
        state_.x = Vector3(msg.position[0], msg.position[1], msg.position[2]);
        state_.v = Vector3(msg.velocity[0], msg.velocity[1], msg.velocity[2]);
        state_.omega = Vector3(msg.angular_velocity[0], msg.angular_velocity[1], msg.angular_velocity[2]);
        // PX4 xếp quaternion w,x,y,z — trùng thứ tự Eigen.
        state_.R = Eigen::Quaterniond(msg.q[0], msg.q[1], msg.q[2], msg.q[3]).normalized().toRotationMatrix();
        sample_ = msg.timestamp_sample;
        have_odom_ = true;
    }

    // Vòng điều khiển: chạy mỗi 10 ms, đọc từ trên xuống.
    void loop()
    {
        const auto tick = std::chrono::steady_clock::now();
        const double now = std::chrono::duration<double>(tick - start_).count();
        const double dt = std::chrono::duration<double>(tick - last_loop_).count();   // bước tích phân thực tế
        last_loop_ = tick;

        // 1. Heartbeat: báo PX4 "tôi điều khiển motor trực tiếp".
        Mode mode{};
        mode.timestamp = stamp_us();
        mode.direct_actuator = true;
        mode_pub_->publish(mode);

        if (!have_odom_) return;   // chưa có dữ liệu thì chưa tính được

        // 2. Trước khi cất cánh: chốt gốc tại vị trí và hướng mũi hiện tại.
        if (now < T_CLIMB)
        {
            origin_ = state_.x;
            heading_ = state_.R.col(0);   // trục x thân = hướng mũi
        }

        // 3. Gửi lệnh Offboard và arm, mỗi lệnh đúng một lần.
        if (now >= T_OFFBOARD && !sent_offboard_) { send_command(Command::VEHICLE_CMD_DO_SET_MODE, 1, 6); sent_offboard_ = true; }
        if (now >= T_ARM && !sent_arm_) { send_command(Command::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1, 0); sent_arm_ = true; }

        // 4. Tham chiếu theo thời gian. t = giây kể từ lúc bắt đầu cất cánh.
        const double t = std::max(0.0, now - T_CLIMB);
        const double t_active = CLIMB_TIME + HOLD_AFTER_CLIMB;   // 7 s
        const double t_land = (mode_ == "case1") ? CASE1_END + HOLD_END : CLIMB_TIME + hover_time_;
        P01Reference ref;
        int stage;
        if (t < t_land)
        {
            if (t < t_active || mode_ != "case1")
                ref = reference_takeoff(t, origin_, heading_, HEIGHT, CLIMB_TIME);   // lên cao rồi giữ
            else
            {
                Vector3 top = origin_;
                top.z() -= HEIGHT;   // NED: lên cao = z giảm
                ref = reference_x500_case1(t - t_active - SETTLE, top, heading_);   // Case I 0,25×, 30 s
            }
            stage = (now < T_CLIMB) ? 0 : (t < t_active ? 7 : 3);
        }
        else
        {
            // Hạ cánh: thẳng đứng từ điểm đang giữ xuống dưới mặt đất 10 cm.
            if (!landing_)
            {
                land_start_ = last_ref_x_;                      // điểm bắt đầu hạ
                ground_ = land_start_;
                ground_.z() = origin_.z() + BELOW_GROUND;       // NED: z lớn hơn = thấp hơn
                landing_ = true;
                RCLCPP_INFO(get_logger(), "Bắt đầu hạ cánh");
            }
            // Dùng lại đa thức cất cánh nhưng chạy NGƯỢC thời gian (từ đỉnh về đất).
            // Đảo thời gian thì đạo hàm bậc lẻ (v, jerk) đổi dấu.
            const double tau = t - t_land;
            ref = reference_takeoff(LAND_TIME - tau, ground_, heading_, ground_.z() - land_start_.z(), LAND_TIME);
            ref.v = -ref.v;
            ref.j = -ref.j;
            stage = 4;   // ghi CSV: đoạn hạ cánh

            // Hết đa thức hạ cánh = đã chạm đất: xóa tích phân P02 (một lần), giống PX4 xóa tích phân khi đã đáp.
            // Nếu giữ, k_i·e_i (~2,6 N khi khai 1,8 kg) làm lực trên đất còn ≈ 97% trọng lượng, UAV gần như lơ lửng.
            if (tau >= LAND_TIME && !reset_on_ground_)
            {
                ctrl_.reset();
                reset_on_ground_ = true;
                RCLCPP_INFO(get_logger(), "Chạm đất: xóa tích phân P02 để giảm lực trước khi disarm");
            }

            // Chạm đất xong 2 s thì disarm, thêm 1 s thì tắt node.
            // Phải dùng disarm CƯỠNG BỨC (param2 = 21196). Log PX4 21/11: disarm thường luôn bị từ chối
            // "Disarming denied: not landed" vì ở chế độ gửi lệnh motor trực tiếp, bộ phát hiện hạ cánh
            // của PX4 không bao giờ báo đã đáp → node tắt → mất Offboard → failsafe Return (bay lên rồi đáp).
            if (tau >= LAND_TIME + 2.0 && !sent_disarm_)
            {
                send_command(Command::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0, 21196);   // 21196 = cưỡng bức
                sent_disarm_ = true;
            }
            if (tau >= LAND_TIME + 3.0)
            {
                RCLCPP_INFO(get_logger(), "Đã hạ cánh và disarm, kết thúc node");
                rclcpp::shutdown();   // spin() trong main() dừng lại
                return;
            }
        }
        last_ref_x_ = ref.x;

        // 5. P02: trạng thái + tham chiếu → lực f, mômen M.
        // Tích phân từ lúc bắt đầu cất cánh (stage 7) để khử lệch trước khi bay Case I.
        // Không tích phân lúc chờ trước cất cánh (stage 0) và lúc hạ cánh (stage 4, đích dưới mặt đất).
        const bool integrate = use_integral_ && (stage == 7 || stage == 3);
        const P02Control c = ctrl_.compute(state_, params_, gains_, ref, dt, integrate);

        // 6. Phân bổ: f, M → 4 lệnh rotor u0..u3 (có giới hạn).
        const X500Allocation a = allocate_x500(c.f, c.M, motors_);

        // 7. Gửi lệnh motor. Kênh 5–12 để NaN = không dùng.
        Motors m{};
        m.timestamp = stamp_us();
        m.timestamp_sample = sample_;
        m.control.fill(std::numeric_limits<float>::quiet_NaN());
        for (int i = 0; i < 4; ++i) m.control[i] = a.commands[i];
        motor_pub_->publish(m);

        // 8. Quan sát: telemetry, log 1 lần/giây, CSV.
        telemetry_->publish(sample_, get_clock()->now(), state_, ref, c);
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "t=%.1f stage=%d ex=%.3f f=%.2f ei_z=%.3f saturated=%d",
            t, stage, c.ex.norm(), c.f, c.ei.z(), a.saturated);
        if (log_)
        {
            log_ << t;
            for (int i = 0; i < 3; ++i) log_ << ',' << state_.x[i];
            for (int i = 0; i < 3; ++i) log_ << ',' << ref.x[i];
            log_ << ',' << c.ex.norm() << ',' << c.Psi << ',' << c.f << ',' << a.achieved_f;
            for (int i = 0; i < 3; ++i) log_ << ',' << c.M[i];
            for (int i = 0; i < 3; ++i) log_ << ',' << a.achieved_M[i];
            for (double u : a.commands) log_ << ',' << u;
            log_ << ',' << a.saturated << ',' << stage << ",1";
            for (int i = 0; i < 3; ++i) log_ << ',' << c.ei[i];
            for (int i = 0; i < 3; ++i) log_ << ',' << c.eI[i];
            log_ << '\n';
        }
    }

    // Gửi một lệnh VehicleCommand. Offboard: (DO_SET_MODE, 1, 6). Arm: (ARM_DISARM, 1, 0). Disarm cưỡng bức: (ARM_DISARM, 0, 21196).
    void send_command(uint32_t command, float p1, float p2)
    {
        Command msg{};
        msg.timestamp = stamp_us();
        msg.command = command;
        msg.param1 = p1;
        msg.param2 = p2;
        msg.target_system = msg.source_system = 1;
        msg.target_component = msg.source_component = 1;
        msg.from_external = true;
        cmd_pub_->publish(msg);
        RCLCPP_INFO(get_logger(), "Đã gửi lệnh %u", command);
    }

    uint64_t stamp_us() { return get_clock()->now().nanoseconds() / 1000; }

    // Dữ liệu
    std::string mode_;
    double hover_time_ = 20.0;
    P01Parameters params_;
    P02Gains gains_;
    P02Controller ctrl_;
    bool use_integral_ = true;
    bool reset_on_ground_ = false;   // đã xóa tích phân lúc chạm đất chưa
    X500Actuators motors_;
    P01State state_;
    Vector3 origin_ = Vector3::Zero(), heading_ = Vector3::UnitX();
    uint64_t sample_ = 0;
    bool have_odom_ = false, sent_offboard_ = false, sent_arm_ = false;
    bool landing_ = false, sent_disarm_ = false;
    Vector3 last_ref_x_ = Vector3::Zero(), land_start_ = Vector3::Zero(), ground_ = Vector3::Zero();
    std::chrono::steady_clock::time_point start_, last_loop_;
    std::ofstream log_;
    // Kết nối ROS
    rclcpp::Publisher<Motors>::SharedPtr motor_pub_;
    rclcpp::Publisher<Mode>::SharedPtr mode_pub_;
    rclcpp::Publisher<Command>::SharedPtr cmd_pub_;
    rclcpp::Subscription<Odom>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<P01Telemetry> telemetry_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<P02Simple>());
    if (rclcpp::ok()) rclcpp::shutdown();   // Ctrl+C: dọn dẹp; tự kết thúc thì đã shutdown rồi
    return 0;
}
