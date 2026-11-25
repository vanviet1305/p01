// NODE P01 ĐƠN GIẢN — CHỈ ĐỂ HỌC LUỒNG CHÍNH, CHỈ DÙNG TRONG GAZEBO
//
// Cùng lõi toán với p01_offboard_hover (P01, phân bổ rotor, tham chiếu cất cánh, Case I)
// nhưng BỎ các lớp an toàn: không máy trạng thái, không dry-run, không chờ PX4 xác nhận,
// không kiểm tra dữ liệu, không trộn lực khi khởi động motor.
// Mọi giai đoạn chuyển theo THỜI GIAN, tự động từ đầu tới cuối, không cần QGC:
//
//   t (s, tính từ lúc bắt đầu lên):
//   hover: 0–6 lên 2 m │ giữ hover_time_s (mặc định 20) │ 6 s hạ cánh │ +2 s disarm │ +1 s node tự tắt
//   case1: 0–6 lên 2 m │ 7–10 chờ │ 10–(10+T) bay Case I │ giữ 5 s │ 8 s bay về trên điểm cất cánh │ giữ 2 s
//          │ 6 s hạ cánh │ disarm │ tự tắt   (return_home:=false bỏ đoạn bay về, hạ tại điểm cuối như cũ)
//          T = 4 + 6,5/case1_speed: 30 s ở 0,25× (mặc định), 17 s ở 0,5×, 10,5 s ở 1×
//   figure8: 0–6 lên 2 m │ 7–10 chờ │ bay số 8 quanh điểm ngay trên chỗ cất cánh, T = fig8_laps·fig8_period_s + 4
//          (mặc định 2 vòng × 10 s + 4 = 24 s) │ giữ 5 s tại tâm │ 6 s hạ cánh │ disarm │ tự tắt
//   Trước t=0 (2 s đầu của node): heartbeat, lệnh Offboard ở 1 s, lệnh arm ở 1,5 s.
//
// Ctrl+C giữa chừng: PX4 mất heartbeat → tự Return và hạ cánh (failsafe).
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/actuator_motors.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <limits>
#include <memory>
#include <string>
#include "uav_research/px4_topic.hpp"
#include "uav_research/p01_takeoff_reference.hpp"
#include "uav_research/x500_reference.hpp"
#include "uav_research/x500_control.hpp"
#include "uav_research/p01_telemetry.hpp"
#include "uav_research/p01_return_reference.hpp"
#include "uav_research/p01_figure8_reference.hpp"
#include "uav_research/gz_ground_truth.hpp"

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
const double CASE1_RAMP = 4.0;   // case1: thời gian tăng/giảm tốc êm của pha (s)
const double CASE1_PHASE = 6.5;  // case1: tổng pha Case I = 0,25 × (30 − 4); giữ nguyên hình quỹ đạo khi đổi tốc độ
const double HOLD_END = 5.0;     // case1: giữ điểm cuối 5 s
const double RETURN_TIME = 8.0;  // case1: bay về ngay trên điểm cất cánh trong 8 s (~2,7 m, v max ~0,74 m/s)
const double RETURN_HOLD = 2.0;  // case1: giữ 2 s trên điểm cất cánh rồi mới hạ
const double LAND_TIME = 6.0;    // hạ cánh trong 6 s
const double BELOW_GROUND = 0.1; // đích hạ thấp hơn mặt đất 10 cm để P01 tự giảm lực

class P01Simple : public rclcpp::Node
{
public:
    P01Simple() : Node("p01_simple")
    {
        // Chỉ 2 tham số: chọn quỹ đạo và tên file CSV.
        mode_ = declare_parameter<std::string>("reference_mode", "hover");   // hover | case1 | figure8
        hover_time_ = declare_parameter<double>("hover_time_s", 20.0);       // hover giữ bao lâu
        // case1_speed: tốc độ pha Case I (0,25 = bản đã bay; 0,5; 1 = tốc độ bài báo).
        // Cùng một hình quỹ đạo, tốc độ gấp đôi thì thời gian bay giảm một nửa, gia tốc gấp 4.
        case1_speed_ = declare_parameter<double>("case1_speed", 0.25);
        // return_home: sau Case I bay về trên điểm cất cánh rồi mới hạ, để lần bay sau vẫn xuất phát từ gốc.
        // Lý do: log 21/11 cho thấy sai số tăng theo khoảng cách tới gốc tọa độ PX4.
        return_home_ = declare_parameter<bool>("return_home", true);
        // state_source: "px4" = trạng thái EKF2 (mặc định, như cũ); "gazebo" = vị trí/tư thế/vận tốc THẬT từ Gazebo
        // (/model/x500_0/odometry, cần chạy tools/gan_odom_gazebo.sh trước). Vận tốc góc luôn lấy từ PX4 (con quay).
        // Lý do: EKF2 ước lượng sai roll 1–4° khi bay gắt (log 21/11), làm lẫn lỗi ước lượng vào so sánh bộ điều khiển.
        // vehicle: mô hình UAV đang chạy trong Gazebo (x500 | x650). Phải khớp lệnh make px4_sitl gz_<vehicle>.
        vehicle_ = declare_parameter<std::string>("vehicle", "x500");
        if (vehicle_ != "x500" && vehicle_ != "x650")
            throw std::invalid_argument("vehicle phải là x500 hoặc x650");
        use_gt_ = declare_parameter<std::string>("state_source", "px4") == "gazebo";
        const std::string gt_topic = "/model/" + vehicle_ + "_0/odometry";
        if (use_gt_ && !gt_.start(gt_topic))
            RCLCPP_ERROR(get_logger(), "Không đăng ký được topic Gazebo %s", gt_topic.c_str());
        case1_duration_ = CASE1_RAMP + CASE1_PHASE / case1_speed_;   // 30 s ở 0,25×; 17 s ở 0,5×; 10,5 s ở 1×
        // Quỹ đạo số 8 (reference_mode:=figure8): dọc = a·sin θ, ngang = b·sin 2θ, θ = 2π·t/period.
        // Giảm fig8_period_s để bay gắt hơn (gia tốc tỉ lệ 1/period²).
        fig8_a_ = declare_parameter<double>("fig8_a", 1.5);              // nửa chiều dài dọc hướng mũi (m)
        fig8_b_ = declare_parameter<double>("fig8_b", 0.75);             // biên độ ngang (m)
        fig8_period_ = declare_parameter<double>("fig8_period_s", 10.0); // thời gian một vòng số 8 (s)
        fig8_laps_ = static_cast<int>(declare_parameter<int>("fig8_laps", 2));   // số vòng
        fig8_duration_ = figure8_duration(fig8_period_, fig8_laps_, CASE1_RAMP);
        if (mode_ == "figure8")
            RCLCPP_INFO(get_logger(), "Quỹ đạo số 8: a=%.2f m, b=%.2f m, chu kỳ %.1f s, %d vòng, bay %.1f s",
                fig8_a_, fig8_b_, fig8_period_, fig8_laps_, fig8_duration_);
        const auto csv = declare_parameter<std::string>("log_csv", "");     // rỗng = không ghi CSV

        // Tham số vật lý X500 và hệ số điều khiển: lấy nguyên bộ đã bay tốt.
        // Tham số vật lý + hệ số điều khiển theo mô hình (x500: bộ đã bay tốt; x650: ước lượng, xem x500_control.hpp).
        params_ = vehicle_ == "x650" ? x650_parameters() : x500_parameters();
        motors_ = vehicle_ == "x650" ? x650_actuators() : X500Actuators{};
        // att_bw_scale (s): co băng thông vòng tư thế. kR nhân s², kOmega nhân s -> giữ hệ số tắt dần, tần số riêng nhân s.
        // Dùng khi motor chậm (x650) làm vòng tư thế dao động. Mặc định 1 = giữ nguyên.
        att_bw_scale_ = declare_parameter<double>("att_bw_scale", 1.0);
        if (!(att_bw_scale_ > 0.0)) throw std::invalid_argument("att_bw_scale phải > 0");
        params_.attitude_scale *= att_bw_scale_ * att_bw_scale_;
        params_.rate_scale *= att_bw_scale_;
        // mass_kg: khối lượng BỘ ĐIỀU KHIỂN tin là đúng (UAV thật trong Gazebo: x500 2,064 kg; x650 3,0 kg).
        // Khai sai = tạo nhiễu lực cố định để thử P01 / P02. Hệ số kx, kv tính theo khối lượng khai.
        params_.m = declare_parameter<double>("mass_kg", params_.m);
        params_.kx = 4.0 * params_.m;
        params_.kv = 3.2 * params_.m;
        // motor_k_scale: hệ số lực motor mà BỘ ĐIỀU KHIỂN tin = k thật (Gazebo) × scale.
        // 1,1 = tin motor khỏe hơn 10% → lệnh u thấp hơn cần thiết → mỗi rotor chỉ ra ~91% lực yêu cầu.
        // Mô phỏng sai đường cong lực/bàn đo lực trên máy bay thật: sai cả lực tổng lẫn mômen.
        const double motor_k_scale = declare_parameter<double>("motor_k_scale", 1.0);
        motors_.motor_constant *= motor_k_scale;

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
                    "state_source_gazebo,ekf_x,ekf_y,ekf_z,ekf_tilt_err_deg\n";
        }

        // 1 timer 100 Hz: TOÀN BỘ việc điều khiển nằm trong loop().
        start_ = std::chrono::steady_clock::now();
        timer_ = create_wall_timer(std::chrono::milliseconds(10), [this]() { loop(); });
        RCLCPP_INFO(get_logger(), "p01_simple: vehicle=%s att_bw_scale=%.2f reference_mode=%s mass_kg=%.3f case1_speed=%.2f motor_k_scale=%.2f (CHỈ DÙNG TRONG MÔ PHỎNG)",
            vehicle_.c_str(), att_bw_scale_, mode_.c_str(), params_.m, case1_speed_, motor_k_scale);
    }

private:
    // Đổi bản tin PX4 sang trạng thái P01: x, v (NED), omega (thân FRD), R.
    void save_state(const Odom &msg)
    {
        state_.x = Vector3(msg.position[0], msg.position[1], msg.position[2]);
        state_.v = Vector3(msg.velocity[0], msg.velocity[1], msg.velocity[2]);
        state_.omega = Vector3(msg.angular_velocity[0], msg.angular_velocity[1], msg.angular_velocity[2]);
        // PX4 xếp quaternion w,x,y,z — trùng thứ tự Eigen.
        state_.R = Eigen::Quaterniond(msg.q[0], msg.q[1], msg.q[2], msg.q[3]).normalized().toRotationMatrix();
        px4_x_ = state_.x;
        px4_R_ = state_.R;
        sample_ = msg.timestamp_sample;
        have_odom_ = true;
    }

    // Vòng điều khiển: chạy mỗi 10 ms, đọc từ trên xuống.
    void loop()
    {
        const double now = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_).count();

        // 1. Heartbeat: báo PX4 "tôi điều khiển motor trực tiếp".
        Mode mode{};
        mode.timestamp = stamp_us();
        mode.direct_actuator = true;
        mode_pub_->publish(mode);

        if (!have_odom_) return;   // chưa có dữ liệu thì chưa tính được

        // 1b. Trạng thái thật từ Gazebo (state_source:=gazebo): thay x, v, R; giữ omega từ con quay PX4.
        //     Chưa có dữ liệu thì giữ đồng hồ node ở 0 (chưa cất cánh) và chờ.
        if (use_gt_)
        {
            Vector3 xg, vg;
            Matrix3 Rg;
            double age = 0.0;
            if (!gt_.get(xg, vg, Rg, age))
            {
                start_ = std::chrono::steady_clock::now();
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                    "Chưa có trạng thái thật từ Gazebo. Chạy: ~/ros2_px4_ws/src/p01/tools/gan_odom_gazebo.sh");
                return;
            }
            if (age > 0.2)
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Trạng thái thật từ Gazebo trễ %.2f s", age);
            state_.x = xg;
            state_.v = vg;
            state_.R = Rg;
        }

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
        const bool is_traj = (mode_ == "case1" || mode_ == "figure8");   // có bay quỹ đạo (không chỉ treo)
        const double traj_duration = (mode_ == "figure8") ? fig8_duration_ : case1_duration_;
        const double case1_end = t_active + SETTLE + traj_duration;   // t lúc hết quỹ đạo (40 s ở Case I 0,25×)
        const double t_return = case1_end + HOLD_END;   // case1: bắt đầu bay về
        // figure8 kết thúc đúng tại tâm (ngay trên điểm cất cánh) nên không cần bay về.
        const double t_land = is_traj
            ? t_return + ((mode_ == "case1" && return_home_) ? RETURN_TIME + RETURN_HOLD : 0.0)
            : CLIMB_TIME + hover_time_;
        P01Reference ref;
        int stage;
        if (t < t_land)
        {
            if (t < t_active || !is_traj)
                ref = reference_takeoff(t, origin_, heading_, HEIGHT, CLIMB_TIME);   // lên cao rồi giữ
            else if (mode_ == "case1" && t >= t_return)
            {
                // Bay về: thẳng ngang từ điểm cuối Case I tới ngay trên điểm cất cánh, độ cao giữ nguyên.
                if (!returning_)
                {
                    return_from_ = last_ref_x_;
                    returning_ = true;
                    RCLCPP_INFO(get_logger(), "Bay về trên điểm cất cánh");
                }
                Vector3 home = origin_;
                home.z() = return_from_.z();
                ref = reference_return(t - t_return, return_from_, home, RETURN_TIME, heading_);
            }
            else
            {
                Vector3 top = origin_;
                top.z() -= HEIGHT;   // NED: lên cao = z giảm
                if (mode_ == "figure8")
                    ref = reference_figure8(t - t_active - SETTLE, top, heading_, fig8_a_, fig8_b_, fig8_period_, fig8_laps_, CASE1_RAMP);
                else
                    ref = reference_x500_case1(t - t_active - SETTLE, top, heading_, case1_speed_, CASE1_RAMP, case1_duration_);
            }
            // stage 5 = bay về (không tính vào sai số Case I).
            stage = (now < T_CLIMB) ? 0 : (t < t_active ? 7 : (mode_ == "case1" && t >= t_return ? 5 : 3));
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

        // 5. P01: trạng thái + tham chiếu → lực f, mômen M.
        const P01Control c = compute_p01_control_with_reference(state_, params_, ref);

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
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "t=%.1f stage=%d ex=%.3f f=%.2f saturated=%d",
            t, stage, c.ex.norm(), c.f, a.saturated);
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
            // EKF2 so với trạng thái đang dùng: lệch vị trí và góc lệch trục z thân (nghiêng).
            log_ << ',' << use_gt_;
            for (int i = 0; i < 3; ++i) log_ << ',' << px4_x_[i];
            log_ << ',' << std::acos(std::clamp(px4_R_.col(2).dot(state_.R.col(2)), -1.0, 1.0)) * 180.0 / M_PI << '\n';
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
    double case1_speed_ = 0.25, case1_duration_ = 30.0;
    double fig8_a_ = 1.5, fig8_b_ = 0.75, fig8_period_ = 10.0, fig8_duration_ = 24.0;
    int fig8_laps_ = 2;
    bool return_home_ = true, returning_ = false;
    bool use_gt_ = false;
    GzGroundTruth gt_;
    Vector3 px4_x_ = Vector3::Zero();          // vị trí EKF2 (ghi CSV để so với trạng thái thật)
    Matrix3 px4_R_ = Matrix3::Identity();      // tư thế EKF2
    Vector3 return_from_ = Vector3::Zero();
    std::string vehicle_;
    double att_bw_scale_ = 1.0;
    P01Parameters params_;
    X500Actuators motors_;
    P01State state_;
    Vector3 origin_ = Vector3::Zero(), heading_ = Vector3::UnitX();
    uint64_t sample_ = 0;
    bool have_odom_ = false, sent_offboard_ = false, sent_arm_ = false;
    bool landing_ = false, sent_disarm_ = false;
    Vector3 last_ref_x_ = Vector3::Zero(), land_start_ = Vector3::Zero(), ground_ = Vector3::Zero();
    std::chrono::steady_clock::time_point start_;
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
    rclcpp::spin(std::make_shared<P01Simple>());
    if (rclcpp::ok()) rclcpp::shutdown();   // Ctrl+C: dọn dẹp; tự kết thúc thì đã shutdown rồi
    return 0;
}
