// NODE CHUNG uav_simple — CHỌN BỘ ĐIỀU KHIỂN BẰNG controller:=p01|p02|p06|p11, CHỈ DÙNG TRONG GAZEBO
//
// Gộp p01_simple và p02_simple (chép từ p02_simple, 25/11/2026). Phần luật điều khiển gọi qua giao diện
// Controller (controller_interface.hpp); mọi phần còn lại (giai đoạn bay, quỹ đạo, phân bổ rotor, CSV) dùng chung.
// controller:=p01 phải cho kết quả trùng p01_simple, controller:=p02 trùng p02_simple (kiểm tra tái lập).
// Trạng thái bên trong (tích phân, thích nghi) được cộng dồn ở stage 7, 3, 5; xóa một lần khi chạm đất.
// p01_simple, p02_simple vẫn giữ để chạy lại các kết quả cũ.
// BỎ các lớp an toàn như p01_simple: không máy trạng thái, không dry-run, không chờ PX4 xác nhận,
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
//   circle_yaw: như figure8 nhưng bay đường tròn r = circle_radius (chu kỳ circle_period_s, circle_laps vòng) và
//          hướng mũi mong muốn nhảy lệch yaw_offset_deg (mặc định 179,4°) rồi quay với chu kỳ yaw_period_s (thí nghiệm P11 Mục 4.1)
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
#include "uav_research/circle_yaw_reference.hpp"
#include "uav_research/gz_ground_truth.hpp"
#include "uav_research/gz_wrench.hpp"
#include "uav_research/controller_interface.hpp"

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

class UavSimple : public rclcpp::Node
{
public:
    // Ctrl+C hoặc tự tắt: xóa nhiễu ngoài nếu còn, để chuyến sau không bị lực cũ.
    ~UavSimple() override
    {
        if (dist_active_) wrench_.clear();
    }

    UavSimple() : Node("uav_simple")
    {
        // Chỉ 2 tham số: chọn quỹ đạo và tên file CSV.
        mode_ = declare_parameter<std::string>("reference_mode", "hover");   // hover | case1 | figure8 | circle_yaw
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
        // Luôn đăng ký topic Gazebo. state_source:=gazebo → dùng cho điều khiển.
        // state_source:=px4 → CHỈ ghi CSV (cột gt_*) để đo sai số THẬT khi bộ điều khiển chạy bằng EKF2;
        // không đưa vào bộ điều khiển. Chưa gắn odometry Gazebo thì các cột gt_* là nan.
        const bool gt_ok = gt_.start(gt_topic);
        if (use_gt_ && !gt_ok)
            RCLCPP_ERROR(get_logger(), "Không đăng ký được topic Gazebo %s", gt_topic.c_str());
        case1_duration_ = CASE1_RAMP + CASE1_PHASE / case1_speed_;   // 30 s ở 0,25×; 17 s ở 0,5×; 10,5 s ở 1×
        // Quỹ đạo số 8 (reference_mode:=figure8): dọc = a·sin θ, ngang = b·sin 2θ, θ = 2π·t/period.
        // Giảm fig8_period_s để bay gắt hơn (gia tốc tỉ lệ 1/period²).
        fig8_a_ = declare_parameter<double>("fig8_a", 1.5);              // nửa chiều dài dọc hướng mũi (m)
        fig8_b_ = declare_parameter<double>("fig8_b", 0.75);             // biên độ ngang (m)
        fig8_period_ = declare_parameter<double>("fig8_period_s", 10.0); // thời gian một vòng số 8 (s)
        fig8_laps_ = static_cast<int>(declare_parameter<int>("fig8_laps", 2));   // số vòng
        fig8_duration_ = figure8_duration(fig8_period_, fig8_laps_, CASE1_RAMP);
        // Đường tròn + yaw quay (reference_mode:=circle_yaw), theo thí nghiệm Mục 4.1 của P11.
        circ_r_ = declare_parameter<double>("circle_radius", 1.0);          // bán kính (m)
        circ_period_ = declare_parameter<double>("circle_period_s", 10.0);  // một vòng tròn (s)
        circ_laps_ = static_cast<int>(declare_parameter<int>("circle_laps", 2));
        yaw_offset_ = declare_parameter<double>("yaw_offset_deg", 179.4) * M_PI / 180.0;   // lệch yaw lúc bắt đầu
        yaw_period_ = declare_parameter<double>("yaw_period_s", 40.0);     // chu kỳ quay b1d; 0 = không quay
        circ_duration_ = circle_yaw_duration(circ_period_, circ_laps_, CASE1_RAMP);
        if (mode_ == "figure8")
            RCLCPP_INFO(get_logger(), "Quỹ đạo số 8: a=%.2f m, b=%.2f m, chu kỳ %.1f s, %d vòng, bay %.1f s",
                fig8_a_, fig8_b_, fig8_period_, fig8_laps_, fig8_duration_);
        const auto csv = declare_parameter<std::string>("log_csv", "");     // rỗng = không ghi CSV

        // Tham số vật lý X500 và hệ số điều khiển: lấy nguyên bộ đã bay tốt.
        // Tham số vật lý + hệ số điều khiển theo mô hình (x500: bộ đã bay tốt; x650: ước lượng, xem x500_control.hpp).
        params_ = vehicle_ == "x650" ? x650_parameters() : x500_parameters();
        motors_ = vehicle_ == "x650" ? x650_actuators() : X500Actuators{};
        // att_bw_scale (s): co băng thông vòng tư thế. kR nhân s², kOmega nhân s -> giữ hệ số tắt dần, tần số riêng nhân s.
        // Dùng khi motor chậm (x650) làm vòng tư thế dao động.
        // Mặc định: x500 = 1,0; x650 = 0,7 (s = 1 làm roll dao động 2,2 Hz ở Case I 1x, xem x650_p01_case1x10_g1 / _bw07_g1).
        att_bw_scale_ = declare_parameter<double>("att_bw_scale", vehicle_ == "x650" ? 0.7 : 1.0);
        if (!(att_bw_scale_ > 0.0)) throw std::invalid_argument("att_bw_scale phải > 0");
        params_.attitude_scale *= att_bw_scale_ * att_bw_scale_;
        params_.rate_scale *= att_bw_scale_;
        // mass_kg: khối lượng BỘ ĐIỀU KHIỂN tin là đúng (UAV thật trong Gazebo: x500 2,064 kg; x650 3,0 kg).
        // Khai sai = tạo nhiễu lực cố định để thử các bộ điều khiển. Hệ số kx, kv tính theo gain_mass_kg (bên dưới).
        const double m_danh_dinh = params_.m;   // khối lượng danh định của mô hình đã chọn (vehicle)
        params_.m = declare_parameter<double>("mass_kg", params_.m);
        // gain_mass_kg: khối lượng dùng để TÍNH HỆ SỐ k_x = 4·m, k_v = 3,2·m (và k_i mặc định của P02).
        // Mặc định = khối lượng danh định → khai sai mass_kg chỉ làm sai phần mô hình (bù trọng lực, bù đón trước),
        // hệ số phản hồi giữ nguyên → so sánh "khai đúng/khai sai" chỉ khác đúng phần bất định.
        // (Trước 25/11/2026 17:30, hệ số tính theo mass_kg; muốn tái lập: -p gain_mass_kg:=<mass_kg>.)
        gain_mass_ = declare_parameter<double>("gain_mass_kg", m_danh_dinh);
        params_.kx = 4.0 * gain_mass_;
        params_.kv = 3.2 * gain_mass_;
        // motor_k_scale: hệ số lực motor mà BỘ ĐIỀU KHIỂN tin = k thật (Gazebo) × scale.
        // 1,1 = tin motor khỏe hơn 10% → lệnh u thấp hơn cần thiết → mỗi rotor chỉ ra ~91% lực yêu cầu.
        // Mô phỏng sai đường cong lực/bàn đo lực trên máy bay thật: sai cả lực tổng lẫn mômen.
        const double motor_k_scale = declare_parameter<double>("motor_k_scale", 1.0);
        motors_.motor_constant *= motor_k_scale;

        // antiwindup: ngừng cập nhật trạng thái bên trong (tích phân P02, θ̄ của P06/P11) ở bước ngay sau
        // một bước motor bão hòa (tích phân có điều kiện). Bổ sung của đồ án; mặc định tắt để giữ kết quả cũ.
        antiwindup_ = declare_parameter<bool>("antiwindup", false);

        // NHIỄU NGOÀI (Gazebo ApplyLinkWrench, xem gz_wrench.hpp): lực/mômen hằng bật tại t = dist_t_on, tắt tại
        // dist_t_off (t tính từ lúc bắt đầu cất cánh, như CSV). Hệ NED thế giới: dist_fz > 0 = kéo xuống.
        // payload_kg: tiện dùng, cộng payload_kg·g vào dist_fz (chỉ trọng lượng, không có khối lượng quán tính).
        // dist_t_on < 0 = tắt (mặc định). Nhiễu GIỮ NGUYÊN qua cả lúc hạ cánh, chỉ xóa khi đã chạm đất
        // (25/11/2026: trước đây xóa lúc bắt đầu hạ → tích phân P02 / θ̄ đang giữ bù lực cũ (ngừng cập nhật khi hạ)
        //  và P01 mất lệch cố định → UAV vọt lên ~40 cm hoặc trôi ngang ~35 cm khi hạ cánh).
        {
            const double payload = declare_parameter<double>("payload_kg", 0.0);
            dist_f_ = Vector3(declare_parameter<double>("dist_fx", 0.0), declare_parameter<double>("dist_fy", 0.0),
                              declare_parameter<double>("dist_fz", 0.0) + payload * 9.81);
            dist_t_ = Vector3(declare_parameter<double>("dist_tx", 0.0), declare_parameter<double>("dist_ty", 0.0),
                              declare_parameter<double>("dist_tz", 0.0));
            dist_on_ = declare_parameter<double>("dist_t_on", -1.0);
            dist_off_ = declare_parameter<double>("dist_t_off", -1.0);
            use_dist_ = dist_on_ >= 0.0 && (dist_f_.norm() > 0.0 || dist_t_.norm() > 0.0);
            if (use_dist_)
            {
                wrench_.start("default", vehicle_ + "_0");
                RCLCPP_INFO(get_logger(), "Nhiễu ngoài: F_NED=(%.2f,%.2f,%.2f) N, T=(%.3f,%.3f,%.3f) N.m, bật t=%.1f s, tắt t=%.1f s",
                    dist_f_.x(), dist_f_.y(), dist_f_.z(), dist_t_.x(), dist_t_.y(), dist_t_.z(), dist_on_, dist_off_);
            }
        }
        // Chọn bộ điều khiển. Tham số riêng của từng bộ chỉ được khai báo khi chọn bộ đó.
        const auto controller = declare_parameter<std::string>("controller", "p01");
        if (controller == "p01")
        {
            ctrl_ = std::make_unique<P01Ctrl>();
        }
        else if (controller == "p02")
        {
            // Hệ số P02 (mặc định xem P02Gains). ki mặc định = 1.0 * gain_mass_kg.
            P02Gains g;
            g.ki = declare_parameter<double>("ki", 1.0 * gain_mass_);
            g.c1 = declare_parameter<double>("c1", g.c1);
            g.sigma = declare_parameter<double>("sigma", g.sigma);
            g.c2 = declare_parameter<double>("c2", g.c2);
            const double kI_xy = declare_parameter<double>("kI_xy", g.kI.x());
            const double kI_z = declare_parameter<double>("kI_z", g.kI.z());
            g.kI = Vector3(kI_xy, kI_xy, kI_z);
            // use_integral:=false → tắt cộng dồn tích phân (còn lại khác P01 ở dạng mômen (26)).
            ctrl_ = std::make_unique<P02Ctrl>(g, declare_parameter<bool>("use_integral", true));
        }
        else if (controller == "p06" || controller == "p11")
        {
            // Hệ số thích nghi. c1, c2, c3 mặc định tính từ điều kiện của bài với hệ số hiện tại (xem default_gains).
            AdaptiveGains g = AdaptiveCtrl::default_gains(params_);
            g.gamma_x = declare_parameter<double>("gamma_x", g.gamma_x);
            g.gamma_R = declare_parameter<double>("gamma_R", g.gamma_R);
            g.B_theta_x = declare_parameter<double>("B_theta_x", g.B_theta_x);
            g.B_theta_R = declare_parameter<double>("B_theta_R", g.B_theta_R);   // chống windup θ̄_R; mặc định tắt
            g.c1 = declare_parameter<double>("c1", g.c1);
            g.c2 = declare_parameter<double>("c2", g.c2);
            g.c3 = declare_parameter<double>("c3", g.c3);
            ctrl_ = std::make_unique<AdaptiveCtrl>(controller == "p11", g);
        }
        else
        {
            throw std::invalid_argument("controller phải là p01, p02, p06 hoặc p11");
        }

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
                    "Mx,My,Mz,Mx_achieved,My_achieved,Mz_achieved,u0,u1,u2,u3,saturated,stage,control_blend"
                 << ctrl_->csv_header() << ",state_source_gazebo,ekf_x,ekf_y,ekf_z,ekf_tilt_err_deg,dist_active"
                 << ",gt_x,gt_y,gt_z,gt_pos_err,gt_tilt_err_deg\n";
        }

        // 1 timer 100 Hz: TOÀN BỘ việc điều khiển nằm trong loop().
        start_ = std::chrono::steady_clock::now();
        last_loop_ = start_;
        timer_ = create_wall_timer(std::chrono::milliseconds(10), [this]() { loop(); });
        RCLCPP_INFO(get_logger(), "uav_simple: controller=%s vehicle=%s att_bw_scale=%.2f reference_mode=%s mass_kg=%.3f gain_mass_kg=%.3f case1_speed=%.2f motor_k_scale=%.2f antiwindup=%d %s (CHỈ DÙNG TRONG MÔ PHỎNG)",
            ctrl_->name().c_str(), vehicle_.c_str(), att_bw_scale_, mode_.c_str(), params_.m, gain_mass_, case1_speed_, motor_k_scale, antiwindup_, ctrl_->describe().c_str());
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
        px4_x_ = state_.x;
        px4_R_ = state_.R;
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

        // 1b. Trạng thái thật từ Gazebo (state_source:=gazebo): thay x, v, R; giữ omega từ con quay PX4.
        //     Chưa có dữ liệu thì giữ đồng hồ node ở 0 (chưa cất cánh) và chờ.
        // 1a. Chế độ EKF2: đọc trạng thái thật chỉ để ghi CSV (không dùng điều khiển).
        if (!use_gt_)
        {
            Vector3 vg_log;
            double age_log = 0.0;
            have_gt_log_ = gt_.get(gt_x_, vg_log, gt_R_, age_log) && age_log < 0.2;
        }

        if (use_gt_)
        {
            Vector3 xg, vg;
            Matrix3 Rg;
            double age = 0.0;
            if (!gt_.get(xg, vg, Rg, age))
            {
                start_ = tick;
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
        const bool is_traj = (mode_ == "case1" || mode_ == "figure8" || mode_ == "circle_yaw");   // có bay quỹ đạo
        const double traj_duration = (mode_ == "figure8") ? fig8_duration_ : (mode_ == "circle_yaw" ? circ_duration_ : case1_duration_);
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
                if (mode_ == "circle_yaw")
                    ref = reference_circle_yaw(t - t_active - SETTLE, top, heading_, circ_r_, circ_period_, circ_laps_, CASE1_RAMP,
                                               yaw_offset_, yaw_period_);
                else if (mode_ == "figure8")
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
                land_heading_ = last_ref_b1_;                   // giữ hướng mũi cuối (circle_yaw: khác hướng cất cánh)
                ground_ = land_start_;
                ground_.z() = origin_.z() + BELOW_GROUND;       // NED: z lớn hơn = thấp hơn
                landing_ = true;
                RCLCPP_INFO(get_logger(), "Bắt đầu hạ cánh");
            }
            // Dùng lại đa thức cất cánh nhưng chạy NGƯỢC thời gian (từ đỉnh về đất).
            // Đảo thời gian thì đạo hàm bậc lẻ (v, jerk) đổi dấu.
            const double tau = t - t_land;
            ref = reference_takeoff(LAND_TIME - tau, ground_, land_heading_, ground_.z() - land_start_.z(), LAND_TIME);
            ref.v = -ref.v;
            ref.j = -ref.j;
            stage = 4;   // ghi CSV: đoạn hạ cánh

            // Hết đa thức hạ cánh = đã chạm đất: xóa trạng thái bên trong bộ điều khiển (một lần), giống PX4 xóa tích phân khi đã đáp.
            // Nếu giữ, k_i·e_i (~2,6 N khi khai 1,8 kg) làm lực trên đất còn ≈ 97% trọng lượng, UAV gần như lơ lửng.
            if (tau >= LAND_TIME && !reset_on_ground_)
            {
                ctrl_->reset();
                reset_on_ground_ = true;
                RCLCPP_INFO(get_logger(), "Chạm đất: xóa trạng thái bên trong bộ điều khiển trước khi disarm");
                // Xóa nhiễu ngoài khi đã chạm đất: không còn thử bộ điều khiển; lực ngang để lại sẽ đẩy UAV trượt trên mặt đất
                // (đo 25/11: gió 2 N làm UAV trượt ~50 cm sau khi chạm đất).
                if (dist_active_) { wrench_.clear(); dist_active_ = false; dist_done_ = true; RCLCPP_INFO(get_logger(), "Tắt nhiễu ngoài (đã chạm đất)"); }
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
        // Nhiễu ngoài: bật đúng một lần (persistent cộng dồn), tắt đúng một lần.
        if (use_dist_ && !dist_active_ && !dist_done_ && !landing_ && t >= dist_on_)
        {
            if (wrench_.apply(dist_f_, dist_t_)) { dist_active_ = true; RCLCPP_INFO(get_logger(), "Bật nhiễu ngoài tại t=%.2f s", t); }
            else RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Chưa gửi được nhiễu ngoài (chưa có bên nhận wrench)");
        }
        if (dist_active_ && dist_off_ >= 0.0 && t >= dist_off_)
        {
            wrench_.clear(); dist_active_ = false; dist_done_ = true;
            RCLCPP_INFO(get_logger(), "Tắt nhiễu ngoài tại t=%.2f s", t);
        }
        last_ref_b1_ = ref.b1;

        // 5. Bộ điều khiển đã chọn: trạng thái + tham chiếu → lực f, mômen M.
        // Tích phân từ lúc bắt đầu cất cánh (stage 7) để khử lệch trước khi bay Case I.
        // Tích phân cả lúc bay về (stage 5). Không tích phân lúc chờ trước cất cánh (stage 0)
        // và lúc hạ cánh (stage 4, đích dưới mặt đất).
        const bool integrate = (stage == 7 || stage == 3 || stage == 5) && !(antiwindup_ && last_saturated_);
        const P01Control c = ctrl_->compute(state_, params_, ref, dt, integrate);

        // 6. Phân bổ: f, M → 4 lệnh rotor u0..u3 (có giới hạn).
        const X500Allocation a = allocate_x500(c.f, c.M, motors_);
        last_saturated_ = a.saturated;

        // 7. Gửi lệnh motor. Kênh 5–12 để NaN = không dùng.
        Motors m{};
        m.timestamp = stamp_us();
        m.timestamp_sample = sample_;
        m.control.fill(std::numeric_limits<float>::quiet_NaN());
        for (int i = 0; i < 4; ++i) m.control[i] = a.commands[i];
        motor_pub_->publish(m);

        // 8. Quan sát: telemetry, log 1 lần/giây, CSV.
        telemetry_->publish(sample_, get_clock()->now(), state_, ref, c);
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "t=%.1f stage=%d ex=%.3f f=%.2f%s saturated=%d",
            t, stage, c.ex.norm(), c.f, ctrl_->status().c_str(), a.saturated);
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
            ctrl_->csv_row(log_);
            // EKF2 so với trạng thái đang dùng: lệch vị trí và góc lệch trục z thân (nghiêng).
            log_ << ',' << use_gt_;
            for (int i = 0; i < 3; ++i) log_ << ',' << px4_x_[i];
            log_ << ',' << std::acos(std::clamp(px4_R_.col(2).dot(state_.R.col(2)), -1.0, 1.0)) * 180.0 / M_PI
                 << ',' << dist_active_;
            // Trạng thái THẬT và sai số thật so với tham chiếu (gazebo: trùng state_; px4: từ Gazebo nếu có, không thì nan).
            if (use_gt_)
            {
                for (int i = 0; i < 3; ++i) log_ << ',' << state_.x[i];
                log_ << ',' << (state_.x - ref.x).norm() << ",0";
            }
            else if (have_gt_log_)
            {
                for (int i = 0; i < 3; ++i) log_ << ',' << gt_x_[i];
                log_ << ',' << (gt_x_ - ref.x).norm() << ','
                     << std::acos(std::clamp(gt_R_.col(2).dot(state_.R.col(2)), -1.0, 1.0)) * 180.0 / M_PI;
            }
            else
                log_ << ",nan,nan,nan,nan,nan";
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
    double case1_speed_ = 0.25, case1_duration_ = 30.0;
    double fig8_a_ = 1.5, fig8_b_ = 0.75, fig8_period_ = 10.0, fig8_duration_ = 24.0;
    int fig8_laps_ = 2;
    bool return_home_ = true, returning_ = false;
    bool use_gt_ = false;
    GzGroundTruth gt_;
    GzWrench wrench_;
    Vector3 dist_f_ = Vector3::Zero(), dist_t_ = Vector3::Zero();
    double dist_on_ = -1.0, dist_off_ = -1.0;
    bool use_dist_ = false, dist_active_ = false, dist_done_ = false;
    Vector3 px4_x_ = Vector3::Zero();          // vị trí EKF2 (ghi CSV để so với trạng thái thật)
    Matrix3 px4_R_ = Matrix3::Identity();      // tư thế EKF2
    bool have_gt_log_ = false;                 // chế độ EKF2: có trạng thái thật để ghi CSV
    Vector3 gt_x_ = Vector3::Zero();           // vị trí thật (chỉ ghi CSV)
    Matrix3 gt_R_ = Matrix3::Identity();       // tư thế thật (chỉ ghi CSV)
    Vector3 return_from_ = Vector3::Zero();
    std::string vehicle_;
    double att_bw_scale_ = 1.0;
    P01Parameters params_;
    std::unique_ptr<Controller> ctrl_;   // bộ điều khiển đã chọn (P01Ctrl, P02Ctrl, ...)
    bool antiwindup_ = false, last_saturated_ = false;
    double gain_mass_ = 0.0;
    bool reset_on_ground_ = false;   // đã xóa tích phân lúc chạm đất chưa
    X500Actuators motors_;
    P01State state_;
    Vector3 origin_ = Vector3::Zero(), heading_ = Vector3::UnitX();
    Vector3 land_heading_ = Vector3::UnitX(), last_ref_b1_ = Vector3::UnitX();
    double circ_r_ = 1.0, circ_period_ = 10.0, circ_duration_ = 24.0, yaw_offset_ = 0.0, yaw_period_ = 40.0;
    int circ_laps_ = 2;
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
    rclcpp::spin(std::make_shared<UavSimple>());
    if (rclcpp::ok()) rclcpp::shutdown();   // Ctrl+C: dọn dẹp; tự kết thúc thì đã shutdown rồi
    return 0;
}
