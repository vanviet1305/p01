// NODE ĐIỀU PHỐI P01 TRÊN PX4 SITL + GAZEBO X500
// Đường tính chính trong odometry(): PX4 -> trạng thái -> tham chiếu -> P01(f,M)
// -> máy trạng thái mặt đất -> chuyển dần đầu ra -> phân bổ rotor -> ActuatorMotors.
//
// Thứ tự đọc để học: constructor -> odometry -> advance -> publish_motors.
// README.md giải thích toàn bộ hệ thống, các tệp thành phần và lần bay đã chạy.
// Package ROS tên p01; namespace/include uav_research giữ từ nguồn mã tham khảo.
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <px4_msgs/msg/actuator_motors.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <Eigen/Geometry>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include "uav_research/p01_ground_session.hpp"
#include "uav_research/p01_takeoff_reference.hpp"
#include "uav_research/px4_topic.hpp"
#include "uav_research/x500_control.hpp"
#include "uav_research/x500_reference.hpp"
#include "uav_research/p01_telemetry.hpp"


// Phiên mới bắt đầu trên mặt đất khi UAV chưa arm và chưa ở Offboard.
// Node chuẩn bị Offboard -> yêu cầu arm -> tăng đầu ra rotor -> P01 lên cao.
// QGroundControl giám sát; trạng thái PX4 xác nhận việc thực hiện các yêu cầu.
// Mặc định dry-run (chỉ tính); --ros-args -p enable_actuation:=true mới cho phát lệnh.
class P01OffboardHover : public rclcpp::Node
{
    // using đặt bí danh kiểu C++ để giảm tên dài; chưa tạo đối tượng nào ở đây.
    // steady_clock đo thời gian chờ/mất kết nối, không bị chỉnh theo giờ hệ thống.
    using Clock = std::chrono::steady_clock;
    using Session = uav_research::P01GroundSession;
    using Odom = px4_msgs::msg::VehicleOdometry;
    using Status = px4_msgs::msg::VehicleStatus;
    using Command = px4_msgs::msg::VehicleCommand;
    using Ack = px4_msgs::msg::VehicleCommandAck;
    using Mode = px4_msgs::msg::OffboardControlMode;
    using Motors = px4_msgs::msg::ActuatorMotors;
    using Land = px4_msgs::msg::VehicleLandDetected;
public:
    P01OffboardHover() : Node("p01_offboard_hover")
    {
        // 1. hover: lên cao rồi giữ vị trí. case1: lên cao, ổn định rồi bám quỹ đạo.
        enabled_ = parameter<bool>("enable_actuation", false);
        mode_ = parameter<std::string>("reference_mode", "hover");
        if (mode_ != "hover" && mode_ != "case1")
            throw std::invalid_argument("reference_mode must be hover or case1");
        takeoff_height_ = positive("takeoff_height_m", 2.0);
        takeoff_duration_ = positive("takeoff_duration_s", 6.0);
        if (takeoff_duration_ > 15.0)
            throw std::invalid_argument("takeoff_duration_s must be <= 15; takeoff timeout is 20 s");
        // Nhóm tham số này chỉ ảnh hưởng khi chọn case1. duration không phải
        // thời hạn tự dừng node; hết quỹ đạo thì giữ điểm cuối. hover không tự hết giờ.
        speed_ = positive("trajectory_speed", 0.25);
        ramp_ = positive("trajectory_ramp_s", 4.0);
        duration_ = positive("trajectory_duration_s", 30.0);
        settle_ = positive("settle_time_s", 3.0);
        rotate_heading_ = parameter<bool>("trajectory_rotate_heading", false);
        if (duration_ < 2*ramp_) throw std::invalid_argument("duration must be >= 2*ramp");
        // 2. Cấu hình vật lý/hệ số X500. Các tham số --ros-args có thể ghi đè
        // khi khởi động; số mặc định không phải tham số UAV 4.34 kg của bài CDC.
        parameters_ = uav_research::x500_parameters();
        parameters_.m = positive("mass_kg", parameters_.m);
        parameters_.kx = positive("position_gain_per_mass", 4.0)*parameters_.m;
        parameters_.kv = positive("velocity_gain_per_mass", 3.2)*parameters_.m;
        parameters_.attitude_scale = vector_parameter("attitude_gains", parameters_.attitude_scale);
        parameters_.rate_scale = vector_parameter("rate_gains", parameters_.rate_scale);
        parameters_.J = vector_parameter("inertia_diagonal", parameters_.J.diagonal()).asDiagonal();
        actuators_.arm = positive("rotor_arm_m", actuators_.arm);
        actuators_.moment_ratio = positive("rotor_moment_ratio", actuators_.moment_ratio);
        actuators_.motor_constant = positive("motor_constant", actuators_.motor_constant);
        actuators_.omega_min = parameter<double>("motor_speed_min", 150.0);
        actuators_.omega_max = parameter<double>("motor_speed_max", 1000.0);
        actuators_.thrust_factor = parameter<double>("motor_thrust_factor", 0.0);
        actuators_.validate();
        if (parameters_.m*parameters_.g >= 4*actuators_.max_thrust())
            throw std::invalid_argument("Configured motors cannot support vehicle weight");

        // 3. Tên topic (kênh bản tin) theo phiên bản px4_msgs đã được biên dịch.
        // /fmu/out: PX4 gửi ra ROS; /fmu/in: ROS gửi vào PX4.
        const auto odom_topic = uav_research::px4_topic<Odom>("/fmu/out/vehicle_odometry");
        const auto status_topic = uav_research::px4_topic<Status>("/fmu/out/vehicle_status");
        const auto command_topic = uav_research::px4_topic<Command>("/fmu/in/vehicle_command");
        telemetry_ = std::make_unique<uav_research::P01Telemetry>(*this);
        
        if (enabled_)
        {
            // Publisher là bộ phát. Chỉ tạo các bộ phát lệnh khi đã bật actuation;
            // dry-run không tạo chúng, nên không tranh quyền điều khiển UAV.
            motor_pub_ = create_publisher<Motors>(
                uav_research::px4_topic<Motors>("/fmu/in/actuator_motors"), rclcpp::SensorDataQoS());
            mode_pub_ = create_publisher<Mode>(
                uav_research::px4_topic<Mode>("/fmu/in/offboard_control_mode"), 10);
            command_pub_ = create_publisher<Command>(command_topic, 10);
        }
        // 4. Subscriber là bộ nhận; callback là hàm được gọi khi nhận bản tin.
        // [this] cho lambda truy cập node hiện tại; SharedPtr quản lý vòng đời message.
        // SensorDataQoS dùng chính sách best-effort (không chờ gửi lại mọi gói),
        // tương thích luồng dữ liệu PX4. Đây là phản hồi từ bộ ước lượng PX4,
        // không phải đọc trực tiếp trạng thái thật tuyệt đối của động cơ vật lý Gazebo.
        odom_sub_ = create_subscription<Odom>(odom_topic, rclcpp::SensorDataQoS(),
            [this](Odom::SharedPtr msg) { odometry(*msg); });
        status_sub_ = create_subscription<Status>(status_topic, rclcpp::SensorDataQoS(),
            [this](Status::SharedPtr msg)
            {
                // VehicleStatus xác nhận mode/armed/failsafe; đầu ra motor còn
                // phụ thuộc giai đoạn và hệ số chuyển dần của GroundSession.
                status_ = *msg;
                status_time_ = Clock::now();
                have_status_ = true;
                const auto actions = advance();
                if (actions.motors) publish_motors();
            });
        // Nhận kết luận mặt đất từ PX4 để phục vụ quy trình tự cất cánh.
        land_sub_ = create_subscription<Land>(
            uav_research::px4_topic<Land>("/fmu/out/vehicle_land_detected"),
            rclcpp::SensorDataQoS(), [this](Land::ConstSharedPtr msg)
            {
                const bool changed = !have_land_ ||
                    msg->landed != land_state_.landed ||
                    msg->ground_contact != land_state_.ground_contact ||
                    msg->freefall != land_state_.freefall;
                land_state_ = *msg;
                land_time_ = Clock::now();
                have_land_ = true;
                if (changed)
                    RCLCPP_INFO(get_logger(), "Ground feedback: landed=%d contact=%d freefall=%d",
                        static_cast<int>(land_state_.landed),
                        static_cast<int>(land_state_.ground_contact),
                        static_cast<int>(land_state_.freefall));
            });
        // ACK chỉ ghi kết quả tiếp nhận lệnh. GroundSession đợi VehicleStatus
        // xác nhận mode và armed; không suy trạng thái chỉ từ ACK=0.
        ack_sub_ = create_subscription<Ack>(
            uav_research::px4_topic<Ack>("/fmu/out/vehicle_command_ack"),
            rclcpp::SensorDataQoS(), [this](Ack::SharedPtr msg)
            {
                if (msg->command == Command::VEHICLE_CMD_DO_SET_MODE ||
                    msg->command == Command::VEHICLE_CMD_COMPONENT_ARM_DISARM)
                    RCLCPP_INFO(get_logger(), "Command ACK: command=%u result=%u (0=accepted)",
                        static_cast<unsigned>(msg->command), static_cast<unsigned>(msg->result));
            });
        // 5. CSV ghi trạng thái/tham chiếu, yêu cầu P01 và kết quả phân bổ TÍNH TOÁN.
        // f_achieved, M*_achieved không phải cảm biến đo lực Gazebo. Chọn stage=3
        // khi phân tích đoạn P01 điều khiển, tránh trộn mẫu trước/sau nhận quyền.
        const auto csv = parameter<std::string>("log_csv", "");
        if (!csv.empty())
        {
            log_.open(csv);
            if (!log_) throw std::runtime_error("Cannot open log_csv: "+csv);
            log_ << "t,x,y,z,xd,yd,zd,position_error,Psi,f_requested,f_achieved,"
                    "Mx,My,Mz,Mx_achieved,My_achieved,Mz_achieved,u0,u1,u2,u3,saturated,stage,control_blend\n";
            log_ << std::setprecision(10);
        }
        // 6. Timer 100 ms = 10 Hz phụ trách heartbeat (tín hiệu duy trì Offboard),
        // kiểm tra phiên và đẩy CSV ra đĩa. Vòng tính lực/mômen chạy theo odometry,
        // không phải chỉ 10 Hz. Log trên màn hình lại được giới hạn khoảng 1 Hz.
        timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]()
        {
            if (enabled_)
                // Dò nguồn phát ROS trùng sau khi ROS discovery nhìn thấy chúng.
                // Không phát hiện mọi nguồn MAVLink ngoài ROS hoặc xung đột chưa được khám phá.
                exclusive_ = count_publishers(mode_pub_->get_topic_name()) <= 1 &&
                             count_publishers(motor_pub_->get_topic_name()) <= 1;
            const auto actions = advance();
            if (actions.heartbeat)
            {
                Mode msg{};
                msg.timestamp = timestamp_us();
                // Message khởi tạo {} có các cờ khác false. direct_actuator chọn
                // đầu vào động cơ, bỏ qua các vòng vị trí/tư thế của PX4.
                // PX4 vẫn chạy ước lượng trạng thái, quản lý chế độ và xuất động cơ.
                msg.direct_actuator = true;
                mode_pub_->publish(msg);
            }
            if (actions.motors) publish_motors();
            if (session_.stage() == Session::Stage::Waiting || !enabled_)
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                    "Waiting/dry-run: status=%d odom=%d valid=%d rate_ok=%d land_fresh=%d ground_ready=%d enabled=%d",
                    have_status_, have_odom_, motor_valid_, rate_ok_,
                    land_feedback_ok(), ready_on_ground(), enabled_);
            if (log_) log_.flush();
        });
        RCLCPP_INFO(get_logger(), "X500 %s, reference=%s; P01 takeoff +%.2f m / %.2f s.",
            enabled_ ? "ACTUATION ENABLED" : "DRY-RUN", mode_.c_str(),
            takeoff_height_, takeoff_duration_);
        RCLCPP_INFO(get_logger(), "Topics: odom=%s status=%s command=%s", odom_topic.c_str(),
            status_topic.c_str(), command_topic.c_str());
    }
private:
    template<typename T> T parameter(const std::string &name, const T &value)
    {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        // --ros-args ghi đè lúc khởi động được; không cho đổi tùy tiện giữa phiên bay.
        descriptor.read_only = true;
        return declare_parameter<T>(name, value, descriptor);
    }
    double positive(const std::string &name, double value)
    {
        // Chặn cấu hình NaN/vô hạn/không dương trước khi bắt đầu tính điều khiển.
        const double out = parameter<double>(name, value);
        if (!std::isfinite(out) || out <= 0) throw std::invalid_argument(name+" must be finite and >0");
        return out;
    }
    Eigen::Vector3d vector_parameter(const std::string &name, const Eigen::Vector3d &value)
    {
        // ROS truyền danh sách số; kiểm tra đủ ba phần tử rồi chuyển sang vector Eigen.
        const auto v = parameter<std::vector<double>>(name, {value.x(), value.y(), value.z()});
        if (v.size() != 3) throw std::invalid_argument(name+" needs 3 values");
        const Eigen::Vector3d out(v[0], v[1], v[2]);
        if (!out.allFinite() || out.minCoeff() <= 0) throw std::invalid_argument(name+" must be positive");
        return out;
    }
    // timestamp_us dùng đồng hồ ROS để điền bản tin; wall_seconds dùng steady_clock
    // cho thời hạn chờ. Thời gian quỹ đạo t phía dưới lấy hiệu timestamp_sample PX4.
    // Không so trừ trực tiếp hai đồng hồ có gốc khác nhau.
    std::uint64_t timestamp_us() { return get_clock()->now().nanoseconds()/1000; }
    double wall_seconds() const { return std::chrono::duration<double>(Clock::now()-epoch_).count(); }
    bool feedback_ok() const
    {
        // Tuổi dữ liệu đo bằng thời điểm nhận trên máy: status <1 s, odom <150 ms.
        // rate_ok kiểm khoảng timestamp mẫu, motor_valid kiểm phép tính/phân bổ.
        const auto now = Clock::now();
        return have_status_ && have_odom_ && motor_valid_ && rate_ok_ &&
            std::chrono::duration<double>(now-status_time_).count() < 1.0 &&
            std::chrono::duration<double>(now-odom_time_).count() < 0.15;
    }
    bool land_feedback_ok() const
    {
        return have_land_ &&
            std::chrono::duration<double>(Clock::now() - land_time_).count() < 2.5;
    }
    bool ready_on_ground() const
    {
        // Chỉ là điều kiện bắt đầu: sau arm, GroundSession dùng các cờ khác.
        return feedback_ok() && land_feedback_ok() && land_state_.landed &&
            !land_state_.freefall && !status_.failsafe &&
            status_.arming_state == Status::ARMING_STATE_DISARMED &&
            state_.v.norm() < 0.2 && state_.omega.norm() < 0.2 && state_.R(2,2) > 0.98;
    }
    double takeoff_time() const
    {
        if (takeoff_sample_ == 0 || last_sample_ < takeoff_sample_) return 0.0;
        return (last_sample_ - takeoff_sample_) * 1e-6;
    }
    uav_research::P01Reference current_reference() const
    {
        if (takeoff_sample_ == 0)
            return uav_research::reference_hover(origin_, heading_);
        if (mode_ == "case1" && active_sample_ != 0 && last_sample_ >= active_sample_)
        {
            Eigen::Vector3d top = origin_;
            top.z() -= takeoff_height_;
            const double trajectory_time = (last_sample_ - active_sample_) * 1e-6;
            return uav_research::reference_x500_case1(
                trajectory_time - settle_, top, heading_, speed_, ramp_, duration_, rotate_heading_);
        }
        return uav_research::reference_takeoff(
            takeoff_time(), origin_, heading_, takeoff_height_, takeoff_duration_);
    }
    bool takeoff_complete()
    {
        Eigen::Vector3d target = origin_;
        target.z() -= takeoff_height_;
        const bool settled = session_.stage() == Session::Stage::Climbing &&
            takeoff_sample_ != 0 && takeoff_time() >= takeoff_duration_ &&
            feedback_ok() && land_feedback_ok() && !land_state_.landed &&
            (state_.x - target).norm() < 0.15 && state_.v.norm() < 0.2 &&
            state_.omega.norm() < 0.3 && state_.R(2,2) > 0.98;
        if (!settled)
        {
            settled_sample_ = 0;
            return false;
        }
        if (settled_sample_ == 0) settled_sample_ = last_sample_;
        // Dùng thời gian mẫu PX4 để yêu cầu ổn định liên tục ít nhất 1 giây.
        return last_sample_ >= settled_sample_ && last_sample_ - settled_sample_ >= 1000000;
    }
    uav_research::GroundActions advance()
    {
        uav_research::GroundInputs in;
        in.enabled = enabled_;
        in.feedback_ok = feedback_ok();
        in.land_ok = land_feedback_ok();
        in.ready_ground = ready_on_ground();
        in.landed = have_land_ && land_state_.landed;
        in.armed = have_status_ && status_.arming_state == Status::ARMING_STATE_ARMED;
        in.failsafe = have_status_ && status_.failsafe;
        in.offboard = have_status_ && status_.nav_state == Status::NAVIGATION_STATE_OFFBOARD;
        in.exclusive = exclusive_;
        in.takeoff_complete = takeoff_complete();
        const auto before = session_.stage();
        const auto actions = session_.step(wall_seconds(), in);
        motor_blend_ = actions.control_blend;
        motor_gate_ = actions.motors;
        if (actions.start_takeoff)
        {
            takeoff_sample_ = last_sample_;
            settled_sample_ = 0;
        }
        if (before != session_.stage())
        {
            RCLCPP_INFO(get_logger(), "Session stage=%d: %s", static_cast<int>(session_.stage()), session_.reason());
            if (session_.stage() == Session::Stage::Active) active_sample_ = last_sample_;
        }
        // Mã chế độ riêng của PX4: main=6 là Offboard; main=4/sub=3 là Auto Loiter (Hold).
        // Đây là lệnh DO_SET_MODE, không phải gửi tọa độ hay lực.
        if (actions.request_offboard) command_mode(6, 0);
        if (actions.request_arm) command_arm();
        if (actions.request_hold) command_mode(4, 3);
        return actions;
    }
    void command_mode(float mode, float submode)
    {
        if (!enabled_ || !command_pub_) return;
        Command msg{};
        msg.timestamp = timestamp_us();
        msg.command = Command::VEHICLE_CMD_DO_SET_MODE;
        msg.param1 = 1;
        msg.param2 = mode;
        msg.param3 = submode;
        msg.target_system = msg.source_system = 1;
        msg.target_component = msg.source_component = 1;
        msg.from_external = true;
        command_pub_->publish(msg);
        RCLCPP_INFO(get_logger(), "Requested PX4 mode %.0f / %.0f", mode, submode);
    }
    void command_arm()
    {
        // Arm thường: PX4 vẫn kiểm tra điều kiện của nó, không dùng force arm.
        if (!enabled_ || !command_pub_) return;
        Command msg{};
        msg.timestamp = timestamp_us();
        msg.command = Command::VEHICLE_CMD_COMPONENT_ARM_DISARM;
        msg.param1 = 1.0f;
        msg.param2 = 0.0f;
        msg.target_system = msg.source_system = 1;
        msg.target_component = msg.source_component = 1;
        msg.from_external = true;
        command_pub_->publish(msg);
        RCLCPP_INFO(get_logger(), "Requested PX4 arm");
    }
    void publish_motors()
    {
        if (!enabled_ || !motor_pub_ || !motor_gate_ || !motor_valid_) return;
        // Trộn trong đơn vị lực/mômen, rồi mới phân bổ và chuẩn hóa lệnh rotor.
        // control_blend=0: mức rotor tối thiểu; =1: toàn bộ yêu cầu P01.
        const double idle_f = 4.0 * actuators_.min_thrust();
        const double applied_f = idle_f + motor_blend_ * (control_.f - idle_f);
        const Eigen::Vector3d applied_M = motor_blend_ * control_.M;
        allocation_ = uav_research::allocate_x500(applied_f, applied_M, actuators_);
        Motors msg{};
        msg.timestamp = timestamp_us();
        msg.timestamp_sample = last_sample_;
        // Các kênh ngoài bốn motor X500 không được điều khiển.
        msg.control.fill(std::numeric_limits<float>::quiet_NaN());
        for (int i = 0; i < 4; ++i) msg.control[i] = allocation_.commands[i];
        motor_pub_->publish(msg);
    }
    void odometry(const Odom &msg)
    {
        // ĐÂY LÀ VÒNG TÍNH ĐIỀU KHIỂN: chạy mỗi khi có mẫu odometry mới.
        // A. Chặn dữ liệu sai trước khi dùng. NED là Bắc-Đông-Xuống;
        // khi đang bay trên gốc cục bộ khoảng 3 m, z thường xấp xỉ -3 m.
        motor_valid_ = false;
        if (msg.pose_frame != Odom::POSE_FRAME_NED || msg.velocity_frame != Odom::VELOCITY_FRAME_NED)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Expected NED pose and velocity frames");
            advance();
            return;
        }
        // B. Ghép trạng thái P01: x/v ở NED, omega ở hệ thân FRD (trước-phải-xuống).
        // PX4 q theo thứ tự (w,x,y,z), trùng thứ tự constructor Quaterniond của Eigen.
        // Sau chuẩn hóa, R đổi vectơ thân sang NED. Không dùng thứ tự (x,y,z,w)
        // của geometry_msgs/Quaternion để đọc nhầm mảng q của px4_msgs.
        uav_research::P01State next;
        next.x = Eigen::Vector3d(msg.position[0], msg.position[1], msg.position[2]);
        next.v = Eigen::Vector3d(msg.velocity[0], msg.velocity[1], msg.velocity[2]);
        next.omega = Eigen::Vector3d(msg.angular_velocity[0], msg.angular_velocity[1], msg.angular_velocity[2]);
        Eigen::Quaterniond q(msg.q[0], msg.q[1], msg.q[2], msg.q[3]);
        if (!next.x.allFinite() || !next.v.allFinite() || !next.omega.allFinite() ||
            !q.coeffs().allFinite() || q.norm() < 1e-6) { advance(); return; }
        const auto sample = msg.timestamp_sample != 0 ? msg.timestamp_sample : msg.timestamp;
        if (sample == 0 || (have_odom_ && sample <= last_sample_)) { advance(); return; }
        // Mẫu phải đi tới; khoảng timestamp >50 ms bị coi là phản hồi không đạt.
        // Các log hover vừa chạy có tốc độ ghi trung bình khoảng 100 mẫu/s.
        rate_ok_ = have_odom_ && (sample-last_sample_) <= 50000;
        if (have_odom_ && msg.reset_counter != reset_counter_)
        {
            // Bộ ước lượng đổi gốc thì mục tiêu cũ có thể không còn cùng hệ trục.
            // Báo mất phản hồi cho phiên để dừng thay vì tiếp tục bám tọa độ cũ.
            have_odom_ = false;
            advance();
            return;
        }
        reset_counter_ = msg.reset_counter;
        q.normalize();
        next.R = q.toRotationMatrix();
        state_ = next;
        last_sample_ = sample;
        have_odom_ = true;
        odom_time_ = Clock::now();
        // C. Gốc cất cánh: cập nhật khi Waiting, cố định từ Priming trở đi.
        // Dry-run vẫn chỉ chốt mẫu đầu tiên và không chạy tham chiếu lên cao.
        const auto stage = session_.stage();
        if (!anchor_set_ || (enabled_ && stage == Session::Stage::Waiting))
        {
            origin_ = state_.x;
            heading_ = uav_research::reference_hover(origin_, state_.R.col(0)).b1;
            anchor_set_ = true;
        }
        // t trong CSV/log mới là giây kể từ bắt đầu Climbing, không phải pha Case I.
        const double t = takeoff_time();
        try
        {
            // D. Mặt đất -> lên cao -> giữ trên cao, hoặc Case I sau ổn định.
            const auto ref = current_reference();
            // E. P01 biến trạng thái/tham chiếu thành yêu cầu f,M (N và N.m).
            const auto control = uav_research::compute_p01_control_with_reference(state_, parameters_, ref);
            control_ = control;
            // F. Ánh xạ yêu cầu đó sang lực và lệnh bốn rotor theo giới hạn X500.
            allocation_ = uav_research::allocate_x500(control.f, control.M, actuators_);
            motor_valid_ = true;
            // G. Phép tính hợp lệ chưa đủ để phát: phải có quyền từ máy trạng thái.
            const auto actions = advance();
            if (actions.motors) publish_motors();

            telemetry_->publish(last_sample_, get_clock()->now(), state_, ref, control);
            // ex trong dòng in là CHUẨN vector sai số vị trí (m), không riêng trục x.
            // f=a/b và Mz=a/b là yêu cầu P01/kết quả phân bổ sau chuyển đầu ra.
            // motor_stream=ON chỉ nói có gửi bản tin, chưa khẳng định PX4 đã arm.
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                "t=%.2f stage=%d blend=%.2f ex=%.3f Psi=%.4f f=%.2f/%.2f Mz=%.3f/%.3f saturated=%d motor_stream=%s",
                t, static_cast<int>(session_.stage()), motor_blend_,
                control.ex.norm(), control.Psi, control.f, allocation_.achieved_f,
                control.M.z(), allocation_.achieved_M.z(), allocation_.saturated,
                actions.motors ? "ON" : "OFF");
            if (log_)
            {
                log_ << t;
                for (int i = 0; i < 3; ++i) log_ << ',' << state_.x[i];
                for (int i = 0; i < 3; ++i) log_ << ',' << ref.x[i];
                log_ << ',' << control.ex.norm() << ',' << control.Psi << ',' << control.f << ',' << allocation_.achieved_f;
                for (int i = 0; i < 3; ++i) log_ << ',' << control.M[i];
                for (int i = 0; i < 3; ++i) log_ << ',' << allocation_.achieved_M[i];
                for (double u : allocation_.commands) log_ << ',' << u;
                log_ << ',' << allocation_.saturated << ',' << static_cast<int>(session_.stage())
                     << ',' << motor_blend_ << '\n';
            }
        }
        catch (const std::exception &e)
        {
            motor_valid_ = false;
            RCLCPP_ERROR(get_logger(), "Control error: %s", e.what());
            advance();
        }
    }
    // Bộ nhận/phát/timer phải tồn tại cùng node; SharedPtr giữ chúng làm thành viên.
    rclcpp::Subscription<Odom>::SharedPtr odom_sub_;
    rclcpp::Subscription<Status>::SharedPtr status_sub_;
    rclcpp::Subscription<Ack>::SharedPtr ack_sub_;
    rclcpp::Publisher<Motors>::SharedPtr motor_pub_;
    rclcpp::Publisher<Mode>::SharedPtr mode_pub_;
    rclcpp::Publisher<Command>::SharedPtr command_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    // Dữ liệu tính toán và bộ nhớ phiên; dấu _ cuối tên là quy ước biến thành viên.
    uav_research::P01Parameters parameters_;
    uav_research::X500Actuators actuators_;
    uav_research::X500Allocation allocation_;
    uav_research::P01State state_;
    uav_research::P01Control control_;
    Session session_;
    Status status_{};
    Clock::time_point epoch_ = Clock::now(), odom_time_{}, status_time_{};
    bool enabled_ = false, have_status_ = false, have_odom_ = false, motor_valid_ = false;
    bool rate_ok_ = false, exclusive_ = true, anchor_set_ = false, rotate_heading_ = false;
    std::uint64_t last_sample_ = 0, active_sample_ = 0;
    std::uint8_t reset_counter_ = 0;
    double speed_ = 0.25, ramp_ = 4, duration_ = 30, settle_ = 3;
    double takeoff_height_ = 2.0, takeoff_duration_ = 6.0, motor_blend_ = 0.0;
    bool motor_gate_ = false;
    std::uint64_t takeoff_sample_ = 0, settled_sample_ = 0;
    std::string mode_;
    Eigen::Vector3d origin_ = Eigen::Vector3d::Zero(), heading_ = Eigen::Vector3d::UnitX();
    std::ofstream log_;
    std::unique_ptr<uav_research::P01Telemetry> telemetry_;
    rclcpp::Subscription<Land>::SharedPtr land_sub_;

    Land land_state_{};
    bool have_land_ = false;
    Clock::time_point land_time_{};
};

int main(int argc, char **argv)
{
    // Khởi tạo ROS -> tạo node (đọc cấu hình/tạo kết nối) -> spin xử lý callback.
    // Không có vòng mô phỏng vật lý ở main; Gazebo đang chạy trong tiến trình khác.
    rclcpp::init(argc, argv); // khoi tao ros2
    try { rclcpp::spin(std::make_shared<P01OffboardHover>()); }// tạo đối tượng node, construcstor bắt đầu chạy
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("p01_offboard_hover"), "%s", e.what());
        if (rclcpp::ok()) rclcpp::shutdown();
        return 1;
    }
    if (rclcpp::ok()) rclcpp::shutdown();
    return 0;
}
