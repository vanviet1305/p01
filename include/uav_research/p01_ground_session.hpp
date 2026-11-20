#pragma once

#include <algorithm>

namespace uav_research
{

// Node ROS cung cấp các thông tin thực tế cho máy trạng thái.
struct GroundInputs
{
    bool enabled = false;
    bool feedback_ok = false;   // Odometry/status/phép tính hợp lệ.
    bool land_ok = false;       // Bản tin mặt đất còn mới.
    bool ready_ground = false; // Đứng yên, nằm ngang, chưa arm.
    bool landed = false;
    bool armed = false;
    bool offboard = false;
    bool failsafe = false;
    bool exclusive = true;     // Không có nguồn Offboard khác.

    // Node xác nhận đã hết đoạn lên cao và ổn định tại mục tiêu.
    bool takeoff_complete = false;
};

// Máy trạng thái quyết định hành động; không tự gọi ROS.
struct GroundActions
{
    bool heartbeat = false;
    bool motors = false;
    bool request_offboard = false;
    bool request_arm = false;
    bool request_hold = false;
    bool start_takeoff = false;

    // 0: đầu ra rotor chạy thấp; 1: dùng đầy đủ đầu ra P01.
    // Node phải áp dụng hệ số này trước khi gửi lệnh motor.
    double control_blend = 0.0;
};

class P01GroundSession
{
public:
    enum class Stage
    {
        Waiting = 0,
        Priming = 1,
        Requested = 2,
        Arming = 5,
        Spooling = 6,
        Climbing = 7,
        Active = 3,  // Giữ tương thích bộ đọc CSV cũ: stage=3 là Active.
        Stopped = 4
    };

    Stage stage() const { return stage_; }
    const char *reason() const { return reason_; }

    GroundActions step(double now, const GroundInputs &in)
    {
        GroundActions out;

        if (!in.enabled || stage_ == Stage::Stopped)
        {
            return out;
        }

        if (!in.exclusive)
        {
            return stop(in, "Another Offboard/motor publisher detected");
        }

        const bool preparing =
            stage_ == Stage::Waiting || stage_ == Stage::Priming;

        if (!in.feedback_ok || !in.land_ok || in.failsafe)
        {
            if (preparing)
            {
                enter(Stage::Waiting, now, "Waiting for valid feedback");
                return out;
            }

            return stop(in, "Feedback lost or PX4 failsafe");
        }

        if (preparing && (in.armed || in.offboard))
        {
            return stop(in, "Start requires disarmed vehicle outside Offboard");
        }

        const bool mode_confirmed =
            stage_ == Stage::Arming ||
            stage_ == Stage::Spooling ||
            stage_ == Stage::Climbing ||
            stage_ == Stage::Active;

        // Không tự giành quyền lại nếu PX4/người vận hành thoát Offboard.
        if (mode_confirmed && !in.offboard)
        {
            return stop(in, "PX4 exited Offboard");
        }

        const bool arm_confirmed =
            stage_ == Stage::Spooling ||
            stage_ == Stage::Climbing ||
            stage_ == Stage::Active;

        if (arm_confirmed && !in.armed)
        {
            return stop(in, "Vehicle disarmed");
        }

        switch (stage_)
        {
        case Stage::Waiting:
            if (!in.ready_ground)
            {
                return out;
            }
            enter(Stage::Priming, now, "Prestreaming on ground");
            break;

        case Stage::Priming:
            if (!in.ready_ground)
            {
                enter(Stage::Waiting, now, "Ground conditions changed");
                return out;
            }

            if (now - since_ >= 1.2)
            {
                enter(Stage::Requested, now, "Waiting for Offboard");
                out.request_offboard = true;
            }
            break;

        case Stage::Requested:
            if (!in.landed || in.armed)
            {
                return stop(in, "Unexpected state before arm request");
            }

            if (in.offboard)
            {
                enter(Stage::Arming, now, "Waiting for armed status");
                out.request_arm = true;
            }
            else if (now - since_ > 3.0)
            {
                return stop(in, "Offboard request timed out");
            }
            break;

        case Stage::Arming:
            if (in.armed)
            {
                enter(Stage::Spooling, now, "Starting rotors");
            }
            else if (!in.landed || now - since_ > 3.0)
            {
                return stop(in, "Arm request failed or timed out");
            }
            break;

        case Stage::Spooling:
            if (now - since_ >= 2.0)
            {
                enter(Stage::Climbing, now, "P01 takeoff started");
                out.start_takeoff = true;
            }
            break;

        case Stage::Climbing:
            if (in.takeoff_complete)
            {
                enter(Stage::Active, now, "Takeoff complete; holding");
            }
            else if (now - since_ > 20.0)
            {
                return stop(in, "Takeoff did not complete in time");
            }
            break;

        case Stage::Active:
            // Tiếp tục giữ vị trí; sau này sẽ nối thêm quỹ đạo.
            break;

        case Stage::Stopped:
            return out;
        }

        // Trong phần chuẩn bị, motor nhận mức thấp, không nhận ngay lực P01.
        out.heartbeat = true;
        out.motors = true;

        if (stage_ == Stage::Spooling)
        {
            const double s = std::clamp((now - since_) / 2.0, 0.0, 1.0);
            out.control_blend = s * s * (3.0 - 2.0 * s);
        }
        else if (stage_ == Stage::Climbing || stage_ == Stage::Active)
        {
            out.control_blend = 1.0;
        }

        return out;
    }

private:
    void enter(Stage next, double now, const char *reason)
    {
        stage_ = next;
        since_ = now;
        reason_ = reason;
    }

    GroundActions stop(const GroundInputs &in, const char *reason)
    {
        stage_ = Stage::Stopped;
        reason_ = reason;

        GroundActions out;
        out.request_hold =
            in.exclusive && in.offboard && in.armed && !in.failsafe;
        return out;
    }

    Stage stage_ = Stage::Waiting;
    double since_ = 0.0;
    const char *reason_ = "Waiting for ground conditions";
};

} // namespace uav_research
