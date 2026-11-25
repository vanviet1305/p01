#pragma once
// NHIỄU NGOÀI TRONG GAZEBO (chỉ dùng trong mô phỏng): lực/mômen tác dụng lên thân UAV qua plugin ApplyLinkWrench.
// Plugin đã được PX4 nạp sẵn cho mọi world (PX4-Autopilot/Tools/simulation/gz/server.config) → KHÔNG gắn thêm,
// gắn thêm sẽ có 2 bản và lực bị nhân đôi (đã đo 25/11/2026: gửi 5 N, P01 tăng lực đẩy 10,0 N).
// Topic: /world/<world>/wrench/persistent (gz.msgs.EntityWrench) tác dụng mỗi bước mô phỏng tới khi gửi
// /world/<world>/wrench/clear (gz.msgs.Entity). Lệnh persistent CỘNG DỒN → chỉ gửi đúng một lần.
// Lực, mômen nhận ở hệ NED (như node); đổi sang ENU của Gazebo trước khi gửi.
// Mômen biểu diễn trong hệ THẾ GIỚI (không quay theo thân). Bộ điều khiển không biết gì về nhiễu này.
#include <Eigen/Dense>
#include <string>

#include <gz/msgs/entity.pb.h>
#include <gz/msgs/entity_wrench.pb.h>
#include <gz/transport/Node.hh>

namespace uav_research
{
class GzWrench
{
public:
    void start(const std::string &world, const std::string &model)
    {
        model_ = model;
        pub_ = node_.Advertise<gz::msgs::EntityWrench>("/world/" + world + "/wrench/persistent");
        clear_ = node_.Advertise<gz::msgs::Entity>("/world/" + world + "/wrench/clear");
    }

    // Bật lực F (N) và mômen T (N.m), cả hai ở hệ NED thế giới. Trả về false nếu chưa có bên nhận.
    bool apply(const Eigen::Vector3d &f_ned, const Eigen::Vector3d &t_ned)
    {
        gz::msgs::EntityWrench m;
        m.mutable_entity()->set_name(model_);
        m.mutable_entity()->set_type(gz::msgs::Entity::MODEL);
        // NED → ENU: (x, y, z)_ENU = (y, x, −z)_NED
        auto *f = m.mutable_wrench()->mutable_force();
        f->set_x(f_ned.y()); f->set_y(f_ned.x()); f->set_z(-f_ned.z());
        auto *t = m.mutable_wrench()->mutable_torque();
        t->set_x(t_ned.y()); t->set_y(t_ned.x()); t->set_z(-t_ned.z());
        return pub_.HasConnections() && pub_.Publish(m);
    }

    void clear()
    {
        gz::msgs::Entity e;
        e.set_name(model_);
        e.set_type(gz::msgs::Entity::MODEL);
        clear_.Publish(e);
    }

private:
    gz::transport::Node node_;
    gz::transport::Node::Publisher pub_, clear_;
    std::string model_;
};
} // namespace uav_research
