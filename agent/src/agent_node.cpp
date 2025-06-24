#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/string.hpp"
#include <cmath>

class AgentNode : public rclcpp::Node
{
public:
    AgentNode()
    : Node("agent_node")
    {
        declare_parameter("agent_id", "agent1");
        declare_parameter("x", 0.0);
        declare_parameter("y", 0.0);

        agent_id_ = get_parameter("agent_id").as_string();
        pose_.position.x = get_parameter("x").as_double();
        pose_.position.y = get_parameter("y").as_double();
        pose_.position.z = 0.0;

        sub_job_ = create_subscription<geometry_msgs::msg::Point>(
            "/job_info", 10,
            std::bind(&AgentNode::on_job_received, this, std::placeholders::_1));

        sub_assignment_ = create_subscription<std_msgs::msg::String>(
            "/task_assignment", 10,
            [this](std_msgs::msg::String::UniquePtr msg) {
                if (msg->data == agent_id_)
                    RCLCPP_INFO(get_logger(), "[%s] I have been assigned the task!", agent_id_.c_str());
            });

        pub_cost_ = create_publisher<std_msgs::msg::String>("/agent_costs", 10);

        RCLCPP_INFO(get_logger(), "Agent %s at (%.1f, %.1f) ready.", agent_id_.c_str(), pose_.position.x, pose_.position.y);
    }

private:
    std::string agent_id_;
    geometry_msgs::msg::Pose pose_;

    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_job_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_assignment_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_cost_;

    void on_job_received(const geometry_msgs::msg::Point::SharedPtr job)
    {
        double dist = std::hypot(
            pose_.position.x - job->x,
            pose_.position.y - job->y);

        std_msgs::msg::String msg;
        msg.data = agent_id_ + ":" + std::to_string(dist);
        pub_cost_->publish(msg);

        RCLCPP_INFO(get_logger(), "[%s] Sent cost %.2f", agent_id_.c_str(), dist);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AgentNode>());
    rclcpp::shutdown();
    return 0;
}
