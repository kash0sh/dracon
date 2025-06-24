#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/string.hpp"
#include <unordered_map>
#include <limits>
#include <chrono>

class ControllerNode : public rclcpp::Node
{
public:
    ControllerNode() : Node("controller_node")
    {
        pub_job_ = create_publisher<geometry_msgs::msg::Point>("/job_info", 10);
        sub_costs_ = create_subscription<std_msgs::msg::String>(
            "/agent_costs", 10, std::bind(&ControllerNode::on_cost_received, this, std::placeholders::_1));
        pub_assignment_ = create_publisher<std_msgs::msg::String>("/task_assignment", 10);

        // Simulate sending a job after 2 seconds
        timer_ = create_wall_timer(std::chrono::seconds(2), std::bind(&ControllerNode::send_job, this));
    }

private:
    geometry_msgs::msg::Point job_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_job_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_assignment_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_costs_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::unordered_map<std::string, double> agent_costs_;
    size_t expected_responses_ = 2; // number of agents to wait for
    bool job_sent_ = false;

    void send_job()
    {
        job_.x = 5.0;
        job_.y = 5.0;
        job_.z = 0.0;

        pub_job_->publish(job_);
        RCLCPP_INFO(get_logger(), "[Controller] Job published at (%.1f, %.1f)", job_.x, job_.y);
        agent_costs_.clear();
        job_sent_ = true;
    }

    void on_cost_received(const std_msgs::msg::String::SharedPtr msg)
    {
        if (!job_sent_) return;

        auto delim_pos = msg->data.find(":");
        if (delim_pos == std::string::npos) return;

        std::string agent_id = msg->data.substr(0, delim_pos);
        double cost = std::stod(msg->data.substr(delim_pos + 1));
        agent_costs_[agent_id] = cost;

        RCLCPP_INFO(get_logger(), "[Controller] Received cost %.2f from %s", cost, agent_id.c_str());

        if (agent_costs_.size() >= expected_responses_)
        {
            assign_task();
        }
    }

    void assign_task()
    {
        std::string winner;
        double min_cost = std::numeric_limits<double>::max();

        for (const auto &[id, cost] : agent_costs_)
        {
            if (cost <= min_cost)
            {
                min_cost = cost;
                winner = id;
            }
        }

        if (!winner.empty())
        {
            std_msgs::msg::String assignment;
            assignment.data = winner;
            pub_assignment_->publish(assignment);
            RCLCPP_INFO(get_logger(), "[Controller] Task assigned to %s (cost %.2f)", winner.c_str(), min_cost);
        }

        job_sent_ = false;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControllerNode>());
    rclcpp::shutdown();
    return 0;
}
