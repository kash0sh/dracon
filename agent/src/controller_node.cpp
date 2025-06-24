#include "rclcpp/rclcpp.hpp"
#include "dracon_msgs/msg/job_list.hpp"
#include "dracon_msgs/msg/job.hpp"
#include "dracon_msgs/msg/job_status.hpp"
#include <vector>
#include <string>
#include <algorithm>

class ControllerNode : public rclcpp::Node {
public:
    ControllerNode() : Node("controller_node") {
        job_pub_ = create_publisher<dracon_msgs::msg::JobList>("available_jobs", 10);
        status_sub_ = create_subscription<dracon_msgs::msg::JobStatus>(
            "job_status", 10,
            std::bind(&ControllerNode::on_job_status, this, std::placeholders::_1));
        // Initialize jobs
        jobs_.clear();
        {
            dracon_msgs::msg::Job job;
            job.job_id = "job1"; job.x = 1.0; job.y = 2.0; jobs_.push_back(job);
            job.job_id = "job2"; job.x = 3.0; job.y = 4.0; jobs_.push_back(job);
            job.job_id = "job3"; job.x = 5.0; job.y = 6.0; jobs_.push_back(job);
            job.job_id = "job4"; job.x = 7.0; job.y = 8.0; jobs_.push_back(job);
        }
        publish_jobs();
    }

private:
    rclcpp::Publisher<dracon_msgs::msg::JobList>::SharedPtr job_pub_;
    rclcpp::Subscription<dracon_msgs::msg::JobStatus>::SharedPtr status_sub_;
    std::vector<dracon_msgs::msg::Job> jobs_;

    void publish_jobs() {
        dracon_msgs::msg::JobList job_list;
        job_list.jobs = jobs_;
        job_pub_->publish(job_list);
        RCLCPP_INFO(get_logger(), "Published available jobs (%zu jobs left)", jobs_.size());
    }

    void on_job_status(const dracon_msgs::msg::JobStatus::SharedPtr msg) {
        if (msg->status == "complete") {
            auto it = std::remove_if(jobs_.begin(), jobs_.end(), [&](const dracon_msgs::msg::Job& job) {
                return job.job_id == msg->job_id;
            });
            if (it != jobs_.end()) {
                jobs_.erase(it, jobs_.end());
                RCLCPP_INFO(get_logger(), "Job %s completed by agent %s. Republishing jobs.", msg->job_id.c_str(), msg->agent_id.c_str());
                if (!jobs_.empty()) {
                    publish_jobs();
                } else {
                    RCLCPP_INFO(get_logger(), "All jobs completed!");
                }
            }
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControllerNode>());
    rclcpp::shutdown();
    return 0;
}