#include "rclcpp/rclcpp.hpp"
#include "dracon_msgs/msg/job_list.hpp"
#include "dracon_msgs/msg/job.hpp"
#include "dracon_msgs/msg/job_status.hpp"
#include <vector>
#include <string>
#include <algorithm>
#include <chrono>

using namespace std::chrono_literals;

class ControllerNode : public rclcpp::Node {
public:
    ControllerNode() : Node("controller_node") {
        job_pub_ = create_publisher<dracon_msgs::msg::JobList>("available_jobs", 10);
        status_sub_ = create_subscription<dracon_msgs::msg::JobStatus>(
            "job_status", 10,
            std::bind(&ControllerNode::on_job_status, this, std::placeholders::_1));
        
        // Initialize all 50 jobs
        initialize_all_jobs();
        
        // Release initial batch
        release_job_batch();
        
        RCLCPP_INFO(get_logger(), "Controller initialized with %zu total jobs. Will maintain %zu active jobs.", all_jobs_.size(), TARGET_ACTIVE_JOBS);
    }

private:
    rclcpp::Publisher<dracon_msgs::msg::JobList>::SharedPtr job_pub_;
    rclcpp::Subscription<dracon_msgs::msg::JobStatus>::SharedPtr status_sub_;
    
    std::vector<dracon_msgs::msg::Job> all_jobs_;      // All 50 jobs
    std::vector<dracon_msgs::msg::Job> active_jobs_;   // Currently active jobs
    std::vector<dracon_msgs::msg::Job> completed_jobs_; // Completed jobs
    size_t next_job_index_ = 0;
    const size_t TARGET_ACTIVE_JOBS = 5;  // Maintain 5 active jobs
    const size_t TOTAL_JOBS = 50;

    void initialize_all_jobs() {
        all_jobs_.clear();
        
        // Generate 50 jobs with different positions
        for (int i = 1; i <= TOTAL_JOBS; ++i) {
            dracon_msgs::msg::Job job;
            job.job_id = "job" + std::to_string(i);
            
            // Distribute jobs in a grid pattern for variety
            int grid_size = 10; // 10x10 grid
            int row = (i - 1) / grid_size;
            int col = (i - 1) % grid_size;
            
            job.x = col * 2.0 + 1.0;  // X positions: 1, 3, 5, 7, 9, 11, 13, 15, 17, 19
            job.y = row * 2.0 + 1.0;  // Y positions: 1, 3, 5, 7, 9, 11, 13, 15, 17, 19
            
            all_jobs_.push_back(job);
        }
        
        RCLCPP_INFO(get_logger(), "Initialized %zu jobs in a 10x10 grid pattern", all_jobs_.size());
    }

    void release_job_batch() {
        if (next_job_index_ >= all_jobs_.size()) {
            RCLCPP_INFO(get_logger(), "All jobs have been released. Waiting for completion...");
            return;
        }
        
        // Calculate how many jobs to add to reach target
        size_t jobs_needed = TARGET_ACTIVE_JOBS - active_jobs_.size();
        if (jobs_needed == 0) {
            return; // Already at target
        }
        
        // Add jobs up to target or until we run out
        size_t jobs_to_add = std::min(jobs_needed, all_jobs_.size() - next_job_index_);
        for (size_t i = 0; i < jobs_to_add; ++i) {
            active_jobs_.push_back(all_jobs_[next_job_index_ + i]);
        }
        
        next_job_index_ += jobs_to_add;
        
        // Publish current active jobs
        publish_jobs();
        
        RCLCPP_INFO(get_logger(), "Released %zu jobs to fill vacancies. Active jobs: %zu, Remaining to release: %zu", 
                   jobs_to_add, active_jobs_.size(), all_jobs_.size() - next_job_index_);
    }

    void publish_jobs() {
        dracon_msgs::msg::JobList job_list;
        job_list.jobs = active_jobs_;
        job_pub_->publish(job_list);
        RCLCPP_INFO(get_logger(), "Published %zu active jobs", active_jobs_.size());
    }

    void on_job_status(const dracon_msgs::msg::JobStatus::SharedPtr msg) {
        if (msg->status == "complete") {
            // Find and remove the completed job from active jobs
            auto it = std::remove_if(active_jobs_.begin(), active_jobs_.end(), 
                [&](const dracon_msgs::msg::Job& job) {
                    return job.job_id == msg->job_id;
                });
            
            if (it != active_jobs_.end()) {
                // Move completed job to completed_jobs_ list
                completed_jobs_.push_back(*it);
                active_jobs_.erase(it, active_jobs_.end());
                
                RCLCPP_INFO(get_logger(), "Job %s completed by agent %s. Active jobs: %zu, Completed: %zu", 
                           msg->job_id.c_str(), msg->agent_id.c_str(), active_jobs_.size(), completed_jobs_.size());
                
                // Check if we need to release more jobs to maintain target
                if (active_jobs_.size() < TARGET_ACTIVE_JOBS && next_job_index_ < all_jobs_.size()) {
                    RCLCPP_INFO(get_logger(), "Vacancy detected! Releasing more jobs to maintain %zu active jobs.", TARGET_ACTIVE_JOBS);
                    release_job_batch();
                } else if (!active_jobs_.empty()) {
                    // Republish remaining active jobs
                    publish_jobs();
                } else if (next_job_index_ >= all_jobs_.size()) {
                    RCLCPP_INFO(get_logger(), "All jobs completed! Total completed: %zu", completed_jobs_.size());
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