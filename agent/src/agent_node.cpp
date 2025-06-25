#include "rclcpp/rclcpp.hpp"
#include "dracon_msgs/msg/job_list.hpp"
#include "dracon_msgs/msg/job_cost_table.hpp"
#include "dracon_msgs/msg/job_cost.hpp"
#include "dracon_msgs/msg/job_assignment_table.hpp"
#include "dracon_msgs/msg/job_assignment.hpp"
#include "dracon_msgs/msg/job_status.hpp"
#include <chrono>
#include <cmath>
#include <unordered_map>
#include <vector>
#include <algorithm>

using namespace std::chrono_literals;

class AgentNode : public rclcpp::Node
{
public:
    AgentNode()
    : Node("agent_node")
    {
        declare_parameter("agent_id", "agent1");
        declare_parameter("x", 0.0);
        declare_parameter("y", 0.0);
        declare_parameter("isLeader", false);

        agent_id_ = get_parameter("agent_id").as_string();
        x_ = get_parameter("x").as_double();
        y_ = get_parameter("y").as_double();
        is_leader_ = get_parameter("isLeader").as_bool();

        sub_jobs_ = create_subscription<dracon_msgs::msg::JobList>(
            "available_jobs", 10,
            std::bind(&AgentNode::on_jobs_received, this, std::placeholders::_1));

        sub_costs_ = create_subscription<dracon_msgs::msg::JobCostTable>(
            "job_costs", 10,
            std::bind(&AgentNode::on_costs_received, this, std::placeholders::_1));

        sub_assignments_ = create_subscription<dracon_msgs::msg::JobAssignmentTable>(
            "job_assignments", 10,
            std::bind(&AgentNode::on_assignments_received, this, std::placeholders::_1));

        pub_costs_ = create_publisher<dracon_msgs::msg::JobCostTable>("job_costs", 10);
        pub_assignments_ = create_publisher<dracon_msgs::msg::JobAssignmentTable>("job_assignments", 10);
        pub_status_ = create_publisher<dracon_msgs::msg::JobStatus>("job_status", 10);

        RCLCPP_INFO(get_logger(), "Agent %s at (%.1f, %.1f) ready. Leader: %s", agent_id_.c_str(), x_, y_, is_leader_ ? "true" : "false");
    }

private:
    std::string agent_id_;
    double x_, y_;
    bool is_leader_;

    std::vector<dracon_msgs::msg::Job> current_jobs_;
    std::unordered_map<std::string, std::unordered_map<std::string, double>> cost_table_; // agent_id -> job_id -> cost
    rclcpp::Time last_jobs_time_;
    bool waiting_for_costs_ = false;

    rclcpp::Subscription<dracon_msgs::msg::JobList>::SharedPtr sub_jobs_;
    rclcpp::Subscription<dracon_msgs::msg::JobCostTable>::SharedPtr sub_costs_;
    rclcpp::Subscription<dracon_msgs::msg::JobAssignmentTable>::SharedPtr sub_assignments_;
    rclcpp::Publisher<dracon_msgs::msg::JobCostTable>::SharedPtr pub_costs_;
    rclcpp::Publisher<dracon_msgs::msg::JobAssignmentTable>::SharedPtr pub_assignments_;
    rclcpp::Publisher<dracon_msgs::msg::JobStatus>::SharedPtr pub_status_;
    rclcpp::TimerBase::SharedPtr cost_timer_;
    rclcpp::TimerBase::SharedPtr job_timer_;

    // Step 1: Receive jobs and start cost gossip
    void on_jobs_received(const dracon_msgs::msg::JobList::SharedPtr msg) {
        if (msg->jobs.empty()) return;
        current_jobs_ = msg->jobs;
        last_jobs_time_ = now();
        waiting_for_costs_ = true;
        cost_table_.clear();
        RCLCPP_INFO(get_logger(), "[%s] Received job list:", agent_id_.c_str());
        for (const auto& job : current_jobs_) {
            RCLCPP_INFO(get_logger(), "  Job %s at (%.1f, %.1f)", job.job_id.c_str(), job.x, job.y);
        }
        // Step 2: Compute and gossip costs
        dracon_msgs::msg::JobCostTable cost_table_msg;
        for (const auto& job : current_jobs_) {
            dracon_msgs::msg::JobCost cost;
            cost.agent_id = agent_id_;
            cost.job_id = job.job_id;
            cost.cost = std::hypot(x_ - job.x, y_ - job.y);
            cost_table_msg.costs.push_back(cost);
            // Store own cost
            cost_table_[agent_id_][job.job_id] = cost.cost;
            RCLCPP_INFO(get_logger(), "[%s] Computed cost for job %s: %.2f", agent_id_.c_str(), job.job_id.c_str(), cost.cost);
        }
        pub_costs_->publish(cost_table_msg);
        RCLCPP_INFO(get_logger(), "[%s] Published cost table with %zu costs.", agent_id_.c_str(), cost_table_msg.costs.size());
        // Start timer to aggregate costs for 2 seconds
        cost_timer_ = create_wall_timer(2s, std::bind(&AgentNode::on_cost_aggregation_timeout, this));
        RCLCPP_INFO(get_logger(), "[%s] Started cost aggregation timer.", agent_id_.c_str());
    }

    // Step 3: Aggregate costs from all agents
    void on_costs_received(const dracon_msgs::msg::JobCostTable::SharedPtr msg) {
        RCLCPP_INFO(get_logger(), "[%s] Received cost table from %s with %zu costs", 
                   agent_id_.c_str(), msg->costs[0].agent_id.c_str(), msg->costs.size());
        for (const auto& cost : msg->costs) {
            cost_table_[cost.agent_id][cost.job_id] = cost.cost;
            RCLCPP_INFO(get_logger(), "[%s] Received cost from %s for job %s: %.2f", agent_id_.c_str(), cost.agent_id.c_str(), cost.job_id.c_str(), cost.cost);
        }
    }

    // Step 4: Leader assigns jobs after 2 seconds
    void on_cost_aggregation_timeout() {
        if (!is_leader_ || !waiting_for_costs_) {
            RCLCPP_INFO(get_logger(), "[%s] Cost aggregation timeout - is_leader_: %s, waiting_for_costs_: %s", 
                       agent_id_.c_str(), is_leader_ ? "true" : "false", waiting_for_costs_ ? "true" : "false");
            return;
        }
        waiting_for_costs_ = false;
        RCLCPP_INFO(get_logger(), "[%s] Aggregating costs and assigning jobs as leader...", agent_id_.c_str());
        
        // Debug: Print all costs received
        RCLCPP_INFO(get_logger(), "[%s] Cost table contains costs from %zu agents:", agent_id_.c_str(), cost_table_.size());
        for (const auto& [agent, jobs] : cost_table_) {
            RCLCPP_INFO(get_logger(), "[%s] Agent %s has costs for %zu jobs", agent_id_.c_str(), agent.c_str(), jobs.size());
        }
        
        // Find all agent_ids and job_ids
        std::vector<std::string> agent_ids;
        std::vector<std::string> job_ids;
        for (const auto& [agent, jobs] : cost_table_) agent_ids.push_back(agent);
        for (const auto& job : current_jobs_) job_ids.push_back(job.job_id);
        
        RCLCPP_INFO(get_logger(), "[%s] Found %zu agents and %zu jobs", agent_id_.c_str(), agent_ids.size(), job_ids.size());
        
        // Build cost matrix
        std::vector<std::vector<double>> cost_matrix(agent_ids.size(), std::vector<double>(job_ids.size(), 1e9));
        for (size_t i = 0; i < agent_ids.size(); ++i) {
            for (size_t j = 0; j < job_ids.size(); ++j) {
                if (cost_table_[agent_ids[i]].count(job_ids[j]))
                    cost_matrix[i][j] = cost_table_[agent_ids[i]][job_ids[j]];
            }
        }
        // Greedy assignment (Hungarian can be added later)
        std::vector<bool> job_assigned(job_ids.size(), false);
        dracon_msgs::msg::JobAssignmentTable assignments_msg;
        for (size_t i = 0; i < agent_ids.size(); ++i) {
            double min_cost = 1e9;
            int min_j = -1;
            for (size_t j = 0; j < job_ids.size(); ++j) {
                if (!job_assigned[j] && cost_matrix[i][j] < min_cost) {
                    min_cost = cost_matrix[i][j];
                    min_j = j;
                }
            }
            dracon_msgs::msg::JobAssignment assignment;
            assignment.agent_id = agent_ids[i];
            if (min_j != -1) {
                assignment.job_id = job_ids[min_j];
                assignment.job_x = current_jobs_[min_j].x;
                assignment.job_y = current_jobs_[min_j].y;
                job_assigned[min_j] = true;
                RCLCPP_INFO(get_logger(), "[%s] Assigning job %s to agent %s (cost: %.2f)", agent_id_.c_str(), assignment.job_id.c_str(), assignment.agent_id.c_str(), min_cost);
            } else {
                assignment.job_id = "";
                assignment.job_x = 0.0;
                assignment.job_y = 0.0;
                RCLCPP_INFO(get_logger(), "[%s] No job assigned to agent %s", agent_id_.c_str(), assignment.agent_id.c_str());
            }
            assignments_msg.assignments.push_back(assignment);
        }
        pub_assignments_->publish(assignments_msg);
        RCLCPP_INFO(get_logger(), "[%s] Published job assignments.", agent_id_.c_str());
    }

    // Step 5: Receive assignments and execute job if assigned
    void on_assignments_received(const dracon_msgs::msg::JobAssignmentTable::SharedPtr msg) {
        for (const auto& assignment : msg->assignments) {
            if (assignment.agent_id == agent_id_) {
                if (assignment.job_id.empty()) {
                    RCLCPP_INFO(get_logger(), "[%s] No job assigned, waiting...", agent_id_.c_str());
                    return;
                }
                RCLCPP_INFO(get_logger(), "[%s] Assigned job %s at (%.1f, %.1f)", agent_id_.c_str(), assignment.job_id.c_str(), assignment.job_x, assignment.job_y);
                // Step 6: Start job timer (simulate 5s task)
                job_timer_ = create_wall_timer(5s, [this, assignment]() {
                    dracon_msgs::msg::JobStatus status;
                    status.agent_id = agent_id_;
                    status.job_id = assignment.job_id;
                    status.status = "complete";
                    pub_status_->publish(status);
                    RCLCPP_INFO(get_logger(), "[%s] Completed job %s", agent_id_.c_str(), assignment.job_id.c_str());
                    job_timer_->cancel();
                });
                RCLCPP_INFO(get_logger(), "[%s] Started job %s, will complete in 5 seconds...", agent_id_.c_str(), assignment.job_id.c_str());
                return;
            }
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AgentNode>());
    rclcpp::shutdown();
    return 0;
}
