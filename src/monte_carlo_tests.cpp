#include "obstacle_processor.hpp"
#include "local_planning.hpp"
#include "global_planning.hpp"
#include <vector>
#include <random>
#include <chrono>
#include <iostream>
#include <limits>

namespace monte_carlo_trials{

    void display_points(const std::vector<std::vector<double>>& points) noexcept{

    }

    void evaluate_global_planner(const obstacle_processor::ObstacleProcessor& obstacles, const global_planning::FreeSpaceGraph& free_space){

        // Create Random Number Generator
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<double> dist(0.0, 1.0);


        // Extract Obstacle Bounds
        const double xlow{obstacles.get_x_lim().at(0)};
        const double xhigh{obstacles.get_x_lim().at(1)};
        const double ylow{obstacles.get_y_lim().at(0)};
        const double yhigh{obstacles.get_y_lim().at(1)};
        const double zlow{obstacles.get_z_lim().at(0)};
        const double zhigh{obstacles.get_z_lim().at(1)};
        

        int valid_count{0};

        std::vector<double> sample_start(3);
        std::vector<double> sample_goal(3);
        std::vector<double> times;
        times.reserve(demo::NUM_MONTE_CARLO_TRIALS_GLOBAL_PLANNING);

        while (valid_count < demo::NUM_MONTE_CARLO_TRIALS_GLOBAL_PLANNING){

            sample_start = {xlow + dist(gen) * (xhigh - xlow), ylow + dist(gen) * (yhigh - ylow), zlow + dist(gen) * (zhigh - zlow)};
            if (obstacles.is_collision(sample_start)){
                continue;
            }
            
            sample_goal = {xlow + dist(gen) * (xhigh - xlow), ylow + dist(gen) * (yhigh - ylow), zlow + dist(gen) * (zhigh - zlow)};
            if (obstacles.is_collision(sample_goal)){
                continue;
            }

            if (helpers::distance_between_points(sample_start, sample_goal) > 700 || 
                helpers::distance_between_points(sample_start, sample_goal) < 600){
                    continue;
            }

            // Time the planning
            const auto trial_start_time{std::chrono::steady_clock::now()};
            std::optional<std::vector<std::vector<double>>> maybe_path = free_space.search(sample_start, sample_goal);
            const auto trial_end_time{std::chrono::steady_clock::now()};
            const auto trial_elapsed_milliseconds{std::chrono::duration_cast<std::chrono::milliseconds>(trial_end_time - trial_start_time)};    

            if (!maybe_path.has_value()){
                continue;
            }

            // If the planning is successful, record the elapsed seconds.
            times.push_back(trial_elapsed_milliseconds.count());


            valid_count++;

            std::cout << "Successfully Completed Trial " << valid_count<< "/" << demo::NUM_MONTE_CARLO_TRIALS_GLOBAL_PLANNING << " (" << trial_elapsed_milliseconds.count() << " ms)"<< '\n';
            

        }



        const double average_planning_milliseconds{std::accumulate(times.begin(), times.end(), 0.0) / static_cast<double>(times.size())};

        std::cout << "Results:" << '\n';
        std::cout << " Average Planning Time: " << average_planning_milliseconds<< " ms" << '\n';

        return;

    }


    void evaluate_local_planner(const obstacle_processor::ObstacleProcessor& obstacles, const global_planning::FreeSpaceGraph& free_space, const double goal_bias){

        // Create Random Number Generator
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<double> dist(0.0, 1.0);


        // Extract Obstacle Bounds
        const double xlow{obstacles.get_x_lim().at(0)};
        const double xhigh{obstacles.get_x_lim().at(1)};
        const double ylow{obstacles.get_y_lim().at(0)};
        const double yhigh{obstacles.get_y_lim().at(1)};
        const double zlow{obstacles.get_z_lim().at(0)};
        const double zhigh{obstacles.get_z_lim().at(1)};

        int valid_count{0};

        std::vector<double> sample_start(3);
        std::vector<double> sample_goal(3);
        std::vector<double> times;
        double prev_mean{};
        double post_mean{};
        double dynamic_percent_diff{std::numeric_limits<double>::max()};

        while (valid_count < demo::NUM_MONTE_CARLO_TRIALS_LOCAL_PLANNING || dynamic_percent_diff > demo::MONTE_CARLO_PERCENT_DIFF_TOLERANCE){


            sample_start = {xlow + dist(gen) * (xhigh - xlow), ylow + dist(gen) * (yhigh - ylow), zlow + dist(gen) * (zhigh - zlow)};
            if (obstacles.is_collision(sample_start)){
                continue;
            }
            
            sample_goal = {xlow + dist(gen) * (xhigh - xlow), ylow + dist(gen) * (yhigh - ylow), zlow + dist(gen) * (zhigh - zlow)};
            if (obstacles.is_collision(sample_goal)){
                continue;
            }

            if (helpers::distance_between_points(sample_start, sample_goal) > 50 || 
                helpers::distance_between_points(sample_start, sample_goal) < 45){
                    continue;
            }


            if (valid_count > 0){
                prev_mean = post_mean;
            }

            // Time the planning
            const auto trial_start_time{std::chrono::steady_clock::now()};
            std::optional<std::vector<std::vector<double>>> maybe_path = local_planning::rrt(obstacles, 
                sample_start, sample_goal, planning_config::RRT_STEP_SIZE, 
                planning_config::RRT_EDGE_VALIDATION_STEP_SIZE, planning_config::RRT_ITERATIONS, goal_bias);
                
            const auto trial_end_time{std::chrono::steady_clock::now()};
            const auto trial_elapsed_milliseconds{std::chrono::duration_cast<std::chrono::milliseconds>(trial_end_time - trial_start_time)};    

            if (!maybe_path.has_value()){
                continue;
            }

            // If the planning is successful, record the elapsed seconds.
            times.push_back(trial_elapsed_milliseconds.count());
            
            // Increment the count of valid trials
            valid_count++;

            std::cout << "Successfully Completed Trial " << valid_count<< "/" << demo::NUM_MONTE_CARLO_TRIALS_LOCAL_PLANNING << " (" << trial_elapsed_milliseconds.count() << " ms)"<< '\n';

            // Recompute the mean planning time. 
            post_mean = std::accumulate(times.begin(), times.end(), 0.0) / static_cast<double>(times.size());
            
            if (valid_count > 1){
                dynamic_percent_diff = abs(post_mean - prev_mean) / prev_mean * 100.0;
            }
            

        }

        std::cout << "Results:" << '\n';
        std::cout << " Average Planning Time: " << post_mean << " ms" << '\n';

        return;
    }

    

} // namespace monte_carlo_trials