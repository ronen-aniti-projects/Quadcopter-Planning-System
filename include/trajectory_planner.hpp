#pragma once

#include "utils.hpp"
#include <Eigen/Dense>
#include <vector> 

namespace trajectory_planner{



    class TrajectoryPlan {

        public: 

            TrajectoryPlan(const std::vector<std::vector<double>>& waypoints, const double max_speed);
            ~TrajectoryPlan() = default;
            void print_data();


        private:
            const double max_speed_;
            Eigen::MatrixXd waypoints_;
            const int num_waypoints_;

            const double heuristic_speed_; 
            const int num_segments_;
            const int num_coeffs_;
            const int num_intermediate_waypoints_;
            Eigen::VectorXd segment_distances_;
            Eigen::VectorXd waypoint_start_times_;
            Eigen::VectorXd elapsed_times_;
            Eigen::VectorXd segment_times_;

            Eigen::MatrixXd boundary_matrix_;
            Eigen::MatrixXd waypoint_matrix_;
            Eigen::MatrixXd continuity_matrix_;
            Eigen::MatrixXd augmented_constraint_matrix_;
            Eigen::MatrixXd constraints_;

            Eigen::MatrixXd solved_coeffs_; // each axis as col

            std::vector<std::vector<double>> evaluated_trajectory_; 

            void solve_trajectory();

            void assemble_continuity_matrix();
            void assemble_waypoint_matrix();
            void assemble_boundary_matrix();
            void assemble_augmented_constraint_matrix();
            void assemble_constraint_vector(const int axis_number);
            void evaluate_trajectory(const double dt = planning_config::TRAJECTORY_TIME_POINT_SPACING);

            [[nodiscard]] Eigen::VectorXd get_coeffs_for_segment(); 
            [[nodiscard]] double normalize_time(const double global_time);
            [[nodiscard]] Eigen::VectorXd get_coeffs_for_segment(const size_t segment_index, const Eigen::VectorXd& all_coeffs);
            [[nodiscard]] size_t find_segment_index(const double global_time);
            void compute_waypoint_start_times();
            void compute_segment_times();
            void compute_segment_distances();
            [[nodiscard]] Eigen::VectorXd derivative_powers(const double normalized_time, const int derivative_order);
            void solve_system_on_axis(const int axis_number);



            


    }; // class TrajectoryPlan
} // namespace trajectory_planner