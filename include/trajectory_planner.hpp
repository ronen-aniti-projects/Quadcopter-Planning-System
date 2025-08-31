/**
 * @file trajectory_planner.hpp
 * @author Ronen Aniti (raniti@umd.edu)
 * @brief Provides tools for determining time-based trajectories that pass through all waypoints of a given waypoint plan.
 * @version 0.1
 * @date 2025-08-30
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#pragma once

#include "utils.hpp"
#include <Eigen/Dense>
#include <vector> 


/**
 * @namespace trajectory_planner
 * @brief Provides tools for generating trajectories between the waypoints of a waypoint plan.
 * 
 */
namespace trajectory_planner{

    /**
     * @brief Implements a degree-seven trajectory segmentation algorithm to generate time-ordered trajectory points
     * from a desired maximum quacopter maximum speed and a provided waypoint plan. 
     * 
     */
    class TrajectoryPlan {

        public: 

            /**
             * @brief Constructs a TrajectoryPlan object. Once created, the object serves to store the computed trajector as well as results from
             * intermediate calculations (matrix assemblies, etc).
             * 
             * @param waypoints (std::vector<std::vector<double>>): Waypoint plan
             * @param max_speed (double): Maximum speed of quadcopter
             */
            TrajectoryPlan(const std::vector<std::vector<double>>& waypoints, const double max_speed);
            
            /**
             * @brief Destroy the Trajectory Plan object
             * 
             */
            ~TrajectoryPlan() = default;
            
        private:

            /**
             * @brief Heuristic maximum speed. Generated trajectories will typically not exceed this. 
             * 
             */
            const double max_speed_;

            /**
             * @brief Waypoint plan
             * 
             */
            Eigen::MatrixXd waypoints_;

            /**
             * @brief Number of waypoints of waypoint plan
             * 
             */
            const int num_waypoints_;

            /**
             * @brief Heuristic speed used in trajectory planning calculations, derived from the provided maximum quadcopter speed. 
             * 
             */
            const double heuristic_speed_; 

            /**
             * @brief Nunmber of trajectory segments
             * 
             */
            const int num_segments_;

            /**
             * @brief Number of coefficients to solve for
             * 
             */
            const int num_coeffs_;

            /**
             * @brief Number of waypoints that are not the start or goal
             * 
             */
            const int num_intermediate_waypoints_;

            /**
             * @brief Meters of distance of each waypoint segment
             * 
             */
            Eigen::VectorXd segment_distances_;

            /**
             * @brief Timestamp assigned to each waypoint
             * 
             */
            Eigen::VectorXd waypoint_start_times_;


            /**
             * @brief Elapsed time allocated to each trajectory segment
             * 
             */
            Eigen::VectorXd segment_times_;

            /**
             * @brief Matrix encoding boundary conditions
             * 
             */
            Eigen::MatrixXd boundary_matrix_;

            /**
             * @brief Matrix encoding prescribed waypoint positions 
             * 
             */
            Eigen::MatrixXd waypoint_matrix_;

            /**
             * @brief Matrix encoding continuity conditions at intermediate waypoints
             * 
             */
            Eigen::MatrixXd continuity_matrix_;

            /**
             * @brief Augmented matrix assembled from boundary, continuity, and waypoint matrices. 
             * 
             */
            Eigen::MatrixXd augmented_constraint_matrix_;

            /**
             * @brief The constraint values imposed on each axis of the system
             * 
             */
            Eigen::MatrixXd constraints_;

            /**
             * @brief Data structure holding the solved system coefficients
             * 
             */
            Eigen::MatrixXd solved_coeffs_; // each axis as col

            /**
             * @brief Evaluated trajectory points
             * 
             */
            std::vector<std::vector<double>> evaluated_trajectory_; 

            /**
             * @brief Solves the trajectory
             * 
             */
            void solve_trajectory();

            /**
             * @brief Assembles the continuity matrix. Enforces continuity in derivatives 1, 2, 3 at all intermediate waypoints.
             * 
             */
            void assemble_continuity_matrix();

            /**
             * @brief Assembles the waypoint matrix. Enforces that the left end position polynomial for each segment evaluates to the prescribed waypoint correspondance. 
             * 
             */
            void assemble_waypoint_matrix();

            /** 
             * Assembles the boundary condition matrix. Enforces derivatives 1, 2, and 3 at the first and last waypoint are 0 for all three axes. 
             */
            void assemble_boundary_matrix();

            /**
             * @brief Asembles the augmented constraint matrix
             * 
             */
            void assemble_augmented_constraint_matrix();

            /**
             * @brief Assembles the constraint vector along the provided axis
             * 
             * @param axis_number (int): The axis being solved
             */
            void assemble_constraint_vector(const int axis_number);

            /**
             * @brief Evaluates the trajectory, with points spaced `dt` seconds apart.
             * 
             * @param dt (double): Spacing between trajectory points 
             */
            void evaluate_trajectory(const double dt = planning_config::TRAJECTORY_TIME_POINT_SPACING);
            
            /**
             * @brief Prints the trajectory plan to the console.
             * 
             */
            void print_data();

            /**
             * @brief Computes the starting timestamp (seconds) for each waypoint, with the first waypoint starting at 0.0. 
             * 
             */
            void compute_waypoint_start_times();

            /**
             * @brief Computes the elapsed time for each segment.
             * 
             */
            void compute_segment_times();

            /**
             * @brief Computes the straight-line distance between consecutive waypoints. 
             * 
             */
            void compute_segment_distances();
            
            /**
             * @brief Evaluates the basis polynomial at the provided normalized time.  
             * 
             * @param normalized_time (double): The normalized segment time ([-1, 1])
             * @param derivative_order (int): Which derivative of the basis polynomial to use
             * @return (Eigen::VectorXd): Returns all 8 elements of the basis polynomial
             */
            [[nodiscard]] Eigen::VectorXd derivative_powers(const double normalized_time, const int derivative_order);

            /**
             * @brief Solves for all coefficients on the axis provided
             * 
             * @param axis_number (int): Axis (0, 1, or 2)
             */
            void solve_system_on_axis(const int axis_number);

    }; // class TrajectoryPlan
} // namespace trajectory_planner