#include "trajectory_planner.hpp"
#include <Eigen/Dense>
#include "utils.hpp"
#include <vector> 
#include <stdexcept>
#include <iostream> 
#include <iomanip> 

namespace trajectory_planner {
    TrajectoryPlan::TrajectoryPlan(const std::vector<std::vector<double>>& waypoints, const double max_speed)

    : max_speed_(max_speed), 
      num_waypoints_(waypoints.size()),
      heuristic_speed_(max_speed / planning_config::HEURISTIC_SPEED_CONSTANT),
      num_segments_(waypoints.size() - 1),
      num_coeffs_(num_segments_ * planning_config::COEFFS_PER_SEGMENT),
      num_intermediate_waypoints_(num_waypoints_ - 2)
      {

        if (num_waypoints_ < 2) {
            throw std::invalid_argument("TrajectoryPlan: need at least 2 waypoints");
        }
        if (planning_config::HEURISTIC_SPEED_CONSTANT == 0.0) {
            throw std::invalid_argument("HEURISTIC_SPEED_CONSTANT must be nonzero");
        }
        for (int i = 0; i < num_waypoints_; ++i) {
            if (waypoints[i].size() != 3) {
                throw std::invalid_argument("Each waypoint must have exactly 3 coordinates");
            }
        }

        // Cast waypoints to type Eigen::MatrixXd
        waypoints_.resize(num_waypoints_, 3); //num_waypoints x 3 matrix (coords arranged xi yi zi on rows)
        for (int i{0}; i < num_waypoints_; ++i){
            waypoints_(i, 0) = waypoints[i][0];
            waypoints_(i, 1) = waypoints[i][1];
            waypoints_(i, 2) = waypoints[i][2];
        }

        solved_coeffs_.setZero(num_coeffs_, planning_config::NUM_AXES);

        compute_segment_distances();
        compute_segment_times();
        compute_waypoint_start_times(); 

        solve_trajectory();
        evaluate_trajectory();

    } // TrajectoryPlan (ctor)

    void TrajectoryPlan::print_data() {
        
        std::cout << "\n====    Trajectory Planner Printout    ====\n";
        std::cout << "\n Max speed: " << max_speed_ << " m/s\n";
        std::cout << " Heuristic speed: " << heuristic_speed_ << " m/s\n";

        std::cout << " Waypoints:\n";
        for (int i = 0; i < num_waypoints_; ++i) {
            std::cout << "  (" 
                    << waypoints_(i,0) << ", "
                    << waypoints_(i,1) << ", "
                    << waypoints_(i,2) << ")\n";
        }

        std::cout << "\n Waypoint start times:\n";
        for (int i = 0; i < waypoint_start_times_.size(); ++i) {
            std::cout << "  Waypoint " << i 
                    << ": " << waypoint_start_times_(i) << " s\n";
        }

        std::cout << "\n Segment elapsed times:\n";
        for (int i = 0; i < segment_times_.size(); ++i) {
            std::cout << "  Segment " << i 
                    << ": " << segment_times_(i) << " s\n";
        }

        std::cout << "\n Boundary matrix:\n";
        for (int i = 0; i < boundary_matrix_.rows(); ++i) {
            for (int j = 0; j < boundary_matrix_.cols(); ++j) {
                if (j == 0) std::cout << "  ";
                std::cout << boundary_matrix_(i,j);
                if (j < boundary_matrix_.cols() - 1) std::cout << ", ";
            }
            std::cout << "\n";
        }

        std::cout << "\n Waypoint matrix:\n";
        for (int i = 0; i < waypoint_matrix_.rows(); ++i) {
            for (int j = 0; j < waypoint_matrix_.cols(); ++j) {
                if (j == 0) std::cout << "  ";
                std::cout << waypoint_matrix_(i,j);
                if (j < waypoint_matrix_.cols() - 1) std::cout << ", ";
            }
            std::cout << "\n";
        }
        std::cout << "\n Evaluated trajectory (" << evaluated_trajectory_.size()
                << " points, dt = 0.01 s):\n";

        std::cout.setf(std::ios::fixed);
        std::cout << std::setprecision(4);

        for (size_t i{0}; i < evaluated_trajectory_.size(); ++i){
           if (i % 10 == 0) // Print every 10th point for brevity 
            std::cout << "  (" << evaluated_trajectory_[i][0] << ", " << evaluated_trajectory_[i][1] << ", " << evaluated_trajectory_[i][2] << ")\n"; 
        }
    }
    
    void TrajectoryPlan::evaluate_trajectory(double dt) {
        evaluated_trajectory_.clear();
        
        const double total_time = waypoint_start_times_(num_waypoints_ - 1);

        for (double t = 0.0; t <= total_time + 1e-9; t += dt) {
            
            // Determines which segment this time point is assigned to
            int current_segment = 0;

            // Increment the current segment when both the current segment is not the final segment and 
            // the current time is greater than the time assigned to the right end waypoint of the segment.
            while (current_segment < num_segments_ - 1 && t > waypoint_start_times_(current_segment + 1)) {
                ++current_segment;
            }

            // Time is normalized from [-1, 1] across all segments. So convert the current time into a normalized time.
            const double segment_start_time{waypoint_start_times_(current_segment)};
            double tau = -1.0 + 2.0 * (t - segment_start_time) / segment_times_(current_segment);  

            // Form the basis vector [1, tau, tau^2, ...] that will be multiplied into the solved coefficients to determine
            // the 3D point at this time step. 
            Eigen::VectorXd basis_components(planning_config::COEFFS_PER_SEGMENT);
            basis_components(0) = 1.0;
            for (int k = 1; k < planning_config::COEFFS_PER_SEGMENT; ++k) {
                basis_components(k) = basis_components(k-1) * tau;
            }

            // Extract all the coeffs for this segment and evaluate the position value on each axis, storing each in a 3D vector.
            int coeff_offset = current_segment * planning_config::COEFFS_PER_SEGMENT;
            std::vector<double> point(3, 0.0);
            for (int axis = 0; axis < 3; ++axis) {
                Eigen::VectorXd coeffs = solved_coeffs_.col(axis).segment(
                                            coeff_offset, planning_config::COEFFS_PER_SEGMENT);
                point[axis] = coeffs.dot(basis_components);
            }

            // Store the resulting 3D point
            evaluated_trajectory_.push_back(point);
        }
    }


    void TrajectoryPlan::compute_segment_distances() {
        segment_distances_.resize(num_segments_);
        for (int i{0}; i < num_segments_; ++i){
            segment_distances_(i) = (waypoints_.row(i + 1) - waypoints_.row(i)).norm();
        }
    }

    void TrajectoryPlan::compute_segment_times(){
        segment_times_.resize(num_segments_);
        for (int i{0}; i < num_segments_; ++i){
            segment_times_(i) = segment_distances_(i) / heuristic_speed_;
        }
    }

    void TrajectoryPlan::compute_waypoint_start_times(){
        waypoint_start_times_.resize(num_waypoints_);
        waypoint_start_times_(0) = 0.0; // start at 0.0 seconds

        double cumulative_time{0.0};
        for (int i{0}; i < num_segments_; ++i){
            cumulative_time += segment_times_(i);
            waypoint_start_times_(i+1) = cumulative_time;
        } 
    }
    
    Eigen::VectorXd TrajectoryPlan::derivative_powers(const double normalized_time, const int derivative_order){
        // Computes the terms of a polynomial in terms of normalized time. Derivative order basically encodes 
        // whether its a polynomial that encodes position (0), velocity (1), etc. Returns each terms as an 
        // element in a column vector.
        Eigen::VectorXd result{Eigen::VectorXd::Zero(planning_config::COEFFS_PER_SEGMENT)}; // Column vector
        for (int i{derivative_order}; i < planning_config::COEFFS_PER_SEGMENT; ++i){
            double coeff{1.0};
            for (int j{i - derivative_order + 1}; j <= i; ++j){
                coeff *= j;
            }
            result(i) = coeff * std::pow(normalized_time, i - derivative_order);
        }
        return result;
    }

    void TrajectoryPlan::solve_trajectory(){
        for (int i{0}; i < 3; ++i){
            solve_system_on_axis(i);     
        }

    }

    void TrajectoryPlan::assemble_boundary_matrix(){

        // Reize the boundary matrix about to be assembled
        boundary_matrix_.setZero(planning_config::NUM_BOUNDARY_CONSTRAINTS, num_coeffs_);
        // Pre-compute chain rule factors that result of normalizing time from [-1, 1] across each segment
        const double chain_rule_factor_segment_0{planning_config::CHAIN_RULE_COMPONENT / segment_times_(0)};
        const double chain_rule_factor_segment_final{planning_config::CHAIN_RULE_COMPONENT / segment_times_(num_segments_ - 1)};

        // --- Apply constraints at the left boundary --- 
        // Enforce velocity=0 at the start of the first segment
        boundary_matrix_.block(0,0,1,planning_config::COEFFS_PER_SEGMENT) = derivative_powers(-1.0, 1).transpose() * chain_rule_factor_segment_0;

        // Enforce accel=0 at the start of the first segment
        boundary_matrix_.block(1,0,1,planning_config::COEFFS_PER_SEGMENT) = derivative_powers(-1.0, 2).transpose() * std::pow(chain_rule_factor_segment_0, 2);

        // Enforce jerk=0 at the start of the first segment
        boundary_matrix_.block(2,0,1,planning_config::COEFFS_PER_SEGMENT) = derivative_powers(-1.0, 3).transpose() * std::pow(chain_rule_factor_segment_0, 3);


        // --- Apply constraints at the right boundary ---
        const int start_column_final_segment{(num_segments_-1) * planning_config::COEFFS_PER_SEGMENT};

        // Enforce velocity=0 at the end of the final segment
        boundary_matrix_.block(3,start_column_final_segment,1,planning_config::COEFFS_PER_SEGMENT) = derivative_powers(1.0, 1).transpose() * chain_rule_factor_segment_final;

        // Enforce accel=0 at the end of the final segment
        boundary_matrix_.block(4,start_column_final_segment,1,planning_config::COEFFS_PER_SEGMENT) = derivative_powers(1.0, 2).transpose() * std::pow(chain_rule_factor_segment_final, 2);

        // Enforce jerk=0 at the end of the final segment
        boundary_matrix_.block(5,start_column_final_segment,1,planning_config::COEFFS_PER_SEGMENT) = derivative_powers(1.0, 3).transpose() * std::pow(chain_rule_factor_segment_final, 3);
    }

    
    void TrajectoryPlan::assemble_waypoint_matrix(){
        waypoint_matrix_.setZero(num_waypoints_, num_coeffs_);

        // Enforce that all segment left ends lie on prescribed waypoint values. This is the LHS matrix laying that constraint in place.
        for (int i{0}; i < num_segments_; ++i){
            const int start_column{planning_config::COEFFS_PER_SEGMENT * i};
            waypoint_matrix_.block(i, start_column, 1, planning_config::COEFFS_PER_SEGMENT) = derivative_powers(-1.0, 0).transpose();
        }

        // Enforce that the final segment right end lies on the final waypoint value
        const int final_segment_column_start{(num_segments_ - 1) * planning_config::COEFFS_PER_SEGMENT};
        waypoint_matrix_.block(num_segments_, final_segment_column_start, 1, planning_config::COEFFS_PER_SEGMENT) = derivative_powers(1.0, 0).transpose();
    }

    void TrajectoryPlan::assemble_continuity_matrix(){
        // Height of the matrix is number of derivatives forced to zero at intermediate waypoints * number of intermediate waypoints
        continuity_matrix_.setZero((planning_config::TRAJ_POLYNOMIAL_ORDER) * num_intermediate_waypoints_, num_coeffs_);
        int row_index{0};
        for (int i{0}; i < num_intermediate_waypoints_; ++i){
            double segment_time_left{segment_times_(i)};
            double segment_time_right{segment_times_(i+1)};
            // Enforce continuity over position plus the first six derivatives at all intermediate waypoints
            // Left segment gets evaluated at right most time and is positive by convension. 
            // Right segment gets evaluated at left most time and is negative. That way, left terms plus right terms will equal zero. 
            for (int order{0}; order <= planning_config::TRAJ_POLYNOMIAL_ORDER - 1; ++order){
                Eigen::VectorXd left_segment_terms{derivative_powers(1.0, order) * std::pow(planning_config::CHAIN_RULE_COMPONENT / segment_time_left, order)};
                Eigen::VectorXd right_segment_terms{-1.0 * derivative_powers(-1.0, order) * std::pow(planning_config::CHAIN_RULE_COMPONENT / segment_time_right, order)};
                
                continuity_matrix_.block(row_index, planning_config::COEFFS_PER_SEGMENT * i, 1, planning_config::COEFFS_PER_SEGMENT) = left_segment_terms.transpose();
                continuity_matrix_.block(row_index, planning_config::COEFFS_PER_SEGMENT * (i + 1), 1, planning_config::COEFFS_PER_SEGMENT) = right_segment_terms.transpose();
               
                row_index++;

            }
        }
    }

    void TrajectoryPlan::assemble_augmented_constraint_matrix(){
        int total_rows{static_cast<int>(boundary_matrix_.rows() + waypoint_matrix_.rows() + continuity_matrix_.rows())};
        augmented_constraint_matrix_.setZero(total_rows, boundary_matrix_.cols());
        int current_row = 0;
        augmented_constraint_matrix_.block(current_row, 0, boundary_matrix_.rows(), boundary_matrix_.cols()) = boundary_matrix_;
        current_row += boundary_matrix_.rows();
        augmented_constraint_matrix_.block(current_row, 0, waypoint_matrix_.rows(), waypoint_matrix_.cols()) = waypoint_matrix_;
        current_row += waypoint_matrix_.rows();
        augmented_constraint_matrix_.block(current_row, 0, continuity_matrix_.rows(), continuity_matrix_.cols()) = continuity_matrix_;
    }




    void TrajectoryPlan::assemble_constraint_vector(const int axis_number){
        constraints_.setZero(boundary_matrix_.rows() + waypoint_matrix_.rows() + continuity_matrix_.rows(), planning_config::NUM_AXES);
        
        // Set the waypoint constraints
        for (int i{0}; i < num_waypoints_; ++i){
            constraints_(boundary_matrix_.rows() + i, axis_number) = waypoints_(i, axis_number);    
        }

        // The continuity constraints remain zero
        // The boundary constraints remain zero    
    }

    void TrajectoryPlan::solve_system_on_axis(const int axis_number){
        assemble_boundary_matrix();
        assemble_waypoint_matrix();
        assemble_continuity_matrix();
        assemble_augmented_constraint_matrix();
        assemble_constraint_vector(axis_number);

        // Solve the system of equations with Eigen
        Eigen::VectorXd axis_constraints{constraints_.col(axis_number)};
        Eigen::VectorXd solved_coeffs{augmented_constraint_matrix_.fullPivLu().solve(axis_constraints)};
        solved_coeffs_.col(axis_number) = solved_coeffs;

        

    }






}