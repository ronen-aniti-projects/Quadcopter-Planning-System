#pragma once 

namespace planning_config{
    constexpr double GLOBAL_RESOLUTION = 25.0;
} // namespace planning_config

namespace types{

    struct Point3D {
        double x{};
        double y{}; 
        double z{};
        Point3D() = delete;
        Point3D(double x_val, double y_val, double z_val)
            : x{x_val}, y{y_val}, z{z_val} {}
    }; // struct Point3D

    struct Point2D {
        double x{};
        double y{};
        Point2D() = delete;
        Point2D(double x_val, double y_val)
            : x{x_val}, y{y_val} {} 
    }; // struct Point2D

} // namespace types