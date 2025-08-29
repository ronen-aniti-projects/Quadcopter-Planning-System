#include "utils.hpp"
#include "global_planning.hpp"
#include <vector> 
#include <cmath>
#include <iostream>

namespace helpers{

    double distance_between_points(const std::vector<double>& point1, const std::vector<double>& point2) noexcept{
        const double dx{point1.at(0) - point2.at(0)};
        const double dy{point1.at(1) - point2.at(1)};
        const double dz{point1.at(2) - point2.at(2)};
        return sqrt(dx * dx + dy * dy + dz * dz);
    }



    double ask_double(const std::string& message) {
        std::cout << message << " " << std::flush;
        double val; 
        std::cin >> val;   
        return val;
    }


} // namespace helpers