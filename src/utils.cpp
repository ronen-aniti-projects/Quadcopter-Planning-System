#include "utils.hpp"
#include <vector> 
#include <cmath>

namespace helpers{

    [[nodiscard]] double distance_between_points(const std::vector<double>& point1, const std::vector<double>& point2) noexcept{
        const double dx{point1.at(0) - point2.at(0)};
        const double dy{point1.at(1) - point2.at(1)};
        const double dz{point1.at(2) - point2.at(2)};
        return sqrt(dx * dx + dy * dy + dz * dz);
    }

} // namespace helpers