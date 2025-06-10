#pragma once 

#include <array>
#include <vector>
#include <unordered_map>

using Point2D = std::array<float, 2>;
using Points2D = std::vector<Point2D>;
using Point3D = std::array<float, 3>;
using Points3D = std::vector<Point3D>;
using Corner = std::array<std::array<float, 3>, 8>;
using Corners = std::vector<Corner>;

namespace std {
    template<>
    struct hash<Point3D> {
        size_t operator()(const Point3D& p) const{
            const size_t PRIME1 = 1610612741; // Prime for X coordinate
            const size_t PRIME2 = 402653189; // Prime for Y coordinate
            const size_t PRIME3 = 201326611; // Prime for Z coordinate
            
            // Combine the hash for each coordinate with XOR
            return hash<float>{}(p[0] * PRIME1) ^ 
                   hash<float>{}(p[1] * PRIME2) ^
                   hash<float>{}(p[2] * PRIME3);

        }
    };
}


using AdjacencyList = std::unordered_map<Point3D, // Key
                                         std::vector<std::pair<Point3D, float>>, // Value
                                         std::hash<Point3D> // Hash
                                         >;