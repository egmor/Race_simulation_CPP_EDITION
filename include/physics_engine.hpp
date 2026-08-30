#ifndef PHYSICS_ENGINE_HPP
#define PHYSICS_ENGINE_HPP

#include <vector>
#include <string>
#include <cmath>
#include <algorithm>

namespace global_constants {
    inline constexpr double g{9.81}; 
    inline constexpr double pi{3.141592653589793};
    inline constexpr double air_density{1.225}; 
    
    inline constexpr double downforce_coefficient{1.85}; 
    inline constexpr double drag_coefficient{0.95};
    
    inline constexpr double horsepower{530.0 * 745.7}; 
    inline constexpr double weight{1320.0};

    inline constexpr double tyre_grip{1.28}; 
    inline constexpr double effective_hp{ horsepower * 0.88 * 0.91 }; 
}

struct Corner {
    double distance{0.0};         
    double degree{0.0};           
    double height_difference{0.0}; 

    Corner(double l, double a = 0.0, double h = 0.0) 
        : distance(l), degree(a), height_difference(h) {}
};

double calculating_radius(const Corner& seg);
double dinamic_grip_max(double speed);
double max_speed_apex(double radius);
double get_current_accelerate(double speed);
double get_safe_decel_forces();
double get_decel_forces(double speed);
double get_target_speed_ahead(const std::vector<Corner>& track, size_t current_idx, double speed_enter);

#endif // PHYSICS_ENGINE_HPP