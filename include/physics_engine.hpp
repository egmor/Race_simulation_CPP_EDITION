#ifndef PHYSICS_ENGINE_HPP
#define PHYSICS_ENGINE_HPP

#pragma once

#include <vector>
#include <string>
#include <cmath>
#include <algorithm>


namespace global_constants {
    constexpr double g{9.81}; 
    constexpr double pi{3.141592653589793};
    constexpr double air_density{1.225}; 
    
    constexpr double downforce_coefficient{1.85}; 
    constexpr double drag_coefficient{0.95};
    
    constexpr double horsepower{530.0 * 745.7}; 
    constexpr double weight{1320.0};

    constexpr double tyre_grip{1.28}; 
    constexpr double effective_hp{ horsepower * 0.88 * 0.91 }; 

    constexpr double apex{0.5};
    constexpr double late_apex{0.675};
    constexpr double early_apex{0.325};

    constexpr double max_accel{12.0};
    constexpr double max_speed{297.5};
}

struct Corner {
    double degree{0.0};   // Угол поворота в градусах 
    double distance{0.0}; // Длина участка в метрах
    double width{0.0};    // Ширина трассы в метрах
    double height_difference{0.0};  // Уклон трассы в метрах
    bool is_straight{false};

    double get_central_radius() const {
        if (std::abs(degree) < 1e-6) {
            return std::numeric_limits<double>::infinity(); // Прямая линия
        }
        return (180.0 * distance) / (global_constants::pi * degree);
    }

    double get_degree_radians() const {
        return degree * (global_constants::pi / 180.0);
    }

    Corner(double deg, double dist, double wid, double h_diff, bool is_str)
        : degree(deg), distance(dist), width(wid), height_difference(h_diff), is_straight(is_str) {}
};

double dinamic_grip_max(double speed);
double max_speed_apex(double radius);
double get_current_accelerate(double speed);
double get_safe_decel_forces();
double get_decel_forces(double speed);

#endif // PHYSICS_ENGINE_HPP