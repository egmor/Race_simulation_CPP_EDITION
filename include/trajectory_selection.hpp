#ifndef TRAJECTORY_SELECTION_HPP
#define TRAJECTORY_SELECTION_HPP

#pragma once 

#include <iostream>
#include <cmath>
#include <limits>
#include <algorithm>
#include "physics_engine.hpp"


struct ApexResult {
    double radius;       // Радиус получившейся траектории
    double speed_apex;   // Корректная максимальная скорость в апексе (м/с)
    double apex_angle;   // Фактический угол точки апекса в радианах
};

double get_geometric_apex_radius(const Corner& seg);
double throughput_speed_limit(const Corner& seg);
ApexResult late_apex(const Corner& seg);
ApexResult early_apex(const Corner& seg);

#endif // TRAJECTORY_SELECTION_HPP