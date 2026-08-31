#ifndef TRAJECTORY_SELECTION_HPP
#define TRAJECTORY_SELECTION_HPP

#pragma once 

#include <iostream>
#include <cmath>
#include <limits>
#include <algorithm>
#include "physics_engine.hpp"

struct ApexResult {
    double entry_radius{0.0};
    double exit_radius{0.0};
    double speed_apex{0.0};
    double l_entry{0.0};
    double l_exit{0.0};
};

double get_effective_radius(const Corner& seg);
double throughput_speed_limit(const Corner& seg);
ApexResult late_apex(const Corner& seg);
ApexResult early_apex(const Corner& seg);

#endif // TRAJECTORY_SELECTION_HPP