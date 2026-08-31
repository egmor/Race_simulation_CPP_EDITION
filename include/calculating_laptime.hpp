#ifndef CALCULATING_LAPTIME_HPP
#define CALCULATING_LAPTIME_HPP

#pragma once 

#include "physics_engine.hpp"
#include "trajectory_selection.hpp"

double cornering_time(const std::vector<Corner>& track, size_t current_index, double speed_entry);
double get_next_entry_speed(const std::vector<Corner>& track, size_t current_index);
double straight_time(const std::vector<Corner>& track, size_t current_index, double speed_entry, double& speed_exit_out);
double laptime(const std::vector<Corner>& track);

#endif // CALCULATING_LAPTIME_HPP