#include "physics_engine.hpp"

double calculating_radius(const Corner& seg) {
    if (std::abs(seg.degree) < 0.001) return 0.0; 
    double angle_rad{seg.degree * (global_constants::pi / 180.0)};
    return seg.distance / angle_rad;
}

double dinamic_grip_max(double speed) {
    double downforce{0.5 * global_constants::air_density * global_constants::downforce_coefficient * speed * speed}; 
    double normal_force{(global_constants::weight * global_constants::g) + downforce}; 
    return global_constants::tyre_grip * normal_force;
}

double max_speed_apex(double radius) {
    if (radius <= 0.0) return 285.0 / 3.6; // Прямая: максимум коробки в м/с

    double numerator{global_constants::tyre_grip * global_constants::g * radius};
    double denominator{1.0 - ((global_constants::tyre_grip * global_constants::air_density * global_constants::downforce_coefficient * radius) / (2.0 * global_constants::weight))};

    if (denominator <= 0.0) return 95.0; 

    return std::sqrt(numerator / denominator); // Возвращает М/С
}

double get_current_accelerate(double speed) {
    double speed_safe = std::max(speed, 2.0);
    double engine_force{global_constants::effective_hp / speed_safe}; 
    double grip_max_force{dinamic_grip_max(speed)}; 
    double drag_force{0.5 * global_constants::air_density * global_constants::drag_coefficient * speed * speed}; 

    double net_force{std::min(engine_force, grip_max_force) - drag_force}; 
    return net_force / global_constants::weight;
}

double get_safe_decel_forces() {
    return global_constants::tyre_grip * global_constants::g;
}

double get_decel_forces(double speed) {
    double f_brake_grip{dinamic_grip_max(speed)};
    double f_drag{0.5 * global_constants::air_density * global_constants::drag_coefficient * speed * speed};
    return (f_brake_grip + f_drag) / global_constants::weight;
}

double get_target_speed_ahead(const std::vector<Corner>& track, size_t current_idx, double speed_enter) {
    double min_speed{285.0 / 3.6}; // В м/с

    // Ищем ближайший следующий поворот
    for (size_t offset = 1; offset <= 10; ++offset) {
        size_t idx = (current_idx + offset) % track.size();
        const Corner& seg{track[idx]};

        if (std::abs(seg.degree) > 0.001) {
            double radius = calculating_radius(seg);
            double seg_max_speed = max_speed_apex(radius);
            
            if (seg.degree > 75.0) {
                seg_max_speed *= 0.88;
            }

            if (seg_max_speed < min_speed) {
                min_speed = seg_max_speed;
            }
            break; // Нашли ближайший поворот — выходим
        }
    }

    return min_speed; // Возвращает М/С
}