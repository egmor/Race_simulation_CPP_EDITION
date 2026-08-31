#include "trajectory_selection.hpp"
#include "physics_engine.hpp"

// Вычисление эффективного радиуса для обычного (геометрического) поворота
double get_effective_radius(const Corner& seg) {
    if (std::abs(seg.degree) < 1e-6) return std::numeric_limits<double>::infinity();
    
    double central_radius = seg.get_central_radius();
    double half_width = seg.width / 2.0;
    double half_rad_angle = seg.get_degree_radians() / 2.0;

    return (((central_radius - half_width) / std::cos(half_rad_angle / 2.0)) - half_width);
}

// Предельная скорость на участке
double throughput_speed_limit(const Corner& seg) {
    if (std::abs(seg.degree) < 1e-6) return std::numeric_limits<double>::infinity();
    
    double effective_radius = get_effective_radius(seg);
    return std::sqrt(global_constants::tyre_grip * global_constants::g * effective_radius);
}

// Поздний апекс (Late Apex)
ApexResult late_apex(const Corner& seg) {
    if (std::abs(seg.degree) < 1e-6) {
        return {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), seg.distance, 0.0};
    }

    double center_degree_radians = seg.get_degree_radians();
    double entry_angle = global_constants::late_apex * center_degree_radians;
    double exit_angle = (1.0 - global_constants::late_apex) * center_degree_radians;

    double center_radius = seg.get_central_radius();
    double half_w = seg.width / 2.0;

    // В косинусы передаём половину угла соответственной фазы
    double entry_radius = (((center_radius + half_w) * (1.0 - std::cos(entry_angle / 2.0))) / std::cos(entry_angle / 2.0)) - half_w;
    double exit_radius = ((center_radius + half_w) / std::cos(exit_angle / 2.0)) - half_w;

    double min_r = std::min(entry_radius, exit_radius);
    double speed_apex = std::sqrt(global_constants::tyre_grip * global_constants::g * min_r);

    return {entry_radius, exit_radius, speed_apex, entry_radius * entry_angle, exit_radius * exit_angle};
}

// Ранний апекс (Early Apex)
ApexResult early_apex(const Corner& seg) {
    if (std::abs(seg.degree) < 1e-6) {
        return {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), seg.distance, 0.0};
    }

    double center_degree_radians = seg.get_degree_radians();
    double entry_angle = global_constants::early_apex * center_degree_radians;
    double exit_angle = (1.0 - global_constants::early_apex) * center_degree_radians;

    double center_radius = seg.get_central_radius();
    double half_w = seg.width / 2.0;

    // Инвертированная геометрия дуг относительно late_apex
    double entry_radius = ((center_radius + half_w) / std::cos(entry_angle / 2.0)) - half_w;
    double exit_radius = (((center_radius + half_w) * (1.0 - std::cos(exit_angle / 2.0))) / std::cos(exit_angle / 2.0)) - half_w;

    double min_r = std::min(entry_radius, exit_radius);
    double speed_apex = std::sqrt(global_constants::tyre_grip * global_constants::g * min_r);

    return {entry_radius, exit_radius, speed_apex, entry_radius * entry_angle, exit_radius * exit_angle};
}