#include "physics_engine.hpp"
#include "trajectory_selection.hpp"

double cornering_time(const std::vector<Corner>& track, size_t current_index, double speed_entry) {
    const Corner& current {track[current_index]};
    
    // Безопасное получение следующего и предыдущего сегментов
    bool has_next {(current_index + 1 < track.size())};
    const Corner* next {has_next ? &track[current_index + 1] : nullptr};

    // 1. Диспетчер выбора апекса
    ApexResult apex;
    
    if (next && (*next).is_straight && next->distance > 100.0) {
        // Выход на длинную прямую -> Поздний апекс
        apex = late_apex(current);
    } else if (next && !(*next).is_straight) {
        // Если следующая секция — поворот в противоположную сторону -> Ранний апекс
        // (При условии наличия знака направления или проверки углов)
        apex = early_apex(current);
    } else {
        // Обычный геометрический апекс для максимального радиуса
        double radius_eff {get_effective_radius(current)};
        double speed_max {throughput_speed_limit(current)};
        apex = {radius_eff, radius_eff, speed_max, current.distance / 2.0, current.distance / 2.0};
    }

    // 2. Фаза 1: Вход и торможение (Entry / Braking) до скорости V_apex
    // Считаем время замедления с v_entry до apex.speed_apex на дуге apex.l_entry
    double speed_apex {std::min(speed_entry, apex.speed_apex)}; // Коррекция, если вошли медленнее апекса
    double speed_avg_entry {(speed_entry + speed_apex) / 2.0};
    double time_entry {(speed_avg_entry > 0.0) ? (apex.l_entry / speed_avg_entry) : 0.0};

    // 3. Фаза 2: Прохождение точки Апекса
    // Короткий микроинтервал постоянной минимальной скорости
    double time_apex {0.05}; // Микро-задержка на перекладывание/качение

    // 4. Фаза 3: Выход и разгон (Exit / Acceleration)
    // Разгон на дуге apex.l_exit с ограничением по эллипсу сцепления
    double acceleration_lat {pow(speed_apex, 2.0) / apex.exit_radius};
    double acceleration_lat_max {global_constants::tyre_grip * global_constants::g};
    
    // Доступное продольное ускорение: a_long = sqrt(a_max^2 - a_lat^2)
    double friction_margin {std::max(0.0, 1.0 - std::pow(acceleration_lat / acceleration_lat_max, 2))};
    double acceleration_long_available = global_constants::max_accel * std::sqrt(friction_margin);

    // Финальная скорость на выходе из поворота v_exit: v_exit^2 = v_apex^2 + 2 * a_long * L_exit
    double speed_exit = std::sqrt(pow(speed_entry, 2.0) + 2.0 * acceleration_long_available * apex.l_exit);
    double speed_avg_exit = (speed_entry + speed_exit) / 2.0;
    double time_exit = (speed_avg_exit > 0.0) ? (apex.l_exit / speed_avg_exit) : 0.0;

    // Суммарное время прохождения поворота
    return time_entry + time_apex + time_exit;
}

double get_next_entry_speed(const std::vector<Corner>& track, size_t current_index) {
    if (current_index + 1 >= track.size()) {
        return global_constants::max_speed; // Если это финишная прямая
    }
    
    const Corner& next_corner = track[current_index + 1];
    if (next_corner.is_straight) {
        return global_constants::max_speed; // Следующий участок тоже прямая
    }

    // Для поворота целевой лимит определяется эффективным радиусом
    return throughput_speed_limit(next_corner);
}

double straight_time(const std::vector<Corner>& track, size_t current_index, double speed_entry, double& speed_exit_out) {
    const Corner& current {track[current_index]};
    double lenght {current.distance};

    // 1. Определение целевой скорости перед следующим поворотом
    double speed_target_next {get_next_entry_speed(track, current_index)};

    // 2. Расчёт дистанции торможения L_brake
    // v_target^2 = v_current^2 - 2 * a_brake * L_brake  =>  L_brake = (v_current^2 - v_target^2) / (2 * a_brake)
    // Берём максимальное замедление с учётом сцепления шин
    double accel_brake_max {global_constants::tyre_grip * global_constants::g}; 
    
    // Дистанция торможения, если бы мы тормозили прямо со скорости входа v_entry
    double lenght_brake {0.0};
    if (speed_entry > speed_target_next) {
        lenght_brake = (pow(speed_entry, 2.0) - pow(speed_target_next, 2.0)) / (2.0 * accel_brake_max);
    }

    // 3. Если длина прямой МЕНЬШЕ дистанции торможения (короткий переклад между поворотами)
    if (lenght_brake >= lenght) {
        // Принудительно тормозим всю прямую
        double speed_exit {std::sqrt(std::max(0.0, pow(speed_entry, 2.0) - 2.0 * accel_brake_max * lenght))};
        speed_exit_out = speed_exit;
        
        double speed_avg {(speed_entry + speed_exit) / 2.0};
        return (speed_avg > 0.0) ? (lenght / speed_avg) : 0.0;
    }

    // 4. Разделение прямой на фазу разгона (L_accel) и фазу торможения (L_brake)
    double lenght_accel_available {lenght - lenght_brake};
    
    // Фаза разгона с учётом сопротивления воздуха (Drag Force):
    // a_accel(v) = a_max * (1 - (v / v_max)^2)
    // Для простейшей симуляции используем среднее продольное ускорение
    double accel_accel_eff {global_constants::max_accel * (1.0 - std::pow(speed_entry / global_constants::max_speed, 2))};
    accel_accel_eff = std::max(0.1, accel_accel_eff); // Защита от нулевого ускорения

    // Рассчитываем пиковую скорость в конце фазы разгона v_peak
    double speed_peak_sq {pow(speed_entry, 2.0) + 2.0 * accel_accel_eff * lenght_accel_available};
    double speed_peak {std::min(global_constants::max_speed, std::sqrt(speed_peak_sq))};

    // Уточняем дистанцию торможения с пиковой скорости v_peak до v_target_next
    if (speed_peak > speed_target_next) {
        lenght_brake = (pow(speed_peak, 2.0) - pow(speed_target_next, 2.0)) / (2.0 * accel_brake_max);
    } else {
        lenght_brake = 0.0;
    }

    double lenght_accel {std::max(0.0, lenght - lenght_brake)};

    // 5. Расчёт времени для обеих фаз
    // Время разгона
    double speed_avg_accel {(speed_entry + speed_peak) / 2.0};
    double time_accel {(speed_avg_accel > 0.0) ? (lenght_accel / speed_avg_accel) : 0.0};

    // Время торможения
    double time_brake {0.0};
    if (lenght_brake > 0.0) {
        double speed_avg_brake {(speed_peak + speed_target_next) / 2.0};
        time_brake = (speed_avg_brake > 0.0) ? (lenght_brake / speed_avg_brake) : 0.0;
        speed_exit_out = speed_target_next; // На выходе получаем идеальную скорость входа в поворот
    } else {
        speed_exit_out = speed_peak;
    }

    return time_accel + time_brake;
}

double laptime(const std::vector<Corner>& track) {
    if (track.empty()) return 0.0;

    double total_time {0.0};

    // Начальная скорость перед первым участком (например, скорость с финишной прямой или 0.0 с места)
    // Берём скорость входа как лимит первого сегмента или 0 при старте с места
    double current_speed {throughput_speed_limit(track[0])}; 
    if (std::isinf(current_speed)) {
        current_speed = 100.0 / 3.6; // Если круг начинается с прямой (например, ~100 км/ч)
    }

    for (size_t current_segment = 0; current_segment < track.size(); ++current_segment) {
        const Corner& seg {track[current_segment]};
        double segment_time {0.0};
        double speed_after_segment {current_speed};

        if (std::abs(seg.degree) < 1e-6) {
            // Участок — ПРЯМАЯ
            segment_time = straight_time(track, current_segment, current_speed, speed_after_segment);
        } else {
            // Участок — ПОВОРОТ
            segment_time = cornering_time(track, current_segment, current_speed);
            
            // Расчёт выездной скорости из поворота для следующего отрезка
            ApexResult apex;
            if (current_segment + 1 < track.size() && std::abs(track[current_segment + 1].degree) < 1e-6 && track[current_segment + 1].distance > 100.0) {
                apex = late_apex(seg);
            } else {
                double r_eff {get_effective_radius(seg)};
                apex = {r_eff, r_eff, throughput_speed_limit(seg), seg.distance / 2.0, seg.distance / 2.0};
            }

            // Рассчитываем разгон на выходе из апекса
            double v_apex {std::min(current_speed, apex.speed_apex)};
            double a_lat {(v_apex * v_apex) / apex.exit_radius};
            double a_lat_max {global_constants::tyre_grip * global_constants::g};
            double friction_margin {std::max(0.0, 1.0 - std::pow(a_lat / a_lat_max, 2))};
            double a_long_available {global_constants::max_accel * std::sqrt(friction_margin)};

            speed_after_segment = std::sqrt(v_apex * v_apex + 2.0 * a_long_available * apex.l_exit);
        }

        total_time += segment_time;
        current_speed = speed_after_segment; // Передаём скорость выхода в следующий сегмент
    }

    return total_time;
}