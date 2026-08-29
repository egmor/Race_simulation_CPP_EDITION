#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>

struct Corner {
    double distance{0.0};         // Длина участка в метрах
    double degree{0.0};           // Угол поворота (0 для прямых)
    double height_difference{0.0}; // Разница высот 

    Corner(double l, double a = 0.0, double h = 0.0) 
        : distance(l), degree(a), height_difference(h) {}
};

// Вспомогательная функция: расчёт максимальной скорости для поворота
double get_max_corner_speed(const Corner& seg, double tyre_grip) {
    if (std::abs(seg.degree) < 0.001) return 95.0; // Для прямой возвращаем топ-спид
    const double pi{3.141592653589793};
    const double g{9.81};
    double angle_rad = seg.degree * (pi / 180.0);
    double corner_radius = seg.distance / angle_rad; 
    double sin_slope = std::clamp(std::abs(seg.height_difference) / seg.distance, 0.0, 1.0);
    double longitudinal_slope = std::asin(sin_slope); 

    return std::sqrt(tyre_grip * g * corner_radius * std::cos(longitudinal_slope));
}

double calculate_segment_time(const Corner& seg, const Corner* next_seg, double& speed_enter) {
    double time_segment{0.0};
    const double g{9.81}; 
    const double tyre_grip{1.6}; 

    // --- 1. ПРЯМОЙ УЧАСТОК ---
    if (std::abs(seg.degree) < 0.001) {
        double accel {5.0};                   // Ускорение разгона (м/с^2)
        double max_brake_decel {tyre_grip * g};// Максимальное замедление при торможении (~15.7 м/с^2)
        double max_straight_speed {90.0};     // Топ-спид (~324 км/ч)

        // Узнаем целевую скорость для СЛЕДУЮЩЕГО поворота
        double target_speed  {next_seg ? get_max_corner_speed(*next_seg, tyre_grip) : max_straight_speed};

        // Расчёт тормозного пути: S_brake = (V_current^2 - V_target^2) / (2 * a_decel)
        double brake_distance {0.0};
        if (speed_enter > target_speed) {
            brake_distance = (speed_enter * speed_enter - target_speed * target_speed) / (2.0 * max_brake_decel);
        }

        // Если вся прямая уходит на торможение или мы уже близко к повороту
        if (seg.distance <= brake_distance) {
            // ФАЗА ПОЛНОГО ТОРМОЖЕНИЯ
            double speed_exit {std::sqrt(std::max(0.0, speed_enter * speed_enter - 2.0 * max_brake_decel * seg.distance))};
            speed_exit = std::max(speed_exit, target_speed); // Не тормозим ниже скорости поворота
            
            double avg_speed { (speed_enter + speed_exit) / 2.0 };
            time_segment = seg.distance / avg_speed;
            speed_enter = speed_exit;
        } 
        else {
            // ФАЗА: СНАЧАЛА РАЗГОН, ПОТОМ ТОРМОЖЕНИЕ
            double accel_distance {seg.distance - brake_distance};

            // 1. Разгоняемся на первой части прямой
            double speed_peak {std::sqrt(speed_enter * speed_enter + 2.0 * accel * accel_distance)};
            speed_peak = std::min(speed_peak, max_straight_speed);
            double time_accel {accel_distance / ((speed_enter + speed_peak) / 2.0)};

            // 2. Пересчитываем реальный тормозной путь от пиковой скорости
            double real_brake_dist {seg.distance - accel_distance};
            double speed_exit {target_speed};
            double time_brake {real_brake_dist / ((speed_peak + speed_exit) / 2.0)};

            time_segment = time_accel + time_brake;
            speed_enter = speed_exit; // К повороту подходим ровно на target_speed!
        }

        return time_segment;
    }

    // --- 2. ПОВОРОТ ---
    double max_possible_enter_speed = get_max_corner_speed(seg, tyre_grip);

    // Защита и проверка на вылет
    if (speed_enter > max_possible_enter_speed * 1.02) {
        std::cout << "[ВНИМАНИЕ] Вылет с трассы в повороте! Вход: " 
                  << speed_enter * 3.6 << " км/ч (Макс: " 
                  << max_possible_enter_speed * 3.6 << " км/ч)\n";
        speed_enter = max_possible_enter_speed;
    }

    double speed_exit {std::min(speed_enter, max_possible_enter_speed)};
    double avg_speed { (speed_enter + speed_exit) / 2.0 };
    
    time_segment = seg.distance / avg_speed;
    speed_enter = speed_exit;

    return time_segment;
}

int main() {
    std::vector<Corner> monza_track = {
        Corner(1120.0, 0.0, 1.5),    // Rettifilo
        Corner(35.0, 110.0, 0.0),    // T1
        Corner(35.0, 100.0, 0.0),    // T2
        Corner(480.0, 0.0, 0.0),     // Прямая к Curva Grande
        Corner(320.0, 35.0, -2.5),   // Curva Grande
        Corner(420.0, 0.0, 0.0),     // Прямая к Roggia
        Corner(40.0, 90.0, 0.0),     // T4
        Corner(40.0, 85.0, 0.0),     // T5
        Corner(260.0, 0.0, 0.0),     // Прямая Lesmo
        Corner(110.0, 65.0, -1.5),   // Lesmo 1
        Corner(150.0, 0.0, 0.0),     // Прямая
        Corner(120.0, 70.0, -3.0),   // Lesmo 2
        Corner(950.0, 0.0, -5.0),    // Serraglio
        Corner(30.0, 75.0, 0.0),     // Ascari T8
        Corner(25.0, 60.0, 0.0),     // Ascari T9
        Corner(35.0, 55.0, 0.0),     // Ascari T10
        Corner(600.0, 0.0, 2.0),     // Прямая перед Параболикой
        Corner(348.0, 42.0, 0.0)     // Parabolica
    };

    double speed = 80.0; // Стартовая скорость на главной прямой
    double total_time{0.0};

    for (size_t i = 0; i < monza_track.size(); ++i) {
        // Указываем следующий сегмент (с закольцовкой трассы)
        const Corner* next_seg = &monza_track[(i + 1) % monza_track.size()];
        
        total_time += calculate_segment_time(monza_track[i], next_seg, speed);
    }

    int total_ms = static_cast<int>(total_time * 1000.0);
    int minutes = total_ms / 60000;
    int seconds = (total_ms % 60000) / 1000;
    int milliseconds = total_ms % 1000;

    std::cout << "\nРезультат без вылетов: " 
              << minutes << ":" 
              << (seconds < 10 ? "0" : "") << seconds << "." 
              << (milliseconds < 100 ? (milliseconds < 10 ? "00" : "0") : "") << milliseconds 
              << " (" << total_time << " с)\n";

    return 0;
}