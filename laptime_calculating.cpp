#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>
#include <string>

namespace gc { //gc сокращенние от "global constants"
    const double g{9.81}; // Ускорение свободного падения (м/с^2)
    const double pi{3.141592653589793};
    const double air_density {1.225}; // Плотность воздуха (кг/м^3)
}

struct Car {
    double horsepower{530.0}; // Мощность двигателя (л.с.)
    double weight{1320.0};     // Масса автомобиля (кг)
    double Cd_A {0.95};        // Коэффициент аэродинамического сопротивления на площадь автомобиля (C_d * A)
    
    double tyre_grip{1.28}; // Коэффициент сцепления шин

    double effective_hp { horsepower * 0.88 * 0.91 }; // Эффективная мощность двигателя с учётом КПД трансмиссии и потерь

    double get_hp_to_watts() const {
        return effective_hp * 745.7; // Перевод л.с. в Вт
    }
};

struct Corner {
    double distance{0.0};         // Длина участка в метрах
    double degree{0.0};           // Угол поворота (0 для прямых)
    double height_difference{0.0}; // Разница высот 

    Corner(double l, double a = 0.0, double h = 0.0) 
        : distance(l), degree(a), height_difference(h) {}
};

void print_laptime(double laptime, const std::string& lap_number) {
    int total_ms = static_cast<int>(std::round(laptime * 1000.0));
    int minutes {total_ms / 60000};
    int seconds { (total_ms % 60000) / 1000 };
    int milliseconds { total_ms % 1000 };

    std::cout << "\n" << lap_number  << " time: "
              << minutes << ":" 
              << (seconds < 10 ? "0" : "") << seconds << "." 
              << (milliseconds < 100 ? (milliseconds < 10 ? "00" : "0") : "") << milliseconds;
}

// Вспомогательная функция: расчёт максимальной скорости для поворота
double get_max_corner_speed(const Corner& seg, double tyre_grip) {
    if (std::abs(seg.degree) < 0.001) return 85.0; // 

    double angle_rad {seg.degree * (gc::pi / 180.0)};
    double corner_radius {seg.distance / angle_rad}; 
    double sin_slope {std::clamp(std::abs(seg.height_difference) / seg.distance, 0.0, 1.0)};
    double longitudinal_slope {std::asin(sin_slope)}; 

    double speed {std::sqrt(tyre_grip * gc::g * corner_radius * std::cos(longitudinal_slope))};

    if (seg.degree > 75.0) {
        speed *= 0.82; 
    }

    return speed;
}

double get_current_accelerate (const Car& car, double speed) {// Функция для расчёта ускорения в данный момент времени на основе текущей скорости и характеристик автомобиля
    double accelerate{0.0};
    double engine_force {car.get_hp_to_watts() / speed}; // Сила от двигателя (Н)
    double grip_max_force {car.tyre_grip * car.weight * gc::g}; // Максимальная сила сцепления (Н)
    double drag_force { 0.5 * gc::air_density * car.Cd_A * speed * speed}; // Сила сопротивления воздуха

    double net_force {std::min(engine_force, grip_max_force) - drag_force}; // Чистая сила (Н)
    accelerate = net_force / car.weight; // Ускорение (м/с^2)

    return accelerate;
}

// Замедление ТОЛЬКО по сцеплению шин (без аэродинамики) для безопасного запаса торможения
double get_safe_decel_forces(const Car& car) {
    return car.tyre_grip * gc::g;
}

// Замедление с учётом аэродинамики для физики
double get_decel_forces(const Car& car, double current_v) {
    double f_brake_grip { car.tyre_grip * car.weight * gc::g};
    double f_drag {0.5 * gc::air_density * car.Cd_A * current_v * current_v};
    return (f_brake_grip + f_drag) / car.weight;
}

// Функция поиска минимальной разрешённой скорости впереди (сквозь связки поворотов)
double get_target_speed_ahead(const std::vector<Corner>& track, size_t current_idx, const Car& car) {
    double min_speed {100.0};
    double accumulated_dist {0.0};

    for (size_t offset = 1; offset <= 3; ++offset) {
        size_t idx = (current_idx + offset) % track.size();
        const Corner& seg { track[idx] };

        double seg_max_speed { get_max_corner_speed(seg, car.tyre_grip)};
        
        // Если нашли поворот с меньшей лимитированной скоростью
        if (seg_max_speed < min_speed) {
            min_speed = seg_max_speed;
        }

        accumulated_dist += seg.distance;
        // Если дистанция заглядывания превысила 300м, останавливаем поиск
        if (accumulated_dist > 300.0) break;
    }

    return min_speed;
}

double calculate_segment_time(const std::vector<Corner>& track, size_t current_idx, const Car& car, double& speed_enter) {
    const Corner& seg = track[current_idx];
    double time_segment{0.0};   

    // --- 1. ПРЯМОЙ УЧАСТОК ---
    if (std::abs(seg.degree) < 0.001) {
        double target_speed = get_target_speed_ahead(track, current_idx, car);
        
        double distance_left {seg.distance};
        double total_time {0.0};
        const double dt {0.005};

        while (distance_left > 0.0) {
            // Используем консервативное замедление (только шины) для расчёта тормозного пути
            double safe_decel {get_safe_decel_forces(car)};

            double brake_distance {0.0};
            if (speed_enter > target_speed) {
                brake_distance = (speed_enter * speed_enter - target_speed * target_speed) / (2.0 * safe_decel);
            }

            double accel {0.0};

            if (distance_left <= brake_distance) {
                // Реальное торможение с учётом аэродинамики
                accel = -get_decel_forces(car, speed_enter);
            } else {
                accel = get_current_accelerate(car, speed_enter);
            }

            speed_enter += accel * dt;
            const double max_gearbox_speed = 285.0 / 3.6; // 285 км/ч в м/с
            speed_enter = std::min(speed_enter, max_gearbox_speed);
            
            if (distance_left <= brake_distance && speed_enter < target_speed) {
                speed_enter = target_speed;
            }

            double ds {speed_enter * dt};
            distance_left -= ds;
            total_time += dt;
        }

        return total_time;
    }

    // --- 2. ПОВОРОТ ---
    double max_possible_enter_speed {get_max_corner_speed(seg, car.tyre_grip)};

    // Если скорость всё ещё выше допустимой — тормозим прямо в повороте (аварийный дотормоз)
    if (speed_enter > max_possible_enter_speed * 1.01) {
        std::cout << "[ВНИМАНИЕ] Вылет с трассы в повороте! Вход: " 
                  << speed_enter * 3.6 << " км/ч (Макс: " 
                  << max_possible_enter_speed * 3.6 << " км/ч)\n";
        speed_enter = max_possible_enter_speed;
    }

    // В связках поворотов (например T1 -> T2) принудительно гасим скорость до лимита следующего поворота
    size_t next_idx { (current_idx + 1) % track.size() };
    double next_max_speed = get_max_corner_speed(track[next_idx], car.tyre_grip);

    double speed_exit {std::min({speed_enter, max_possible_enter_speed, next_max_speed})};
    double avg_speed  { (speed_enter + speed_exit) / 2.0 };
    
    time_segment = seg.distance / avg_speed;
    speed_enter = speed_exit;

    return time_segment;
}

int main() {
    Car car;
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

    double speed {80.0};
    double total_time{0.0};

    for (size_t lap = 0; lap < 50; ++lap) {
        double lap_time {0.0};
        for (size_t i = 0; i < monza_track.size(); ++i) {
            lap_time += calculate_segment_time(monza_track, i, car, speed);
        }
        print_laptime(lap_time,"Lap " + std::to_string(lap + 1) + (lap + 1 < 10 ? " " : "") ); // Вывод времени круга с выравниванием для однозначных чисел
        total_time += lap_time;
        car.tyre_grip -= 0.0005; // Имитация износа шин после каждого круга
        car.weight -= 2.75; // Имитация сжигания топлива после каждого круга
    }
    

    int total_ms {static_cast<int>(total_time * 1000.0)};
    int minutes {total_ms / 60000};
    int seconds { (total_ms % 60000) / 1000 };
    int milliseconds { total_ms % 1000 };

    std::cout << "\n";

    print_laptime(total_time, "Total"); // "Total" indicates total time

    return 0;
}