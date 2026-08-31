#include <iostream>
#ifdef _WIN32
#include <windows.h>
#endif
#include "physics_engine.hpp"

struct Driver {
    double braking_skill{1.0};   
    double cornering_skill{1.0}; 
    double consistency{1.0};     
    double reaction_time{0.10};  
    
    [[maybe_unused]] double racecraft{1.0}; 

    double realization_speed_car() const {
        return 0.95 + (cornering_skill * 0.05); 
    }

    double get_braking_margin() const {
        return 1.0 + (1.0 - braking_skill) * 0.08; 
    }
};

void print_laptime(double laptime, const std::string& lap_number) {
    int total_ms = static_cast<int>(std::round(laptime * 1000.0));
    int minutes {total_ms / 60000};
    int seconds { (total_ms % 60000) / 1000 };
    int milliseconds { total_ms % 1000 };

    std::cout << lap_number << " time: "
              << minutes << ":" 
              << (seconds < 10 ? "0" : "") << seconds << "." 
              << (milliseconds < 100 ? (milliseconds < 10 ? "00" : "0") : "") << milliseconds << "\n";
}

double calculate_segment_time(const std::vector<Corner>& track, size_t current_idx, const Driver& driver, double& speed_enter) {
    const Corner& seg = track[current_idx];
    double time_segment{0.0};   

    // --- 1. ПРЯМОЙ УЧАСТОК ---
    if (std::abs(seg.degree) < 0.001) {
        // get_target_speed_ahead возвращает м/с
        double target_speed = get_target_speed_ahead(track, current_idx, speed_enter) * driver.realization_speed_car();
        
        double distance_left {seg.distance};
        double total_time {0.0};
        const double dt {0.005};

        while (distance_left > 0.0) {
            bool need_braking {false};
            if (speed_enter > target_speed) {
                // ВАЖНО: decel растёт со скоростью (больше прижимной силы = больше сцепления),
                // поэтому нельзя брать decel на ТЕКУЩЕЙ (ещё высокой) скорости — это занижает
                // тормозной путь, машина тормозит слишком поздно и не успевает погасить скорость.
                // Берём decel на целевой (более низкой) скорости — консервативная нижняя граница,
                // которая гарантированно даёт достаточно места для торможения.
                double conservative_decel = get_decel_forces(target_speed);
                // Тормозной путь: S = (v1^2 - v2^2) / (2 * a) + путь за время реакции водителя
                double reaction_dist = speed_enter * driver.reaction_time;
                double req_brake_dist = ((speed_enter * speed_enter - target_speed * target_speed) / (2.0 * conservative_decel)) * driver.get_braking_margin() + reaction_dist;
                
                if (distance_left <= req_brake_dist) {
                    need_braking = true;
                }
            }

            double accel {0.0};
            if (need_braking) {
                accel = -get_decel_forces(speed_enter);
            } else {
                accel = get_current_accelerate(speed_enter);
            }

            speed_enter += accel * dt;
            const double max_gearbox_speed = 285.0 / 3.6;
            speed_enter = std::min(speed_enter, max_gearbox_speed);
            
            if (need_braking && speed_enter < target_speed) {
                speed_enter = target_speed;
            }

            double ds {speed_enter * dt};
            distance_left -= ds;
            total_time += dt;
        }

        return total_time;
    }

    // --- 2. ПОВОРОТ ---
    double max_possible_enter_speed {max_speed_apex(calculating_radius(seg))}; // Значение в м/с
    if (seg.degree > 75.0) {
        max_possible_enter_speed *= 0.88;
    }
    max_possible_enter_speed *= driver.realization_speed_car();

    // Сравниваем М/С с М/С (без умножения на 3.6 внутри условия!)
    if (speed_enter > max_possible_enter_speed * 1.01) {
        std::cout << "[ВНИМАНИЕ] Вылет с трассы в повороте! Вход: " 
                << speed_enter * 3.6 << " км/ч (Макс: " 
                << max_possible_enter_speed * 3.6 << " км/ч)\n";
        speed_enter = max_possible_enter_speed;
    }

    // Целевая скорость к концу поворота: если сразу за ним ещё один поворот —
    // нужно быть готовым войти в него на его лимите. Если дальше прямая —
    // сдерживать скорость не нужно (прямая сама разгонится/затормозит),
    // поэтому цель — собственный лимит этого поворота.
    size_t next_idx { (current_idx + 1) % track.size() };
    double target_speed {max_possible_enter_speed};
    if (std::abs(track[next_idx].degree) > 0.001) {
        double next_max_speed {max_speed_apex(calculating_radius(track[next_idx]))};
        if (track[next_idx].degree > 75.0) {
            next_max_speed *= 0.88;
        }
        next_max_speed *= driver.realization_speed_car();
        target_speed = std::min(target_speed, next_max_speed);
    }

    // ВАЖНО: в повороте машина не просто "плывёт" от входа к выходу — она
    // может и разгоняться (если далеко до следующего ограничения по скорости),
    // и тормозить (если нужно успеть погасить скорость к следующему повороту).
    // Раньше здесь брался min(вход, лимит_поворота, лимит_след.поворота), из-за
    // чего скорость в цепочке поворотов могла только падать и никогда не росла,
    // даже если следующий поворот был гораздо быстрее текущего.
    double distance_left {seg.distance};
    double total_time {0.0};
    const double dt {0.005};

    while (distance_left > 0.0) {
        bool need_braking {false};
        if (speed_enter > target_speed) {
            double conservative_decel = get_decel_forces(target_speed);
            double reaction_dist = speed_enter * driver.reaction_time;
            double req_brake_dist = ((speed_enter * speed_enter - target_speed * target_speed) / (2.0 * conservative_decel)) * driver.get_braking_margin() + reaction_dist;
            if (distance_left <= req_brake_dist) {
                need_braking = true;
            }
        }

        double accel {0.0};
        if (need_braking) {
            accel = -get_decel_forces(speed_enter);
        } else {
            accel = get_current_accelerate(speed_enter);
        }

        speed_enter += accel * dt;
        speed_enter = std::min(speed_enter, max_possible_enter_speed); // нельзя ехать быстрее предела сцепления в этом повороте

        if (need_braking && speed_enter < target_speed) {
            speed_enter = target_speed;
        }

        double ds {speed_enter * dt};
        distance_left -= ds;
        total_time += dt;
    }

    time_segment = total_time;

    return time_segment;
}

int main() {

#ifdef _WIN32
    SetConsoleOutputCP(65001);
    SetConsoleCP(65001);
#endif

    Driver driver;
    std::vector<std::vector<Corner>> tracks = {
        {   //Suzuka GP Detailed
            Corner( 750.0,     0.0,  0.0),   // Main straight
            Corner( 100.0,    85.0,  0.0),   // T1 (R)
            Corner(  82.0,    75.0,  0.0),   // T2 (R)
            Corner(  45.0,   55.0,  0.0),   // S1 (L)
            Corner(  48.0,    48.0,  0.0),   // S2 (R)
            Corner(  46.0,   45.0,  0.0),   // S3 (L)
            Corner(  52.0,    42.0,  0.0),   // S4 (R)
            Corner(  95.0,   55.0,  0.0),   // Dunlop (L)
            Corner( 165.0,   65.0,  0.0),   // Reverse Bank (L)
            Corner(  65.0,    25.0,  0.0),   // NIPPO (R)
            Corner(  80.0,     0.0,  0.0),   // to Degner
            Corner(  65.0,   55.0,  0.0),   // Degner 1 (L)
            Corner(  55.0,   70.0,  0.0),   // Degner 2 (L)
            Corner(  90.0,     0.0,  0.0),   // to Hairpin
            Corner(  62.0,   70.0,  0.0),   // Hairpin entry (L)
            Corner(  78.0,   80.0,  0.0),   // Hairpin apex (L)
            Corner(  50.0,   35.0,  0.0),   // Hairpin exit (L)
            Corner( 250.0,     0.0,  0.0),   // Hairpin to 200R
            Corner( 105.0,   35.0,  0.0),   // 200R (L)
            Corner(  85.0,   30.0,  0.0),   // 200R exit (L)
            Corner( 180.0,     0.0,  0.0),   // to Spoon
            Corner( 105.0,   45.0,  0.0),   // Spoon entry (L)
            Corner( 125.0,   50.0,  0.0),   // Spoon middle (L)
            Corner( 110.0,   35.0,  0.0),   // Spoon exit (L)
            Corner( 820.0,     0.0,  0.0),   // West straight
            Corner( 120.0,     0.0,  0.0),   // 130R approach
            Corner( 280.0,   65.0,  0.0),   // 130R (L)
            Corner(  80.0,   20.0,  0.0),   // 130R exit (L)
            Corner( 520.0,     0.0,  0.0),   // to chicane
            Corner(  45.0,    80.0,  0.0),   // Casio right (R)
            Corner(  45.0,  105.0,  0.0),   // Casio left (L)
            Corner(  65.0,     0.0,  0.0),   // Casio exit
            Corner( 124.0,     0.0,  0.0),   // Final approach
            Corner(  95.0,    45.0,  0.0),   // Final corner entry (R)
            Corner(  75.0,    40.0,  0.0),   // Final corner exit (R)
            Corner( 650.0,     0.0,  0.0),   // Main straight approach
        },
        {   //Suzuka GP
            {800.0, 0.0},     // Главная прямая
            {50.0, 70.0},     // First Corner (T1)
            {45.0, 85.0},     // S Curves (T2-T3)
            {40.0, 75.0},     // S Curves (T4-T5)
            {45.0, 80.0},     // Dunlop Curve (T6)
            {350.0, 0.0},     // Прямая
            {35.0, 105.0},    // Degner 1 & 2 (T8-T9)
            {400.0, 0.0},     // Под мост
            {30.0, 120.0},    // Hairpin (T11) — шпилька
            {600.0, 0.0},     // Разгон
            {65.0, 55.0},     // Spoon Curve (T13-T14)
            {1200.0, 0.0},    // Длинный разгон к 130R
            {60.0, 45.0},     // 130R (T15) — газ в пол
            {30.0, 115.0},    // Casio Triangle Chicane (T16-T17)
            {400.0, 0.0}      // Выход на старт
        }
    };

    std::vector<std::string> track_names = {"Suzuka GP", "Suzuka GP (Detailed)"};

    for (size_t t = 0; t < tracks.size(); ++t) {
        std::cout << "=== " << track_names[t] << " ===\n";

        double speed {80.0 / 3.6}; // Начальная скорость в м/с (80 км/ч)
        double total_time{0.0};

        for (size_t lap = 0; lap < 50; ++lap) {
            double lap_time {0.0};
            for (size_t i = 0; i < tracks[t].size(); ++i) {
                lap_time += calculate_segment_time(tracks[t], i, driver, speed);
            }
            print_laptime(lap_time,"Lap " + std::to_string(lap + 1) + (lap + 1 < 10 ? " " : "") ); // Вывод времени круга с выравниванием для однозначных чисел
            total_time += lap_time;
        }

        std::cout << "\n";
        print_laptime(total_time, "Total"); // "Total" indicates total time
        std::cout << "\n";
    }

    return 0;
}