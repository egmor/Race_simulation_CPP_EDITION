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
        {   //Monza GP
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
            Corner(600.0, 0.0, -2.5),    // Прямая перед Параболикой с небольшим уклоном вниз и влево
            Corner(348.6, -33., -3.)     // Parabolica с уклоном вниз и влево (внутренний радиус)
        },
        {   //Spa GP
            Corner(850.0, 0.0, -10.0),   // Главная прямая (спуск к La Source)
            Corner(30.0, 120.0, -1.0),   // La Source (T1)
            Corner(450.0, 0.0, -15.0),   // Спуск к Eau Rouge
            Corner(60.0, 35.0, 5.0),     // Eau Rouge (T2)
            Corner(80.0, 45.0, 18.0),    // Raidillon (T3-T4) — крутой подъём
            Corner(780.0, 0.0, 8.0),     // Прямая Kemmel Straight
            Corner(45.0, 85.0, 0.0),     // Les Combes T5
            Corner(40.0, 80.0, 0.0),     // Les Combes T6
            Corner(150.0, 0.0, -4.0),    // Прямая к Malmedy
            Corner(50.0, 65.0, -3.0),    // Malmedy (T7)
            Corner(320.0, 0.0, -12.0),   // Спуск к Rivage
            Corner(70.0, 95.0, -6.0),    // Bruxelles / Rivage (T8)
            Corner(110.0, 50.0, -4.0),   // T9
            Corner(300.0, 0.0, -8.0),    // Спуск к Pouhon
            Corner(220.0, 85.0, -2.0),   // Pouhon (T10-T11) — скоростная дуга
            Corner(250.0, 0.0, 3.0),     // Прямая к Fagnes
            Corner(55.0, 75.0, 1.0),     // Fagnes T12
            Corner(50.0, 70.0, 0.0),     // Fagnes T13
            Corner(180.0, 0.0, 2.0),     // Прямая к Stavelot
            Corner(90.0, 60.0, 1.0),     // Stavelot (T14-T15)
            Corner(950.0, 0.0, 5.0),     // Прямая Blanchimont
            Corner(260.0, 30.0, 1.0),    // Blanchimont (T16) — плоский скоростной поворот
            Corner(500.0, 0.0, 0.0),     // Подход к Bus Stop
            Corner(35.0, 110.0, 0.0),    // Bus Stop Chicane T17
            Corner(35.0, 105.0, 0.0)     // Bus Stop Chicane T18
        },
        {   //Nürburgring GP
            Corner(700.0, 0.0, -5.0),    // Старт/Финиш
            Corner(45.0, 115.0, -2.0),   // Yokohama T1
            Corner(120.0, 60.0, 1.0),    // Mercedes Arena (T2)
            Corner(140.0, 130.0, 3.0),   // Mercedes Arena (T3)
            Corner(150.0, 75.0, 0.0),    // Выход из Arena (T4)
            Corner(380.0, 0.0, -8.0),    // Спуск к Kurve 5
            Corner(110.0, 70.0, -4.0),   // T5
            Corner(220.0, 0.0, -5.0),    // Прямая к Ford Kurve
            Corner(130.0, 80.0, -3.0),   // Ford Kurve (T6)
            Corner(310.0, 0.0, 6.0),     // Подъём к Dunlop
            Corner(150.0, 105.0, 4.0),   // Dunlop Hairpin (T7)
            Corner(460.0, 0.0, 0.0),     // Прямая к Schumacher S
            Corner(140.0, 45.0, -2.0),   // Schumacher S (T8)
            Corner(130.0, 50.0, 0.0),    // Schumacher S (T9)
            Corner(280.0, 0.0, 2.0),     // Прямая к Kumho
            Corner(110.0, 75.0, 1.0),    // Kumho Kurve (T10)
            Corner(620.0, 0.0, 5.0),     // Прямая Backstraight
            Corner(40.0, 90.0, 0.0),     // NGK Chicane (T11)
            Corner(40.0, 85.0, 0.0),     // NGK Chicane (T12)
            Corner(180.0, 0.0, -1.0),    // Прямая к Coca-Cola Kurve
            Corner(115.0, 75.0, 0.0)     // Coca-Cola Kurve (T13)
        },
        {   //Red Bull Ring GP
            Corner(800.0, 0.0, 12.0),    // Старт/Финиш (крутой подъём)
            Corner(50.0, 95.0, 2.0),     // Niki Lauda Corner (T1)
            Corner(880.0, 0.0, 25.0),    // Прямая вверх по холму
            Corner(35.0, 125.0, 2.0),    // Remus Hairpin (T3) — главное место для обгонов
            Corner(820.0, 0.0, -20.0),   // Крутой спуск к Schlossgold
            Corner(55.0, 90.0, -5.0),    // Schlossgold (T4)
            Corner(160.0, 0.0, -3.0),    // Прямая
            Corner(110.0, 55.0, -2.0),   // Rauch (T5)
            Corner(140.0, 60.0, 0.0),    // T6
            Corner(210.0, 0.0, 1.0),     // Прямая
            Corner(120.0, 65.0, 0.0),    // Wurth (T7)
            Corner(150.0, 0.0, 0.0),     // Прямая к финальным поворотам
            Corner(100.0, 70.0, -2.0),   // Rindt (T9)
            Corner(110.0, 75.0, -1.0)    // Red Bull Mobile (T10)
        }
    };

    std::vector<std::string> track_names = {"Monza GP", "Spa GP", "Nurburgring GP", "Red Bull Ring GP"};

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