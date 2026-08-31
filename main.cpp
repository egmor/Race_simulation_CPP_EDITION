#include <iostream>
#ifdef _WIN32
#include <windows.h>
#endif

#include "physics_engine.hpp"
#include "calculating_laptime.hpp"
#include "trajectory_selection.hpp"

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

int main() {

    #ifdef _WIN32
        SetConsoleOutputCP(65001);
        SetConsoleCP(65001);
    #endif

    std::vector<std::vector<Corner>> tracks {
        {//Suzuka detailed
            Corner(0.0, 750.0, 12.0, 0.0, false),    // Main straight
            Corner(85.0, 100.0, 12.0, 0.0, false),   // T1 (R)
            Corner(75.0, 82.0, 12.0, 0.0, false),    // T2 (R)
            Corner(55.0, 45.0, 11.0, 0.0, false),    // S1 (L)
            Corner(48.0, 48.0, 11.0, 0.0, false),    // S2 (R)
            Corner(45.0, 46.0, 11.0, 0.0, false),    // S3 (L)
            Corner(42.0, 52.0, 11.0, 0.0, false),    // S4 (R)
            Corner(55.0, 95.0, 11.0, 0.0, true),     // Dunlop (L) — в пол
            Corner(65.0, 165.0, 11.0, 0.0, false),   // Reverse Bank (L)
            Corner(25.0, 65.0, 11.0, 0.0, false),    // NIPPO (R)
            Corner(0.0, 80.0, 11.0, 0.0, false),     // to Degner
            Corner(55.0, 65.0, 10.0, 0.0, false),    // Degner 1 (L)
            Corner(70.0, 55.0, 10.0, 0.0, false),    // Degner 2 (L)
            Corner(0.0, 90.0, 10.0, 0.0, false),     // to Hairpin
            Corner(70.0, 62.0, 10.0, 0.0, false),    // Hairpin entry (L)
            Corner(80.0, 78.0, 10.0, 0.0, false),    // Hairpin apex (L)
            Corner(35.0, 50.0, 10.0, 0.0, false),    // Hairpin exit (L)
            Corner(0.0, 250.0, 10.0, 0.0, false),    // Hairpin to 200R
            Corner(35.0, 105.0, 11.0, 0.0, true),    // 200R (L) — в пол
            Corner(30.0, 85.0, 11.0, 0.0, true),     // 200R exit (L) — в пол
            Corner(0.0, 180.0, 11.0, 0.0, false),    // to Spoon
            Corner(45.0, 105.0, 11.0, 0.0, false),   // Spoon entry (L)
            Corner(50.0, 125.0, 11.0, 0.0, false),   // Spoon middle (L)
            Corner(35.0, 110.0, 11.0, 0.0, false),   // Spoon exit (L)
            Corner(0.0, 820.0, 12.0, 0.0, false),    // West straight
            Corner(0.0, 120.0, 12.0, 0.0, false),    // 130R approach
            Corner(65.0, 280.0, 12.0, 0.0, true),    // 130R (L) — в пол
            Corner(20.0, 80.0, 12.0, 0.0, true),     // 130R exit (L) — в пол
            Corner(0.0, 520.0, 11.0, 0.0, false),    // to chicane
            Corner(80.0, 45.0, 10.0, 0.0, false),    // Casio right (R)
            Corner(105.0, 45.0, 10.0, 0.0, false),   // Casio left (L)
            Corner(0.0, 65.0, 10.0, 0.0, false),     // Casio exit
            Corner(0.0, 124.0, 11.0, 0.0, false),    // Final approach
            Corner(45.0, 95.0, 12.0, 0.0, false),    // Final corner entry (R)
            Corner(40.0, 75.0, 12.0, 0.0, true),     // Final corner exit (R) — в пол
            Corner(0.0, 650.0, 12.0, 0.0, false)     // Main straight approach
        },
        {   //Suzuka
            Corner(0.0, 800.0, 12.0, 0.0, false),    // Главная прямая
            Corner(70.0, 50.0, 12.0, 0.0, false),    // First Corner (T1)
            Corner(85.0, 45.0, 11.0, 0.0, false),    // S Curves (T2-T3)
            Corner(75.0, 40.0, 11.0, 0.0, false),    // S Curves (T4-T5)
            Corner(80.0, 45.0, 11.0, 0.0, true),     // Dunlop Curve (T6) — в пол
            Corner(0.0, 350.0, 11.0, 0.0, false),    // Прямая
            Corner(105.0, 35.0, 10.0, 0.0, false),   // Degner 1 & 2 (T8-T9)
            Corner(0.0, 400.0, 10.0, 0.0, false),    // Под мост
            Corner(120.0, 30.0, 10.0, 0.0, false),   // Hairpin (T11)
            Corner(0.0, 600.0, 11.0, 0.0, false),    // Разгон
            Corner(55.0, 65.0, 11.0, 0.0, false),    // Spoon Curve (T13-T14)
            Corner(0.0, 1200.0, 12.0, 0.0, false),   // Длинный разгон к 130R
            Corner(45.0, 60.0, 12.0, 0.0, true),     // 130R (T15) — в пол
            Corner(115.0, 30.0, 10.0, 0.0, false),   // Casio Triangle Chicane (T16-T17)
            Corner(0.0, 400.0, 12.0, 0.0, false)     // Выход на старт
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
                lap_time += laptime(tracks[i]);
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