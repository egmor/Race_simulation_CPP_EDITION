#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>

struct Segment {
    double distance{0.0};        // Длина участка в метрах
    double degree{0.0};          // Угол поворота (0 для прямых)
    double height_difference{0.0};// Разница высот
    bool is_straight{false};     // Флаг: прямая это или поворот

    // Конструктор для поворота
    Segment(double l, double a, double h) 
        : distance(l), degree(a), height_difference(h), is_straight(false) {}

    // Перегруженный конструктор для прямой
    Segment(double l) 
        : distance(l), degree(0.0), height_difference(0.0), is_straight(true) {}
};

double calculate_segment_time(const Segment& seg) {
    // прямая
    if (seg.is_straight) {
        const double avg_straight_speed = 58.0; 
        return seg.distance / avg_straight_speed;
    }

    // поворот
    double tyre_grip {1.6}; 
    const double g {9.81}; 
    
    // Радиус поворота
    double corner_radius {(seg.distance) / (2.0 * std::sin(seg.degree * 3.14159 / 180.0))}; 
    
    // Уклон
    double longitudinal_slope {std::asin(std::abs(seg.height_difference) / seg.distance)}; 

    // Максимальная скорость в повороте
    double max_speed_corner = std::sqrt(tyre_grip * g * corner_radius * std::cos(longitudinal_slope)); 

    return seg.distance / max_speed_corner;
}

int main() {
    std::vector<Segment> track = {
        // 1. Главная прямая (Rettifilo) — старт/финиш
        Segment(1120.0), 

        // 2-3. Variante del Rettifilo (T1–T2) — медленная шикана
        Segment(70.0, 110.0, 0.0),  // T1 (правый)
        Segment(60.0, 100.0, 0.0),  // T2 (левый)

        // 4. Прямая к Curva Grande
        Segment(480.0),

        // 5. Curva Grande / Curva del Serraglio (T3) — затяжной быстрый правый
        Segment(320.0, 35.0, -3.0),

        // 6. Прямая перед второй шиканой
        Segment(420.0),

        // 7-8. Variante della Roggia (T4–T5) — среднескоростная шикана
        Segment(80.0, 90.0, 0.0),   // T4 (левый)
        Segment(75.0, 85.0, 0.0),   // T5 (правый)

        // 9. Прямая Лесмо
        Segment(260.0),

        // 10. Curva di Lesmo 1 (T6) — среднескоростной правый
        Segment(110.0, 65.0, -2.0),

        // 11. Короткая прямая между Лесмо
        Segment(150.0),

        // 12. Curva di Lesmo 2 (T7) — правый с выходом на длинную прямую
        Segment(120.0, 70.0, -4.0),

        // 13. Прямая Serraglio (с проходом под мостом)
        Segment(950.0),

        // 14-16. Variante Ascari (T8–T10) — быстрая связка
        Segment(90.0, 75.0, 0.0),   // T8 (вход влево)
        Segment(80.0, 60.0, 0.0),   // T9 (смена направления вправо)
        Segment(100.0, 55.0, 0.0),  // T10 (выход влево)

        // 17. Прямая перед Параболикой
        Segment(600.0),

        // 18. Curva Parabolica / Curva Alboreto (T11) — затяжной правый на главную прямую
        Segment(348.0, 42.0, 0.0)
    };

    // Считаем общее время прохождения трека
    double total_time {0.0};
    for (const auto& seg : track) {
        total_time += calculate_segment_time(seg);
    }

    int minutes = static_cast<int>(total_time) / 60;
    int seconds = static_cast<int>(total_time) % 60;
    int milliseconds = static_cast<int>((total_time - static_cast<int>(total_time)) * 1000);

    std::cout << "Общее время прохождения трека: " << minutes << ":" << seconds << "." << milliseconds;
    return 0;
}