# uvd-radar

# 1. Генерация ответов (5 минут симуляции)
# 90% ошибок в Mode A, 90% в Mode C, 50% невалидных
./1_generate_replies ../radar.conf 300 replies.txt 0.9 0.9 0.5

# 50% ошибок в обоих режимах
./1_generate_replies ../radar.conf 300 replies.txt 0.5 0.5 0.3

# 100% ошибок (для теста)
./1_generate_replies ../radar.conf 300 replies.txt 1.0 1.0 0.
# --------------------------

# 2. Формирование плотов
./2_form_plots replies.txt plots.txt

# 3. Трековая обработка
./3_track_processing plots.txt tracks.txt 5.0 30.0 3 10 2.0 0.5

# 2_3 Запуск с конфигом по умолчанию
./2_3_combined

# Запуск с указанием конфига, входного и выходного файлов
./2_3_combined radar.conf replies.txt tracks_combined.txt

# 4. Визуализация с разверткой (5 секунд на оборот)
./4_radar_player replies.txt plots_combined.txt tracks_combined.txt


=====================

Текущие проблемы и задачи для улучшения:

    Синхронизация луча — единичные ответы и плоты появляются с небольшой задержкой относительно луча. Нужно точно синхронизировать отображение.

    Улучшение трекера — скорость иногда вычисляется с ошибками. Нужно настроить фильтр Калмана или добавить медианную фильтрацию.

    Обработка перекрытий — при наложении двух ответов нужно определять количество целей. Нужен алгоритм разделения перекрытий.

    Реальное время — сейчас все работает с файлами. Нужно добавить прием данных по UDP/TCP и потоковую обработку.

    Поддержка UVD — добавить обработку и отображение UVD ответов.

    Тестирование — добавить unit-тесты для всех модулей.

=====================

Рекомендации по дальнейшему развитию
1. Добавить поддержку реальных данных

    Интерфейс для подключения к сетевым источникам

    Парсеры для стандартных форматов (ASTERIX, RPF)

2. Улучшить трекер

    Добавить IMM (Interacting Multiple Model) фильтр

    Поддержка маневрирующих целей

3. Дополнительные алгоритмы

    Обнаружение разрывов траекторий

    Ассоциация треков с целями

    Экстраполяция для прогнозирования

4. GUI улучшения

    Масштабирование с панорамированием

    Выбор отображаемых треков по коду

    Показ истории трека

    Экспорт текущего кадра в изображение

5. Производительность

    Использовать GPU для обработки (CUDA/OpenCL)

    Многопоточная обработка оборотов

    Оптимизация рендеринга

Итоговый чек-лист улучшений:

    RAII для SDL ресурсов

    Оптимизация group_by_range (O(N²)→O(N log N))

    Единая система логирования

    Потокобезопасность трекера

    Структурированный парсинг конфигурации

    Качество сигнала на визуализации

    Буферизированная загрузка данных

    Взвешенная кластеризация

    Юнит-тесты

    Экспорт в различные форматы

===============================

Общая оценка проекта VRL-Radar
Сильные стороны

    Архитектура - четкое разделение на этапы обработки, потоковый пайплайн

    Качество кода - хорошо структурирован, используются современные C++17 возможности

    Реалистичность - учтены многие реальные аспекты радарной обработки (SLS, перекрытия, шумы)

    Визуализация - рабочий SDL2 плеер с удобным управлением

    Гибкость - конфигурация через файл, настраиваемые параметры

Требующие улучшения аспекты
1. Высокоуровневая архитектура
text

Текущий подход: Монолитные утилиты
Рекомендуется: Модульная библиотека + приложения

Предложение: Создать единую библиотеку libvrl_radar с четкими интерфейсами:
text

libvrl_radar/
├── core/           # Ядро обработки
├── processing/     # Алгоритмы
├── visualization/  # Визуализация
└── utils/          # Утилиты

2. Производительность и оптимизация

Проблемы:

    Линейный поиск при кластеризации O(n²)

    Отсутствие использования SIMD

    Нет многопоточной обработки

Предложения:
cpp

// 1. Пространственное индексирование для кластеризации
class SpatialIndex {
    // Использовать сетку или R-tree для быстрого поиска соседей
    std::unordered_map<uint64_t, std::vector<Reply*>> grid_cells_;
    
    uint64_t get_cell_key(uint16_t azimuth, uint16_t range) {
        const int AZ_BINS = 64;  // 4096/64
        const int RANGE_BINS = 32;
        return (azimuth / AZ_BINS) * RANGE_BINS + (range / RANGE_BINS);
    }
};

// 2. Пул объектов для избежания аллокаций
template<typename T>
class ObjectPool {
    std::vector<std::unique_ptr<T>> pool_;
    std::vector<T*> free_list_;
public:
    T* acquire();
    void release(T* obj);
};

3. Тестирование и верификация

Предложения:
cpp

// tests/test_pipeline.cpp - Сквозные тесты
class PipelineTest : public ::testing::Test {
    void run_pipeline_with_known_targets() {
        // Генерируем данные с известными треками
        // Проверяем совпадение треков
    }
};

// tests/test_kalman_filter.cpp - Юнит-тесты
TEST(KalmanFilterTest, TrackConstantVelocity) {
    KalmanFilter kf(1.0, 0.1, 0.1);
    // Проверяем, что фильтр корректно отслеживает движение
}

4. Обработка ошибок и валидация

Проблемы:

    Нет единой системы логирования

    Ошибки часто просто игнорируются

    Отсутствует валидация входных данных

Предложения:
cpp

// Единая система логирования
enum class LogLevel { DEBUG, INFO, WARNING, ERROR, FATAL };

class Logger {
    static Logger& instance() { static Logger inst; return inst; }
    void log(LogLevel level, const std::string& msg);
    void set_output(std::ostream& out);
};

#define VRL_LOG(level, msg) Logger::instance().log(level, msg)

// Валидация конфигурации
class ConfigValidator {
    bool validate(const SystemConfig& config, std::vector<std::string>& errors);
};

5. Продвинутые алгоритмы

Предложения:

A. Многогипотезный трекер (MHT)
cpp

class MHTTracker {
    struct Hypothesis {
        std::vector<Track> tracks;
        double probability;
        uint32_t last_revolution;
    };
    
    std::vector<Hypothesis> hypotheses_;
    
    void update(const std::vector<TargetReport>& targets, uint32_t rev) {
        // Генерация новых гипотез
        // Оценка вероятностей
        // Пранирование маловероятных
    }
};

B. Рекурсивный кластеризатор с адаптивными порогами
cpp

class AdaptiveClusterer {
    double adaptive_threshold() const {
        // Анализ плотности целей
        // Плотные области -> меньший порог
        // Разреженные -> больший порог
    }
};

C. Интеллектуальный решатель перекрытий с машинным обучением
cpp

class MLGarblingSolver {
    // Использовать нейросеть для разделения перекрытий
    // Обучить на синтетических данных
    
    struct Features {
        std::array<float, 18> amplitude_pattern;
        float snr_estimate;
        // ...
    };
    
    std::unique_ptr<onnxruntime::Session> model_;
};

6. Визуализация и UX

Улучшения для 4_radar_player.cpp:
cpp

class EnhancedRadarPlayer {
    // 1. Настраиваемая палитра цветов
    struct ColorScheme {
        SDL_Color background{0,0,0,255};
        SDL_Color rbs_track{0,255,0,255};
        SDL_Color uvd_track{0,255,255,255};
        // ...
    };
    
    // 2. Различные режимы отображения
    enum DisplayMode {
        FULL_HISTORY,      // Все треки
        CURRENT_ONLY,      // Только текущие
        HEATMAP,           // Тепловая карта плотности
        PREDICTIVE         // С прогнозированием
    };
    
    // 3. Интерактивный выбор объектов
    void handle_click(int x, int y) {
        auto selected = find_nearest_track(x, y);
        show_track_details(selected);
    }
    
    // 4. Экспорт данных
    void export_to_csv(const std::string& filename);
};

7. Интеграция с реальными данными

Предложения:
cpp

// Адаптеры для различных форматов данных
class DataSource {
public:
    virtual std::vector<Reply> read_next_scan() = 0;
    virtual bool has_next() = 0;
};

class ASTERIXSource : public DataSource {
    // Чтение данных в формате ASTERIX CAT048
};

class JSONSource : public DataSource {
    // Чтение из JSON-логов
};

class NetworkSource : public DataSource {
    // Получение данных по сети (UDP/TCP)
};

// Обработка в реальном времени
class RealTimeProcessor {
    std::unique_ptr<DataSource> source_;
    std::unique_ptr<RadarSystem> system_;
    
    void run_loop() {
        while (source_->has_next()) {
            auto scan = source_->read_next_scan();
            system_->process_scan(scan);
            
            // Отображение с задержкой для визуализации
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
};

8. Метрики и валидация качества
cpp

class QualityMetrics {
    struct Metrics {
        double track_accuracy;      // Сравнение с ground truth
        double track_completeness;   // Сколько целей обнаружено
        double false_alarm_rate;     // Ложные треки
        double track_stability;      // Стабильность треков
    };
    
    Metrics evaluate(const std::vector<Track>& tracks, 
                     const std::vector<GeneratedTarget>& ground_truth) {
        // Сравнение треков с реальными целями
        // Анализ ошибок и пропусков
    }
};

9. Документация и удобство использования

Предложения:

    Doxygen документация для всех классов

    Примеры использования в examples/

    Более подробный README с архитектурой

    Скрипты для автоматического тестирования

10. Масштабирование
cpp

// Поддержка нескольких радаров одновременно
class MultiRadarFusion {
    std::vector<std::unique_ptr<RadarSystem>> radars_;
    std::unique_ptr<TrackFusion> fusion_;
    
    void add_radar(const SystemConfig& config);
    void process_all();
    std::vector<FusedTrack> get_fused_tracks();
};

// Координатное преобразование для разных радаров
class CoordinateTransformer {
    std::map<std::string, Eigen::Matrix4d> transforms_;
    void add_transform(const std::string& radar_id, const Eigen::Matrix4d& t);
    Eigen::Vector3d transform(const std::string& from, const std::string& to, 
                              const Eigen::Vector3d& point);
};

Приоритеты развития
Немедленные улучшения (1-2 недели)

    ✅ Добавить систему логирования

    ✅ Создать юнит-тесты для ключевых компонентов

    ✅ Оптимизировать кластеризацию (пространственный индекс)

    ✅ Документировать API

Среднесрочные (1-2 месяца)

    🔄 Разработать MHT-трекер

    🔄 Добавить обработку реальных данных (ASTERIX)

    🔄 Реализовать многопоточную обработку

    🔄 Создать веб-интерфейс для визуализации

Долгосрочные (3-6 месяцев)

    🎯 Интеграция с ML для решения перекрытий

    🎯 Поддержка нескольких радаров

    🎯 Автоматическая калибровка параметров

    🎯 Система мониторинга качества в реальном времени

Заключение

Проект имеет отличную основу и демонстрирует глубокое понимание предметной области. Основной рекомендацией является переход от набора утилит к полноценной библиотеке с четкой архитектурой, добавление тестирования и дальнейшее развитие алгоритмов в сторону более продвинутых методов (MHT, ML, адаптивные алгоритмы). Это позволит создать профессиональный продукт, готовый к использованию в реальных системах.


====================


# 1. Генерация ответов
./tools/1_generate_replies ../config/radar.conf 300 replies.txt

# 2. Обработка
./tools/2_3_combined ../config/radar.conf replies.txt tracks_combined.txt

# 3. Визуализация
./tools/4_radar_player replies.txt plots_combined.txt tracks_combined.txt

====================
