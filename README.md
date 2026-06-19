
./tools/1_generate_replies ../config/radar.json 300 replies.txt
./tools/2_3_combined ../config/radar.json replies.txt tracks_combined.txt
./tools/4_radar_player replies.txt plots_combined.txt tracks_combined.txt

./tools/1_generate_replies
./tools/2_3_combined
./tools/4_radar_player


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


cmake .. -DCMAKE_BUILD_TYPE=Release -DENABLE_VERBOSE_LOGGING=ON


# 1. Генерация ответов
./tools/1_generate_replies ../config/radar.conf 300 replies.txt

# 2. Обработка
./tools/2_3_combined ../config/radar.conf replies.txt tracks_combined.txt

# 3. Визуализация
./tools/4_radar_player replies.txt plots_combined.txt tracks_combined.txt

====================

Критические проблемы
1. Дублирование реализации

Файлы garbling_solver_iterative.cpp и garbling_solver_threshold.cpp содержат одинаковый код (оба содержат реализацию IterativeSubtractionSolver). Это приведет к ошибкам линковки.

Решение:

    Удалить garbling_solver_threshold.cpp

    Переименовать garbling_solver_iterative.cpp → garbling_solver.cpp

    Объединить обе реализации в одном файле

cpp

// garbling_solver.cpp должно содержать:
// 1. ThresholdGarblingSolver
// 2. IterativeSubtractionSolver

2. Отсутствие реализации UVD в garbling_solver

Методы separate_uvd в обоих солверах имеют заглушки (TODO: Implement).

Решение: Реализовать полную обработку UVD:
cpp

SeparationResult<UVDReply> ThresholdGarblingSolver::separate_uvd(
    const std::vector<UVDReply>& mixture,
    const std::vector<uint32_t>& expected_data) {
    
    // Аналогично RBS, но для 20-битных данных
    std::array<uint8_t, UVDReply::ETHER_POSITIONS> total{};
    // ... сложение и обнаружение бит
}

3. Отсутствие валидации range_bin в ClusterProcessor

В process_rbs_group используется config_.range_bin_rbs, но нет проверки, что группа действительно принадлежит этому типу.

Решение:
cpp

std::optional<TargetReport> ClusterProcessor::process_rbs_group(const RangeGroup& group) {
    // Проверка, что группа содержит RBS
    if (group.rbs_replies.empty()) return std::nullopt;
    
    // Проверка, что группа не содержит UVD (иначе это смешанная группа)
    if (!group.uvd_replies.empty()) {
        VRL_LOG_WARN(modules::CLUSTER, "Mixed RBS/UVD group detected");
        // Можно обработать отдельно
    }
    // ... остальной код
}

Важные улучшения
4. Сборка - неоптимальные include директории

Проблема: include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include) в корневом CMakeLists.txt делает все заголовки глобально доступными.

Решение: Использовать target_include_directories:
cmake

# В src/CMakeLists.txt
target_include_directories(vrl_radar
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/../include
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
)

target_include_directories(vrl_radar SYSTEM PUBLIC
    ${EIGEN3_INCLUDE_DIR}
)

5. Отсутствие обработки ошибок в ConfigParser

Проблема: При парсинге чисел нет проверки на корректность формата.

Решение:
cpp

template<typename T>
static std::optional<T> safe_stod(const std::string& str) {
    if (str.empty()) return std::nullopt;
    
    std::string cleaned = trim(str);
    if (cleaned.empty()) return std::nullopt;
    
    // Проверка, что строка содержит только допустимые символы
    bool has_digit = false;
    for (char c : cleaned) {
        if (std::isdigit(c) || c == '.' || c == '-' || c == '+') {
            if (std::isdigit(c)) has_digit = true;
        } else if (c != 'e' && c != 'E') {
            return std::nullopt;
        }
    }
    if (!has_digit) return std::nullopt;
    
    try {
        // ... существующий код
    } catch (const std::exception& e) {
        VRL_LOG_ERROR(modules::CONFIG, "Failed to parse value: " + str);
        return std::nullopt;
    }
}

6. Утечка памяти в ReplySimulator

Проблема: Нет управления ресурсами при генерации большого количества ответов.

Решение: Добавить возможность ограничения памяти:
cpp

class ReplySimulator {
public:
    // Добавить лимит на количество хранимых ответов
    void set_max_replies(size_t max) { max_replies_ = max; }
    
    // Использовать move-семантику
    OverlapResult mix_two_rbs(RBSReply&& r1, RBSReply&& r2, ...) {
        // ... обработка без копирования
    }
};

7. Медленная кластеризация

Проблема: OnlineClusterer::check_completed_clusters выполняется линейно по всем кластерам.

Решение: Использовать spatial index:
cpp

class OnlineClusterer {
private:
    // Использовать std::map для быстрого поиска по диапазону
    std::map<uint16_t, std::vector<Cluster*>> azimuth_index_;
    
    void update_index(Cluster& cluster) {
        // Индексация по азимуту
        uint16_t az_bucket = cluster.min_azimuth / azimuth_threshold_bins_;
        azimuth_index_[az_bucket].push_back(&cluster);
    }
};

Улучшения производительности
8. Копирование больших структур

Проблема: Повсеместное копирование std::array<uint8_t, 80> в UVDReply.

Решение: Использовать const ссылки и move-семантику:
cpp

// Вместо
void add_reply(const Reply& r) { replies.push_back(r); }

// Использовать
void add_reply(Reply&& r) { replies.push_back(std::move(r)); }
void add_reply(const Reply& r) { replies.push_back(r); } // fallback

9. Замена std::map на std::unordered_map

Проблема: В местах с частым доступом по ключу используется std::map (O(log n)).

Решение:
cpp

// cluster.cpp
// std::map<uint16_t, RangeGroup> range_map;
// → 
std::unordered_map<uint16_t, RangeGroup> range_map;
range_map.reserve(100); // если ожидаем ~100 групп

10. Оптимизация логгера

Проблема: Логгер использует блокировки для каждой записи.

Решение: Использовать потокобезопасный буфер:
cpp

class Logger {
private:
    class AsyncLogger {
        std::queue<std::string> buffer_;
        std::thread worker_;
        std::atomic<bool> running_{true};
        // ... асинхронная запись
    };
    std::unique_ptr<AsyncLogger> async_logger_;
};

Функциональные улучшения
11. Добавить валидацию конфигурации
cpp

class ConfigValidator {
public:
    static bool validate(const SystemConfig& config, std::string& error) {
        // Проверка наличия целей
        if (!config.has_targets()) {
            error = "No targets defined";
            return false;
        }
        
        // Проверка корректности параметров
        if (config.radar.range_bin_rbs <= 0) {
            error = "Invalid range_bin_rbs";
            return false;
        }
        
        // Проверка уникальности кодов
        std::set<uint16_t> codes;
        for (const auto& t : config.rbs_targets) {
            if (codes.count(t.rbs_code_octal)) {
                error = "Duplicate RBS code: " + std::to_string(t.rbs_code_octal);
                return false;
            }
            codes.insert(t.rbs_code_octal);
        }
        
        return true;
    }
};

12. Добавить сохранение состояния трекера
cpp

class TrackManager {
public:
    // Сериализация
    bool save_state(const std::string& filename) const {
        std::ofstream file(filename, std::ios::binary);
        // Записать количество треков, их состояния, фильтры Калмана
        return file.good();
    }
    
    // Десериализация
    bool load_state(const std::string& filename) {
        std::ifstream file(filename, std::ios::binary);
        // Восстановить состояние
        return file.good();
    }
};

13. Добавить метрики производительности
cpp

class PerformanceMetrics {
public:
    void record_processing_time(const std::string& stage, double seconds);
    void record_reply_count(const std::string& type, size_t count);
    void record_track_count(size_t count);
    
    void print_report() const {
        VRL_LOG_INFO(modules::MAIN, "=== Performance Report ===");
        // Вывод статистики
    }
    
private:
    std::map<std::string, std::vector<double>> processing_times_;
    size_t total_replies_ = 0;
    size_t total_plots_ = 0;
    size_t total_tracks_ = 0;
};

Исправление багов
14. Потенциальное переполнение в azimuth_diff
cpp

// В cluster.cpp
int16_t gap = current_azimuth - last_reply_azimuth;
if (gap < 0) gap += 4096;
// Проблема: gap может выйти за пределы int16_t

Исправление:
cpp

int gap = static_cast<int>(current_azimuth) - static_cast<int>(last_reply_azimuth);
if (gap < 0) gap += 4096;
return gap <= max_gap_azimuth; // gap теперь int

15. Деление на ноль в ReplyProcessor
cpp

double ReplyProcessor::estimate_snr(const RBSReply& reply) {
    // ...
    double avg_signal = signal / signal_count; // signal_count может быть 0
    double avg_noise = noise / noise_count;    // noise_count может быть 0
    // ...
}

Исправление:
cpp

double ReplyProcessor::estimate_snr(const RBSReply& reply) {
    // ...
    if (signal_count == 0 || noise_count == 0) return 0.0;
    double avg_signal = signal / signal_count;
    double avg_noise = noise / noise_count;
    if (avg_noise == 0) return 30.0;
    // ...
}

Архитектурные улучшения
16. Паттерн Observer для уведомлений
cpp

// Трекер уведомляет о новых треках
class TrackObserver {
public:
    virtual void on_track_created(const Track& track) {}
    virtual void on_track_updated(const Track& track) {}
    virtual void on_track_dropped(uint64_t id) {}
};

class TrackManager {
    std::vector<TrackObserver*> observers_;
    
    void notify_track_created(const Track& track) {
        for (auto* obs : observers_) {
            obs->on_track_created(track);
        }
    }
};

17. Фабрика для GarblingSolver
cpp

enum class SolverType {
    THRESHOLD,
    ITERATIVE,
    ML_BASED
};

class GarblingSolverFactory {
public:
    static std::unique_ptr<GarblingSolver> create(SolverType type, const RadarConfig& config) {
        switch (type) {
            case SolverType::THRESHOLD:
                return std::make_unique<ThresholdGarblingSolver>(config);
            case SolverType::ITERATIVE:
                return std::make_unique<IterativeSubtractionSolver>(config);
            default:
                return std::make_unique<ThresholdGarblingSolver>(config);
        }
    }
};

Тестирование
18. Добавить unit-тесты
cmake

# tests/CMakeLists.txt
add_executable(test_radar
    test_config.cpp
    test_cluster.cpp
    test_kalman.cpp
)

target_link_libraries(test_radar
    vrl_radar
    GTest::GTest
)

add_test(NAME test_radar COMMAND test_radar)

cpp

// tests/test_config.cpp
TEST(ConfigParserTest, ParseFloat) {
    ConfigParser parser;
    parser.load("test.conf");
    auto val = parser.get<double>("range_bin_rbs");
    EXPECT_TRUE(val.has_value());
    EXPECT_DOUBLE_EQ(*val, 30.0);
}

Документация
19. Добавить Doxygen документацию
cpp

/**
 * @brief Обрабатывает кластер ответов и формирует целевые отчеты
 * 
 * @param cluster Кластер, содержащий группу связанных ответов
 * @return std::vector<TargetReport> Список обнаруженных целей
 * 
 * @note Кластер должен содержать как минимум min_hits_ ответов
 * @warning При включенном split_garbled_ возможно разделение перекрывающихся ответов
 */
std::vector<TargetReport> process_cluster(const TargetCluster& cluster);

20. Добавить README с примерами
markdown

# VRL-Radar

## Быстрый старт

```bash
# Сборка
mkdir build && cd build
cmake .. -DBUILD_TOOLS=ON
make -j4

# Генерация данных
./tools/1_generate_replies ../config/radar.conf 300 replies.txt

# Обработка
./tools/2_3_combined ../config/radar.conf replies.txt tracks.txt

# Визуализация
./tools/4_radar_player replies.txt plots.txt tracks.txt

Формат данных
replies.txt
Поле	Тип	Описание
time_sec	double	Время в секундах
azimuth	uint16_t	Азимут в дискретах (0-4095)
range	uint16_t	Дальность в дискретах
type	string	Тип ответа (RBS_A, RBS_C, UVD_DATA, UVD_ALT)
text


---

## Приоритет исправлений

| Приоритет | Задача | Сложность |
|-----------|--------|-----------|
| 🔴 Критический | Удалить дублирующиеся файлы garbling_solver | Низкая |
| 🔴 Критический | Реализовать UVD в garbling_solver | Средняя |
| 🟡 Высокий | Исправить деление на ноль | Низкая |
| 🟡 Высокий | Исправить переполнение azimuth_diff | Низкая |
| 🟡 Высокий | Валидация конфигурации | Средняя |
| 🟢 Средний | Spatial index для кластеризации | Высокая |
| 🟢 Средний | Move-семантика для больших структур | Средняя |
| 🟢 Средний | Unit-тесты | Средняя |
| 🔵 Низкий | Doxygen документация | Низкая |
| 🔵 Низкий | Асинхронный логгер | Высокая |

---

## Итоговая оценка

**Общая оценка проекта: 7.5/10**

Проект имеет хорошую архитектуру и работает как задумано. Основные проблемы связаны с дублированием кода и неполной реализацией некоторых компонентов. После исправления критических ошибок и добавления тестов проект может быть оценен на 9/10.



==============

./tools/1_generate_replies ../config/radar.json 300 replies.txt
./tools/2_3_combined ../config/radar.json replies.txt plots_combined.txt
./tools/4_radar_player replies.txt plots_combined.txt tracks_combined.txt


Проблемы и предложения по улучшению
🔴 Критические проблемы
1. Дублирование кода в Kalman Filter

В kalman_filter.cpp и tracker.cpp есть дублирование класса RevolutionKalmanFilter. Это серьезная проблема:
cpp

// kalman_filter.cpp - НЕПРАВИЛЬНО: дублирует реализацию
void RevolutionKalmanFilter::init(...) { ... }

// tracker.cpp - ТОЖЕ САМЫЙ КОД (дублирование!)
void RevolutionKalmanFilter::init(...) { ... }

Решение: Удалить дублирование из tracker.cpp и использовать реализацию из kalman_filter.cpp.
2. Утечка памяти в 4_radar_player.cpp

В функции draw_track_label создаются текстуры, но не всегда корректно освобождаются:
cpp

// 4_radar_player.cpp, стр. ~340
SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer_, surface);
// ... использование ...
// НЕТ SDL_DestroyTexture(texture) в некоторых путях

Решение: Использовать RAII-обертку для SDL ресурсов.
3. Неопределенное поведение в garbling_solver.cpp
cpp

// garbling_solver.cpp, стр. ~240
bool bit1 = (left1 > threshold_ && right1 <= threshold_) ? false :
            (left1 <= threshold_ && right1 > threshold_) ? true : false;
// Если оба условия ложны, bit1 = false, но логика не полная

Решение: Явно обрабатывать все случаи.
🟡 Важные улучшения
4. Отсутствие юнит-тестов

Проект имеет опцию BUILD_TESTS, но тесты не реализованы.

Предложение: Добавить тесты для:

    ConfigLoader::parse_config()

    RevolutionKalmanFilter::predict/update()

    ClusterTracker::process_scan()

    ReplyProcessor::decode_rbs_with_errors()

cpp

// Пример теста
TEST(KalmanFilter, PredictPosition) {
    RevolutionKalmanFilter filter(0.1, 1.0);
    filter.init(100, 100, 0);
    filter.update(120, 120, 10);
    auto [x, y] = filter.predict_position(5);
    EXPECT_GT(x, 100);
    EXPECT_GT(y, 100);
}

5. Использование std::endl вместо \n
cpp

// logger.cpp
file_ << formatted << std::endl;  // Вызывает flush() каждый раз

Решение: Использовать '\n' для производительности.
6. Магические числа
cpp

// cluster.cpp
if (azimuth % 128 == 0) { ... }  // Почему 128?
if (reply_count >= 2) { ... }     // Почему 2?

Решение: Вынести в конфигурацию или константы с именами.
7. Отсутствие обработки ошибок в ConfigLoader
cpp

if (r.contains("snr_db")) config.simulator.rbs.snr_db = r["snr_db"].get<double>();
// Нет проверки на отрицательные значения или другие некорректные данные

🟢 Оптимизации
8. Копирование больших объектов
cpp

// cluster.cpp
std::vector<RBSReply> get_all_rbs() const {
    std::vector<RBSReply> result;
    for (const auto& scan : scans) {
        result.insert(result.end(), scan.rbs_replies.begin(), scan.rbs_replies.end());
    }
    return result;  // Копирование при возврате
}

Решение: Использовать move-семантику или возвращать const std::vector&.
9. Линейный поиск в ClusterProcessor::group_by_range
cpp

// O(n²) сложность
for (auto& [nominal, group] : range_map) {
    if (std::abs(reply.range - nominal) <= range_tolerance_) {
        group.add_rbs(&reply);
        added = true;
        break;
    }
}

Решение: Использовать std::multimap или сортировку.
10. Избыточное логирование в TRACE

Макрос VRL_LOG_TRACE вызывается в циклах, что может замедлять работу.

Решение: Условная компиляция для TRACE уровня.
📝 Документация
11. Неполная документация API

Многие публичные методы не имеют комментариев.

Решение: Добавить Doxygen-комментарии для всех публичных интерфейсов:
cpp

/**
 * @brief Обрабатывает сканирование радара
 * @param scan Данные сканирования
 * @return Вектор обнаруженных целей
 * @note Вызывается для каждого оборота антенны
 */
std::vector<TargetReport> process_scan(const ScanReplies& scan);

🔧 Архитектурные улучшения
12. Использование интерфейсов вместо конкретных типов

Сейчас трекер жестко привязан к RevolutionKalmanFilter.

Решение: Создать интерфейс ITrackerFilter:
cpp

class ITrackerFilter {
public:
    virtual void init(double x, double y, uint32_t rev) = 0;
    virtual void predict(uint32_t delta) = 0;
    virtual void update(double x, double y, uint32_t rev) = 0;
    virtual ~ITrackerFilter() = default;
};

13. Разделение ответственности в ClusterProcessor

Класс делает слишком много: группировка, обработка RBS, обработка UVD, обработка перекрытий.

Решение: Выделить отдельные классы:

    RangeGrouper - группировка по дальности

    RBSProcessor - обработка RBS ответов

    UVDProcessor - обработка UVD ответов

📊 Дополнительные предложения
14. Метрики производительности

Добавить сбор метрик:

    Количество обработанных ответов/сек

    Время обработки сканирования

    Загрузка CPU

15. Сериализация треков

Сохранять треки в JSON для последующего анализа.
16. Поддержка реального времени

Сейчас система работает в симуляционном времени. Добавить режим реального времени с приемом данных по сети.
17. Экспорт в KML/GPX

Для визуализации треков на карте.
Итоговый список приоритетов
Приоритет	Задача	Сложность
🔴 HIGH	Удалить дублирование Kalman Filter	1 час
🔴 HIGH	Исправить утечку памяти в SDL	2 часа
🟡 MEDIUM	Добавить юнит-тесты	8 часов
🟡 MEDIUM	Устранить магические числа	2 часа
🟡 MEDIUM	Добавить Doxygen документацию	4 часа
🟢 LOW	Оптимизировать группировку по диапазонам	4 часа
🟢 LOW	Рефакторинг ClusterProcessor	8 часов
🟢 LOW	Добавить экспорт в KML	4 часа
Заключение

Проект находится в хорошем состоянии и готов к использованию. Основные проблемы — дублирование кода, потенциальные утечки памяти и отсутствие тестов. Рекомендую:

    Немедленно исправить дублирование Kalman Filter и утечки памяти

    В ближайшее время добавить юнит-тесты для критических компонентов

    Постепенно улучшить документацию и архитектуру

Проект демонстрирует высокий уровень владения C++ и понимания предметной области. Система логирования, конфигурация и визуализация выполнены на профессиональном уровне.

===============================

1. Условная компиляция для TRACE
cpp

#if VRL_ENABLE_TRACE
    #define VRL_LOG_TRACE(module, msg) ... 
#else
    #define VRL_LOG_TRACE(module, msg) ((void)0)
#endif

2. Управление через CMake

    ENABLE_TRACE_LOGGING - включает TRACE (по умолчанию OFF)

    ENABLE_DEBUG_LOGGING - включает DEBUG (по умолчанию ON)

3. Автоматическое отключение в Release

В Release сборке TRACE автоматически отключается для производительности.
4. Новые макросы

    VRL_LOG_TRACE_IF - условный TRACE с проверкой на этапе компиляции

    VRL_LOG_DEBUG_IF - условный DEBUG с проверкой на этапе компиляции

Использование
Сборка с TRACE
bash

cd build
cmake -DENABLE_TRACE_LOGGING=ON ..
make -j4

Сборка без TRACE (по умолчанию)
bash

cd build
cmake ..
make -j4

Сборка с полным отключением DEBUG
bash

cd build
cmake -DENABLE_DEBUG_LOGGING=OFF ..
make -j4

========================

Как это работает
Две перегруженные версии
cpp

// Для временных объектов (rvalue) - эффективно
std::vector<RBSReply> get_all_rbs() &&;

// Для постоянных объектов (lvalue) - безопасно
std::vector<RBSReply> get_all_rbs() const&;

Использование
cpp

// Случай 1: Временный объект (move)
TargetCluster temp;
auto replies = std::move(temp).get_all_rbs();  // Использует && версию - move

// Случай 2: Постоянный объект (копирование)
const TargetCluster& const_cluster = ...;
auto replies = const_cluster.get_all_rbs();    // Использует const& версию - копирование

// Случай 3: Не-const объект (копирование по умолчанию)
TargetCluster cluster;
auto replies = cluster.get_all_rbs();          // Использует const& версию (так как cluster - lvalue)

=============

Как использовать новый фильтр
cpp

// Создаем кастомный фильтр
class MyCustomFilter : public ITrackerFilter {
    // ... реализация ...
};

// Используем в TrackManager
TrackerConfig config;
auto filter = std::make_unique<MyCustomFilter>();
TrackManager manager(config, std::move(filter));

// Или заменяем позже
manager.set_filter(std::make_unique<AnotherFilter>());

===============


Как добавить новый фильтр в будущем
1. Создать класс, реализующий ITrackerFilter
cpp

// include/vrl/radar/processing/extended_kalman_filter.h
#pragma once

#include "i_tracker_filter.h"
#include <Eigen/Dense>

namespace vrl {
namespace radar {

class ExtendedKalmanFilter : public ITrackerFilter {
public:
    ExtendedKalmanFilter(/* параметры */);
    
    // Реализация всех виртуальных методов ITrackerFilter
    void init(double x, double y, uint32_t revolution) override;
    void predict(uint32_t delta_revolutions) override;
    void update(double x, double y, uint32_t revolution) override;
    // ... остальные методы ...
    
    std::string get_name() const override { return "ExtendedKalmanFilter"; }
    std::unique_ptr<ITrackerFilter> clone() const override;
    
private:
    // Специфичные для EKF поля
    Eigen::MatrixXd F_, H_, Q_, R_, P_;
    Eigen::VectorXd x_;
    // ...
};

} // namespace radar
} // namespace vrl

2. Использовать в TrackManager
cpp

// В коде приложения
TrackerConfig config;
auto filter = std::make_unique<ExtendedKalmanFilter>(/* параметры */);
TrackManager manager(config, std::move(filter));

// Или заменить позже
manager.set_filter(std::make_unique<ExtendedKalmanFilter>(/* параметры */));



==================

Как добавить новый алгоритм кластеризации:
cpp

// 1. Создать класс
class DBSCANClusterer : public IClusterer {
    // ... реализация всех методов ...
    std::string get_name() const override { return "DBSCANClusterer"; }
};

// 2. Использовать
auto clusterer = std::make_unique<DBSCANClusterer>(eps, min_samples);
ClusterTracker tracker(std::move(clusterer));

// Или заменить позже
tracker.set_clusterer(std::make_unique<OPTICSClusterer>(eps, min_samples));


==============

VRL-Radar Project Summary для нового чата
О проекте

VRL-Radar — система обработки радиолокационных данных на C++17. Выполняет полный конвейер: генерация синтетических ответов → кластеризация → трекинг → визуализация.
Структура проекта
text

.
├── CMakeLists.txt
├── config/
│   ├── radar.json                    # Основной файл конфигурации
│   ├── radar_common.json             # Общие параметры
│   ├── processing_config.json        # Параметры обработки
│   ├── simulation_config.json        # Параметры симуляции
│   ├── confidence_config.json        # Параметры уверенности
│   ├── targets_rbs.json              # RBS цели
│   ├── targets_uvd.json              # UVD цели
│   └── display_config.json           # Параметры отображения
├── include/vrl/radar/
│   ├── core/
│   │   ├── config.h                  # Структуры SystemConfig, GeneratedTarget
│   │   ├── config_loader.hpp         # Загрузчик JSON с поддержкой include
│   │   ├── logging_config.h          # Конфигурация логирования
│   │   ├── types.h
│   │   └── replies.h
│   ├── processing/
│   │   ├── i_tracker_filter.h        # Интерфейс для фильтров трекинга
│   │   ├── i_clusterer.h             # Интерфейс для алгоритмов кластеризации
│   │   ├── kalman_filter.h           # Фильтр Калмана (реализует ITrackerFilter)
│   │   ├── legacy_clusterer.h        # Существующий алгоритм кластеризации (реализует IClusterer)
│   │   ├── cluster.h                 # TargetCluster, ClusterTracker, ClusterProcessor
│   │   ├── range_grouper.h           # Группировка по дальности (O(n log n))
│   │   ├── rbs_processor.h           # Обработка RBS ответов
│   │   ├── uvd_processor.h           # Обработка UVD ответов
│   │   ├── garbling_solver.h         # Решение перекрытий
│   │   ├── reply_processor.h
│   │   └── tracker.h                 # TrackManager (использует ITrackerFilter)
│   ├── simulation/
│   │   └── simulator.h
│   └── utils/
│       ├── logger.h                  # Многоуровневое логирование
│       └── utils.h
├── src/
│   ├── core/
│   │   ├── config.cpp
│   │   └── config_loader.cpp
│   ├── processing/
│   │   ├── cluster.cpp
│   │   ├── garbling_solver.cpp
│   │   ├── kalman_filter.cpp
│   │   ├── legacy_clusterer.cpp
│   │   ├── range_grouper.cpp
│   │   ├── rbs_processor.cpp
│   │   ├── uvd_processor.cpp
│   │   ├── reply_processor.cpp
│   │   └── tracker.cpp
│   ├── simulation/
│   │   └── simulator.cpp
│   └── utils/
│       ├── logger.cpp
│       └── utils.cpp
├── tools/
│   ├── 1_generate_replies.cpp        # Генерация синтетических ответов
│   ├── 2_3_combined.cpp              # Кластеризация + трекинг
│   └── 4_radar_player.cpp            # Визуализация с SDL2
└── tests/
    ├── test_config_loader.cpp
    ├── test_kalman_filter.cpp
    ├── test_cluster_tracker.cpp
    ├── test_reply_processor.cpp
    ├── test_tracker_filter.cpp
    └── test_clusterer.cpp

Зависимости

    Eigen3 — линейная алгебра

    SDL2, SDL2_ttf — визуализация

    nlohmann/json — парсинг JSON (установлен через sudo apt-get install nlohmann-json3-dev)

    Google Test — юнит-тесты (опционально)

Сборка
bash

mkdir build && cd build
cmake -DBUILD_TESTS=ON -DBUILD_TOOLS=ON -DCMAKE_BUILD_TYPE=Debug ..
make -j4

Опции CMake
Опция	По умолчанию	Описание
BUILD_TESTS	OFF	Сборка юнит-тестов
BUILD_TOOLS	ON	Сборка утилит
ENABLE_VERBOSE_LOGGING	OFF	Подробное логирование
ENABLE_TRACE_LOGGING	OFF	TRACE уровень (отключается в Release)
ENABLE_DEBUG_LOGGING	ON	DEBUG уровень
Запуск
bash

cd build

# Шаг 1: Генерация ответов
./tools/1_generate_replies ../config/radar.json 300 replies.txt

# Шаг 2-3: Кластеризация + трекинг
./tools/2_3_combined ../config/radar.json replies.txt tracks_combined.txt

# Шаг 4: Визуализация
./tools/4_radar_player replies.txt plots_combined.txt tracks_combined.txt

# Запуск тестов
ctest --output-on-failure

Ключевые архитектурные решения
1. Интерфейс для фильтров трекинга (ITrackerFilter)

Позволяет легко подменять реализацию фильтра (Kalman, EKF, UKF, Particle).
cpp

class ITrackerFilter {
    virtual void init(double x, double y, uint32_t rev) = 0;
    virtual void predict(uint32_t delta) = 0;
    virtual void update(double x, double y, uint32_t rev) = 0;
    virtual std::unique_ptr<ITrackerFilter> clone() const = 0;
    virtual std::string get_name() const = 0;
    // ...
};

2. Интерфейс для алгоритмов кластеризации (IClusterer)

Позволяет легко подменять алгоритм (Legacy, DBSCAN, OPTICS, иерархический).
cpp

class IClusterer {
    virtual void process_scan(const ScanReplies& scan) = 0;
    virtual std::vector<TargetCluster> get_completed_clusters() = 0;
    virtual std::unique_ptr<IClusterer> clone() const = 0;
    virtual std::string get_name() const = 0;
    // ...
};

3. Разделение ClusterProcessor

    RangeGrouper — группировка по дальности (O(n log n))

    RBSProcessor — обработка RBS ответов

    UVDProcessor — обработка UVD ответов

4. Модульная конфигурация

JSON файлы с поддержкой _includes директив для композиции.
5. Многоуровневое логирование

Индивидуальные уровни для каждого модуля, условная компиляция для TRACE.
Статус проекта
Компонент	Статус
Сборка	✅
Юнит-тесты	✅ (6 тестов, 100% passed)
Утечки памяти	✅ (проверено Valgrind)
Конфигурация	✅ (модульная, с валидацией)
Логирование	✅ (многоуровневое)
Интерфейс фильтров	✅ (готов к расширению)
Интерфейс кластеризации	✅ (готов к расширению)
TODO / Возможные улучшения

    Новые алгоритмы кластеризации — DBSCAN, OPTICS, иерархическая

    Новые фильтры трекинга — EKF, UKF, Particle Filter

    Экспорт в KML/GPX — для визуализации на карте

    Поддержка реального времени — прием данных по сети

    Метрики производительности — сбор статистики

Файлы для передачи в новый чат

Если нужно продолжить работу в новом чате, вот список ключевых файлов, которые стоит показать:

    CMakeLists.txt (корневой и src/tests/tools)

    include/vrl/radar/processing/i_tracker_filter.h

    include/vrl/radar/processing/i_clusterer.h

    include/vrl/radar/processing/tracker.h

    include/vrl/radar/processing/cluster.h

    src/processing/tracker.cpp

    src/processing/cluster.cpp

    config/radar.json и связанные конфиги

    include/vrl/radar/utils/logger.h

Проект полностью собран и готов к использованию! 🚀


===============


Слабые стороны и проблемы
1. Потенциальные проблемы производительности
cpp

// В cluster.cpp - копирование вместо move
std::vector<RBSReply> TargetCluster::get_all_rbs() const& {
    std::vector<RBSReply> result;
    // ... копирование
    result.insert(result.end(), scan.rbs_replies.begin(), scan.rbs_replies.end());
    return result;
}

2. Отсутствие многопоточности

    Вся обработка последовательная

    Не используется современный C++ параллелизм

    Нет пула потоков для обработки сканов

3. Недостаточная оптимизация
cpp

// В range_grouper.cpp - O(n²) в некоторых местах
for (auto& cluster : active_clusters_) {
    for (const auto& reply : scan.rbs_replies) {
        if (reply.range >= cluster.min_range - range_window_ && ...) {

4. Проблемы с памятью

    Нет ограничения на размер истории треков

    Возможна утечка через sources в TargetReport

    Большие массивы копируются без необходимости

5. Отсутствие явной обработки исключений
cpp

// В config_loader.cpp - просто ловим и логируем
catch (const std::exception& e) {
    VRL_LOG_WARN(modules::CONFIG, "Failed to get value: " + std::string(e.what()));
    return false;
}

6. Нет асинхронного ввода/вывода

    Файлы читаются синхронно

    Нет потоковой обработки больших файлов

План модернизации
Фаза 1: Критические улучшения (1-2 недели)
1.1 Оптимизация памяти и производительности

Добавить пул объектов для Reply:
cpp

// include/vrl/radar/core/object_pool.hpp
template<typename T>
class ObjectPool {
    std::vector<std::unique_ptr<T>> pool_;
    std::queue<T*> available_;
public:
    T* acquire();
    void release(T* obj);
};

// Использование в Cluster
class ReplyPool {
    ObjectPool<RBSReply> rbs_pool_;
    ObjectPool<UVDReply> uvd_pool_;
};

Использовать move-семантику везде:
cpp

// В cluster.h - добавить методы с && для всех больших объектов
std::vector<RBSReply> get_all_rbs() &&;  // Уже есть
std::vector<UVDReply> get_all_uvd() &&;  // Уже есть

// Использовать в коде
auto rbs = std::move(cluster).get_all_rbs();

Добавить резервирование памяти:
cpp

void Cluster::add_reply(const Reply& r) {
    if (replies.capacity() == 0) {
        replies.reserve(32);  // Предварительное резервирование
    }
    // ...
}

1.2 Исправление ошибок

Исправить утечку в TargetReport:
cpp

struct TargetReport {
    // Вместо vector<const void*> sources
    std::vector<std::variant<const RBSReply*, const UVDReply*>> sources;
    // Или использовать weak_ptr
};

Добавить ограничение на историю:
cpp

struct Track {
    static constexpr size_t MAX_HISTORY = 20;
    std::vector<TargetReport> history;
    
    void add_history(const TargetReport& report) {
        if (history.size() >= MAX_HISTORY) {
            history.erase(history.begin());
        }
        history.push_back(report);
    }
};

1.3 Улучшить обработку ошибок

Добавить Result<T> тип:
cpp

// include/vrl/radar/core/result.hpp
template<typename T, typename E = std::string>
class Result {
    std::variant<T, E> value_;
public:
    bool is_ok() const;
    T& value();
    E& error();
};

// Использование
Result<SystemConfig> load_config(const std::string& filename);

Фаза 2: Производительность (2-3 недели)
2.1 Параллельная обработка

Добавить параллельную кластеризацию:
cpp

// include/vrl/radar/processing/parallel_clusterer.h
class ParallelClusterer : public IClusterer {
    std::unique_ptr<IClusterer> clusterer_;
    std::unique_ptr<ThreadPool> pool_;
    std::queue<ScanReplies> queue_;
    
public:
    void process_scan(const ScanReplies& scan) override {
        // Асинхронная обработка
        pool_->enqueue([this, scan]() {
            clusterer_->process_scan(scan);
        });
    }
};

Добавить std::execution для алгоритмов:
cpp

// В range_grouper.cpp
#include <execution>

std::sort(std::execution::par, rbs_replies.begin(), rbs_replies.end(),
    [](const ReplyEntry& a, const ReplyEntry& b) {
        return a.range < b.range;
    });

2.2 Оптимизация поиска

Заменить линейный поиск на пространственные индексы:
cpp

// include/vrl/radar/processing/spatial_index.hpp
class SpatialIndex {
    std::map<uint16_t, std::vector<Reply*>> azimuth_index_;
    std::multimap<uint16_t, Reply*> range_index_;
    
public:
    void add(Reply* reply);
    std::vector<Reply*> find_near(uint16_t azimuth, uint16_t range, int tolerance);
};

2.3 Асинхронный ввод/вывод
cpp

// Использовать boost::asio или std::async для файлового ввода
class AsyncFileReader {
    std::future<std::string> read_async(const std::string& filename);
};

Фаза 3: Новые возможности (3-4 недели)
3.1 Дополнительные фильтры

Добавить EKF и UKF:
cpp

// include/vrl/radar/processing/extended_kalman_filter.h
class ExtendedKalmanFilter : public ITrackerFilter {
    // Нелинейная модель движения
};

class UnscentedKalmanFilter : public ITrackerFilter {
    // Без производных
};

3.2 Интеллектуальная кластеризация

Добавить DBSCAN:
cpp

// include/vrl/radar/processing/dbscan_clusterer.h
class DBSCANClusterer : public IClusterer {
    double eps_;  // Радиус окрестности
    int min_pts_; // Минимальное количество точек
};

3.3 Улучшенный трекинг

Добавить GNN (Global Nearest Neighbor):
cpp

class GNNTracker {
    // Оптимальное сопоставление треков и измерений
    std::vector<std::pair<Track, TargetReport>> solve_assignment(
        const std::vector<Track>& tracks,
        const std::vector<TargetReport>& targets);
};

3.4 Поддержка реального времени
cpp

// include/vrl/radar/processing/realtime_processor.h
class RealtimeProcessor {
    std::chrono::microseconds target_latency_;
    // Приоритетные очереди
    // Динамическое управление нагрузкой
};

Фаза 4: Инфраструктура (2-3 недели)
4.1 Мониторинг и метрики
cpp

// include/vrl/radar/utils/metrics.h
class Metrics {
    std::atomic<size_t> processed_scans_;
    std::atomic<size_t> dropped_scans_;
    std::chrono::steady_clock::duration processing_time_;
    
public:
    void record_scan(double latency_ms);
    void record_drop();
    json to_json() const;
};

4.2 Конфигурация как код
cpp

// Использовать std::variant для типизированной конфигурации
using ConfigValue = std::variant<int, double, bool, std::string, std::vector<...>>;

class TypedConfig {
    std::map<std::string, ConfigValue> values_;
public:
    template<typename T>
    T get(const std::string& key) const;
};

4.3 Бенчмарки
cpp

// tests/benchmark/
// Добавить Google Benchmark
#include <benchmark/benchmark.h>

static void BM_ClusterProcessing(benchmark::State& state) {
    // Тесты производительности
}

4.4 Документация API
cpp

// Добавить Doxygen комментарии к публичным интерфейсам
/**
 * @brief Интерфейс для фильтров трекинга
 * @see KalmanFilter, ExtendedKalmanFilter
 */
class ITrackerFilter {
    // ...
};

Фаза 5: Расширяемость (1-2 недели)
5.1 Плагины
cpp

// include/vrl/radar/plugin/plugin_manager.hpp
class PluginManager {
    void load_plugin(const std::string& path);
    std::vector<IClusterer*> get_clusterers();
    std::vector<ITrackerFilter*> get_filters();
};

// Динамическая загрузка .so/.dll

5.2 Python биндинги
cpp

// Использовать pybind11
#include <pybind11/pybind11.h>

PYBIND11_MODULE(vrl_radar, m) {
    py::class_<TrackManager>(m, "TrackManager")
        .def("process_targets", &TrackManager::process_targets)
        .def("get_active_tracks", &TrackManager::get_active_tracks);
}

5.3 REST API
cpp

// Использовать cpprestsdk или oatpp
class RadarAPI {
    void start_server(int port);
    void handle_get_tracks(http_request request);
    void handle_get_config(http_request request);
};

Приоритетный список исправлений
🔴 Критические (сделать сейчас)

    Исправить утечку памяти в TargetReport::sources

    Добавить ограничения на историю треков

    Исправить накопление объектов в кластерах

    Улучшить обработку исключений в config_loader

🟡 Важные (в ближайшее время)

    Оптимизировать range_grouper (O(n²) → O(n log n))

    Добавить параллельную обработку сканов

    Внедрить пул объектов для Reply

    Улучшить валидацию конфигурации

🟢 Желательные (в перспективе)

    Добавить EKF/UKF фильтры

    Реализовать DBSCAN кластеризацию

    Добавить Python биндинги

    Внедрить мониторинг и метрики

    =======================