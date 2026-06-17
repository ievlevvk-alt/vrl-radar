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
./3_track_processing plots.txt tracks.txt

# 4. Визуализация с разверткой (5 секунд на оборот)
./4_radar_player replies.txt plots.txt tracks.txt
