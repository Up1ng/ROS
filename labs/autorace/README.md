# AutoRace. Удержание в полосе на коротком участке

Первый этап командного проекта из письма преподавателя. Контроллер работает с учебной дифференциальной моделью. Исходная трасса, модель и мост взяты из ветки `competition` репозитория [virusapex/my_robot](https://github.com/virusapex/my_robot/tree/32a6aae4c5fe500725a14b8a30d532355950eab3). Для воспроизводимости используется коммит `32a6aae4c5fe500725a14b8a30d532355950eab3`.

- `robot_bringup/worlds/course.sdf` задаёт дорожную сетку, перекрёсток, тоннель, конусы и движущуюся машину.
- `robot_description/urdf` задаёт дифференциальную модель. В Xacro RGB-камере заданы наклон вниз 0,2 рад, разрешение 848 × 480 и частота 30 Гц по времени симуляции.
- `robot_bringup/config/robot_bridge.yaml` передаёт `/color/image` из Gazebo в ROS 2, `/cmd_vel` типа `Twist` — из ROS 2 в Gazebo, а `/clock` — в ROS 2.

## Собрать и запустить

В рабочую область `~/autorace_ws/src` поместите `my_robot` на указанном коммите и пакет `src/lane_controller`. Для другой поддерживаемой среды устанавливайте зависимости под её активный `$ROS_DISTRO`.

Для запуска в Docker выполните из корня командного репозитория. Здесь `../my_robot-upstream` — соседний каталог с исходной трассой.

```bash
docker build -t autorace-jazzy labs/autorace
docker run --rm -it --name autorace -e ROS_DOMAIN_ID=82 \
  -v "$PWD:/team:ro" \
  -v "$(cd ../my_robot-upstream && pwd):/upstream:ro" \
  autorace-jazzy bash
```

В контейнере соберите workspace.

```bash
source /opt/ros/jazzy/setup.bash
cd ~/autorace_ws
mkdir -p src
ln -s /upstream src/my_robot
ln -s /team/src/lane_controller src/lane_controller
colcon build --symlink-install --packages-select robot_description robot_bringup lane_controller \
  --cmake-args -DBUILD_TESTING=OFF
source ~/autorace_ws/install/setup.bash
```

В каждой новой оболочке подключайте ROS и `~/autorace_ws/install/setup.bash`. С X11 или Xvfb можно открыть исходную трассу с RViz в одном терминале и после появления камеры включить контроллер во втором. Для проверки в контейнере без окна используйте сценарий заезда ниже.

```bash
ros2 launch robot_bringup autorace_2025.launch.py
```

```bash
ros2 launch lane_controller lane.launch.py
```

`lane.launch.py` включает `use_sim_time`. Параметры по умолчанию лежат в `src/lane_controller/config/lane.yaml`. Другой файл можно передать как `config:=/полный/путь/к/lane.yaml`. Для сравнения меняется по одному коэффициенту и повторяется старт с той же позиции.

```bash
ros2 topic list
ros2 topic info /color/image -v
ros2 topic hz /color/image
ros2 topic info /cmd_vel -v
```

В RViz открыть Image для `/color/image` или `/lane_controller/debug/image`. Второй топик показывает исходный кадр, разметку с найденными точками жёлтой и белой границы, обе HSV-маски, центр полосы (зелёный круг) и целевую точку (синий крест). `/lane_controller/telemetry` содержит ошибку по горизонтали (`vector.x`), угловую (`vector.y`) и линейную скорость (`vector.z`). При остановке ошибка обозначается `NaN`.

## Как работает контроллер

1. Детектор оставляет ROI на дороге и строит HSV-маски. Он проецирует в кадр дуги на плоскости дороги и ищет на них жёлтую и белую границы. На повороте эти границы могут быть на разных высотах в кадре. Детектор проверяет порядок границ, ширину полосы и непрерывность последовательности пар, затем оценивает центр на заданном расстоянии впереди.
2. Ошибка равна «целевой столбец минус найденный центр», делённому на половину ширины изображения. Когда центр слева, ошибка положительная, `angular.z` положительная и робот доворачивает влево.
3. Сначала используется P с постоянной скоростью. D оценивает скорость изменения ошибки и фильтрует её. I суммирует ошибку по фактическому интервалу между метками кадров. При насыщении интегратор разрешает только изменение, которое уменьшает насыщение. Значения I и `angular.z` ограничены.
4. При отсутствии надёжной пары, устаревшей метке или паузе камеры публикуется полностью нулевой Twist и сбрасывается состояние PID. Фоновый таймер использует монотонное время для остановки даже при паузе `/clock`.

Контроллер рассчитан на выбранный участок с видимой разметкой. Перекрёстки, объезд препятствий и перенос на физического робота требуют отдельных проверок.

## Воспроизвести измерения

Сценарии запускаются из корня командного репозитория (`cd /team` в контейнере) с подключённым `~/autorace_ws/install/setup.bash`. Используйте свободный `ROS_DOMAIN_ID`. `run_trial.py` запускает Gazebo заново для каждого опыта, добавляет камеру наблюдения сверху и сохраняет кадры, команды и одометрию. Для `--rviz` нужен X11 или Xvfb.

```bash
python3 tools/autorace/check_stop.py
python3 tools/autorace/run_trial.py /tmp/autorace-P --seconds 16 --video --inspect
python3 tools/autorace/run_trial.py /tmp/autorace-PD --seconds 16 --kd 0.08
python3 tools/autorace/run_trial.py /tmp/autorace-PI --seconds 16 --ki 0.10
python3 tools/autorace/run_trial.py /tmp/autorace-offset --y -2.24 --seconds 4 --camera-loss
python3 tools/autorace/plot_trials.py /tmp/autorace-comparison.png \
  /tmp/autorace-P /tmp/autorace-PD /tmp/autorace-PI
```

Файлы заездов, условия экспериментов и выводы собраны в [отчёте](../../evidence/autorace/README.md).
