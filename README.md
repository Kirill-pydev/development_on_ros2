# ROS 2 — хакатон: лидара, объезд препятствий, YOLO

В репозитории два представления одних и тех же трёх нод:

- **Исходники как раньше:** `moving.txt`, `new_mover.txt`, `lidar_colibrovka.txt` (логику этих файлов не меняли).
- **Формат пакета ROS 2:** каталог `hackathon_robot/` — те же скрипты как модули `.py`, разложенные по **ролям команды** (как в учебнике, занятие 19), плюс нода для датасета под дообучение YOLO.

---

## Роли и каталоги (по учебнику, § 5.2)

В методичке за каждым направлением закреплены типичные файлы. В этом репозитории каталоги Python-пакета сопоставлены так:

| Роль на хакатоне | Зона ответственности в учебнике | Каталог в `hackathon_robot/` | Что лежит сейчас |
|------------------|----------------------------------|------------------------------|------------------|
| **Архитектор FSM** | `robot_node.py`: диспетчер, переходы состояний | `hackathon_robot/fsm_architect/` | `robot_fsm_node.py` — приоритеты батарея / приветствие / исследование, Nav2, `robot/command` |
| **CV-инженер** | детекция (в учебнике — `person_detector_node`, HOG), топики `/person_detected`, offset | `hackathon_robot/cv_engineer/` | `person_detector_node.py` (HOG), `yolo_finetune_node.py` |
| **Nav-инженер** | exploration, цели Nav2, обработка результата | `hackathon_robot/nav_engineer/` | `autonomous_drive_forward.py`, `contour_avoidance.py` — низкоуровневый объезд по `/scan` + `/cmd_vel` (параллель учебниковому Nav2; один стек выбирает команда) |
| **Платформа / сенсоры** *(дополнение к карте ролей)* | в учебнике косвенно (карта, лидара) | `hackathon_robot/sensing/` | `lidar_calibrator.py` — калибровка лидара и публикация TF |
| **Интегратор** | `launch/`, `config/params.yaml`, совместные прогоны, демо | `launch/`, `config/` | `voice_stack.launch.py`, `bringup_voice_fsm.launch.py`, `config/yolo_data.example.yaml` |

Имена исполняемых ROS-команд (`ros2 run hackathon_robot …`) **не менялись** — меняется только путь к модулю внутри пакета.

---

## Сборка и зависимости

### Зависимости ROS 2

Пакет `hackathon_robot` (`ament_python`) ожидает стандартные сообщения, **`nav2_msgs`**, **`tf2_ros`**, **`cv_bridge`**. Для FSM с навигацией на роботе должен быть запущен Nav2 (action `navigate_to_pose`).

### Зависимости Python (частично через `setup.py`)

В `setup.py`: `PyYAML`, **`numpy`**. Дополнительно:

```bash
pip install opencv-python-headless
pip install vosk sounddevice
```

Опционально после локальной установки пакета:

```bash
pip install ".[voice]"
```

**Синтез офлайн:** [espeak-ng](https://github.com/espeak-ng/espeak-ng) (Linux: `sudo apt install espeak-ng`).

**Модель Vosk:** [alphacephei.com/vosk/models](https://alphacephei.com/vosk/models) — распакуйте и передайте путь в `vosk_model_path` при запуске launch.

Обучение YOLO офлайн (вне ROS или в отдельном терминале):

```bash
pip install ultralytics
```

### Сборка `colcon`

Разместите пакет в `src` вашего воркспейса, например:

```text
ros2_ws/src/hackathon_robot/   ← содержимое из этого репозитория
```

Сборка и окружение (Linux; под Windows ROS 2 — по документации вашего дистрибутива):

```bash
cd ros2_ws
colcon build --packages-select hackathon_robot
source install/setup.bash
```

---

## Ноды и исполняемые имена

После сборки доступны команды:

| Команда ROS 2 | Модуль | Назначение |
|---------------|--------|------------|
| `ros2 run hackathon_robot lidar_calibrator` | `sensing/lidar_calibrator.py` | Калибровка лидара, TF, топики препятствия впереди |
| `ros2 run hackathon_robot autonomous_drive_forward` | `nav_engineer/autonomous_drive_forward.py` | Движение вперёд с объездом препятствий (конечный автомат) |
| `ros2 run hackathon_robot contour_avoidance` | `nav_engineer/contour_avoidance.py` | Альтернативный сценарий объезда контура с одометрией |
| `ros2 run hackathon_robot yolo_finetune_node` | `cv_engineer/yolo_finetune_node.py` | Сбор кадров с камеры для датасета и дообучения YOLO |
| `ros2 run hackathon_robot person_detector` | `cv_engineer/person_detector_node.py` | HOG: `/person_detected`, `/person_offset_x` |
| `ros2 run hackathon_robot robot_fsm` | `fsm_architect/robot_fsm_node.py` | FSM + Nav2 + голосовые команды (`robot/command`) |
| `ros2 run hackathon_robot tts_espeak` | `voice/tts_espeak_node.py` | Очередь фраз → espeak-ng, статус `voice/tts_state` |
| `ros2 run hackathon_robot asr_vosk` | `voice/asr_vosk_node.py` | Vosk → `voice/recognized_text` |
| `ros2 run hackathon_robot voice_command_processor` | `voice/command_processor_node.py` | Парсер → `robot/command` + ответ в TTS |

**Launch:**

```bash
ros2 launch hackathon_robot voice_stack.launch.py vosk_model_path:=/ABS/PATH/vosk-model-small-ru-0.22
ros2 launch hackathon_robot bringup_voice_fsm.launch.py vosk_model_path:=/ABS/PATH/vosk-model-small-ru-0.22 camera_topic:=/camera_node/image_raw
```

**Важно:** не совмещайте **`robot_fsm`** (Nav2 и `/cmd_vel`) с **`autonomous_drive_forward`** / **`contour_avoidance`**, если все публикуют в один `/cmd_vel`. Обе ноды объезда по-прежнему объявляют имя `autonomous_drive` — их не запускайте одновременно друг с другом.

---

## Голос (офлайн), поток как в учебнике

Цепочка: **микрофон → Vosk (`voice/recognized_text`) → парсер (`robot/command`) → исполнение в `robot_fsm` + синтез ответа (`voice/text_to_speak` → espeak-ng)**. Топики совместимы с описанием в файле «Распознавание и синтез речи», но ASR/TTS без SpeechKit.

Имитация команды:

```bash
ros2 topic pub --once /voice_cmd std_msgs/msg/String "data: 'робот найди синий куб'"
```

---

## Калибровка лидара (`lidar_calibrator`)

- Подписывается на скан (параметр `scan_topic`, по умолчанию `/scan`).
- При отсутствии сохранённой калибровки просит выполнить процедуру (ввод с консоли **Enter** — удобно в интерактивном терминале).
- Сохраняет **`params/lidar_calibration.yaml`** относительно **текущей рабочей директории** процесса (откуда запущен `ros2 run`). Перед запуском можно перейти в нужный каталог или скопировать файл `params/` в то место, откуда стартуете драйверы и ноды движения.
- Публикует статический TF `base_frame` → `lidar_frame` (имена задаются параметрами).
- Топики: `/obstacle_detected` (`Bool`), `/front_distance` (`Float32`).

Пример запуска:

```bash
cd /path/to/workspace_where_params_should_be_created
ros2 run hackathon_robot lidar_calibrator
```

---

## Объезд препятствий

### `autonomous_drive_forward`

- Читает калибровку из `params/lidar_calibration.yaml` (тот же принцип рабочего каталога).
- Без успешной калибровки робот не едет (остаётся в стопе).
- Публикует `/cmd_vel`, состояние `/robot_state`, пройденный путь `/distance_traveled`.
- Параметры скорости и дистанций задаются через `declare_parameter` в коде (можно переопределять из CLI/launch).

Пример с переопределением параметров:

```bash
ros2 run hackathon_robot autonomous_drive_forward --ros-args -p linear_speed:=0.15 -p obstacle_distance:=0.35
```

### `contour_avoidance`

- Также использует `params/lidar_calibration.yaml`.
- Другая логика состояний (обход с накоплением угла по одометрии, финальный прямолинейный участок и завершение).
- Имя ноды в коде тоже `autonomous_drive`; топики те же по смыслу (`/cmd_vel`, `/robot_state`).

Запуск только одной из двух нод объезда на роботе.

---

## Дообучение YOLO (`yolo_finetune_node`)

Нода **не размечает** изображения и **не запускает обучение внутри ROS** по умолчанию: она сохраняет кадры с камеры, чтобы вы могли разметить классы, которых нет в базовой модели, и дообучить YOLO офлайн.

### Запуск

```bash
ros2 run hackathon_robot yolo_finetune_node --ros-args \
  -p image_topic:=/image_raw \
  -p save_directory:=~/yolo_dataset \
  -p auto_capture_period_sec:=0.5
```

Параметры:

- `image_topic` — `sensor_msgs/Image`.
- `save_directory` — корень датасета.
- `images_subdir` — подпапка для файлов (по умолчанию `images/train`).
- `jpeg_quality` — качество JPEG.
- `auto_capture_period_sec` — если `> 0`, периодически сохранять кадры (секунды); если `0`, только по сервису.

### Сервисы

- **`~/capture_image`** (`std_srvs/srv/Trigger`) — сохранить один текущий кадр; в `message` будет путь к файлу или текст ошибки.
- **`~/dataset_paths`** (`Trigger`) — вернуть строку с путём к каталогу изображений.

Пример (после `source install/setup.bash`):

```bash
ros2 service call /yolo_finetune_node/capture_image std_srvs/srv/Trigger
```

### Дальнейшие шаги (офлайн)

1. Разметка в формате **YOLO** (каждый `.jpg` → `labels/train/*.txt` с строками `class x_center y_center width height`, координаты 0…1).
2. Подготовьте **`images/train`**, **`images/val`**, **`labels/train`**, **`labels/val`**.
3. Скопируйте пример **`hackathon_robot/config/yolo_data.example.yaml`** в корень датасета как `data.yaml` и укажите абсолютный `path`, число классов `nc` и `names`.
4. Установите Ultralytics и обучите модель, например:

```bash
yolo detect train data=/ABS/PATH/yolo_dataset/data.yaml model=yolov8n.pt epochs=80 imgsz=640
```

или через Python:

```python
from ultralytics import YOLO

model = YOLO("yolov8n.pt")
model.train(data="/ABS/PATH/yolo_dataset/data.yaml", epochs=80, imgsz=640)
```

5. Полученные веса (`runs/detect/train/weights/best.pt`) подключите в свою ноду/скрипт инференса (например, подписка на ту же камеру и публикация детекций в пользовательский топик).

Подробные комментарии к параметрам и сервисам — в модуле `hackathon_robot/cv_engineer/yolo_finetune_node.py`.

---

## Структура пакета

```text
hackathon_robot/
  package.xml
  setup.py
  setup.cfg
  resource/hackathon_robot
  config/                         ← зона интегратора (пример data.yaml)
    yolo_data.example.yaml
  launch/
    voice_stack.launch.py
    bringup_voice_fsm.launch.py
  hackathon_robot/
    __init__.py
    voice/
      command_parser.py
      asr_vosk_node.py
      tts_espeak_node.py
      command_processor_node.py
    fsm_architect/
      __init__.py
      robot_fsm_node.py
    cv_engineer/
      __init__.py
      person_detector_node.py
      yolo_finetune_node.py
    nav_engineer/                 ← Nav-инженер (+ лидара-объезд как альтернатива Nav2)
      __init__.py
      autonomous_drive_forward.py # было moving.txt
      contour_avoidance.py       # было new_mover.txt
    sensing/                      ← платформа/сенсоры (доп. к учебнику)
      __init__.py
      lidar_calibrator.py        # было lidar_colibrovka.txt
```

Корень репозитория по-прежнему может содержать `.txt` копии для справки.

---

## Краткий чеклист на день хакатона

1. Запустить драйвер лидара и при необходимости камеры.
2. Выполнить **`lidar_calibrator`**, убедиться, что создан **`params/lidar_calibration.yaml`**.
3. Запустить **ровно одну** ноду объезда из двух.
4. Для новых объектов в vision: **`yolo_finetune_node`** → разметка → **`data.yaml`** → **`yolo train`** → интеграция `best.pt`.

Удачи на хакатоне.
