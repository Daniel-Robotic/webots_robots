# Webots Robots — KUKA LBR iiwa 7 R800

Проект моделирует рабочее пространсво с роботом **KUKA LBR iiwa 7 R800**, двупальцевым захватом и RGB/Depth‑камерой. Репозиторий включает:
- модели и прототипы Webots,
- библиотеку для кинематики, обмена сообщениями и работы с камерами (`extensions/`),
- контроллеры робота, камеры и набор супервизоров для типовых сценариев (`controllers/`),
- утилиту генерации калибровочных шаблонов (`generate_calibration_pattern.py`).

Поддерживаются калибровочные паттерны **ChArUco**, **ArUco**, **Chessboard**, **CircleGrid** (симметричный/асимметричный).

---

## Содержание
- [Возможности](#возможности)
- [Структура проекта](#структура-проекта)
- [Установка](#установка)
- [Настройка VS Code и Webots](#настройка-vs-code-и-webots)
- [Быстрый старт](#быстрый-старт)
- [Архитектура и обмен сообщениями](#архитектура-и-обмен-сообщениями)
- [Контроллеры и супервизоры](#контроллеры-и-супервизоры)
- [Управление камерой и сохранение данных](#управление-камерой-и-сохранение-данных)
- [Генерация калибровочного паттерна](#генерация-калибровочного-паттерна)
- [Форматы файлов (JSON/CSV)](#форматы-файлов-jsoncsv)
- [Пути и переменные окружения](#пути-и-переменные-окружения)
- [Советы и устранение неполадок](#советы-и-устранение-неполадок)

---

## Возможности
- Управление **iiwa7** в начальном мире и в мире **калибровки Eye‑in‑hand**.
- Движение по **декартовой системе координат** (IK + планировщик траектории) и по **CSV‑траектории** в суставах.
- **Следование за гизмой** (целевым объектом) из UI Webots (перетаскиваете `TARGET_GRIPPER` — робот следует).
- Управление **захватом** (открыть/закрыть) из сценариев или через гизму.
- Сохранение изображений **RGB/Depth/RGBD** по команде.
- Онлайн‑детекция калибровочных шаблонов и наложение результата на изображение.
- Генерация шаблонов и **автообновление** размеров/текстуры в `CalibrationPattern.proto`.

---

## Структура проекта
Основные узлы:
```
.
├── controllers/                # контроллеры робота/камеры и супервизоры
│   ├── camera_controller/      # камера: сенсоры, распознавание, сохранение, паблишер
│   ├── iiwa_controller/        # низкоуровневое управление суставами и хватом
│   ├── supervisor_*            # супервизоры сценариев (см. ниже)
├── extensions/                 # библиотека проекта
│   ├── board_generation/       # генераторы: Aruco/Charuco/Chess/Circlegrid
│   ├── core/                   # команды, коммуникации, трекинг движения
│   ├── kinematics/             # модель iiwa, IK/PK, планировщик траекторий
│   ├── webots/                 # обёртки под Emitter/Receiver, камера и цель
│   └── utils/                  # утилиты (поиск устройств, прогресс‑бар и т.д.)
├── protos/                     # PROTO, меши, текстуры и иконки
├── worlds/
│   ├── coloborative_world.wbt          # базовый мир с iiwa и столом
│   └── eye-in-hand calibration.wbt      # мир для калибровки камеры (eye‑in‑hand)
├── generate_calibration_pattern.py      # CLI‑утилита генерации паттерна
├── requirements.txt / pyproject.toml    # зависимости
└── webots.yaml / README.md / uv.lock
```
> Папка `worlds` содержит два мира: базовый с коллаборативным роботом и мир для калибровки камеры.

---

## Установка
### Требования
- **Webots** с поддержкой Python‑контроллеров.
- **Python 3.12+**.

### Клонирование репозитория
```bash
git clone https://github.com/Daniel-Robotic/webots_robots.git
```

### Установка зависимостей
**pip**
```bash
python -m venv .venv
source .venv/bin/activate   # Windows: .venv\Scripts\activate
pip install -r requirements.txt
```

**uv**
```bash
uv pip install -r requirements.txt
```

> Примечание: часть контроллеров использует `${HOME}` и относительные пути — при необходимости скорректируйте их в начале файлов.

---

## Настройка VS Code и Webots
### VS Code
Создайте `.vscode/settings.json` в корне проекта:
```json
{
  "python.analysis.extraPaths": [
    "<webots_path>",
    "${workspace}/extensions"
  ],
  "python.autoComplete.extraPaths": [
    "<webots_path>",
    "${workspace}/extensions"
  ]
}
```
Замените `<webots_path>` на путь к каталогу **Python API Webots**. Например:
`/usr/local/webots/lib/controller/python/`

### Webots → Tools → Preferences
В поле **Python command** укажите путь до интерпретатора вашей виртуальной среды (`.venv/bin/python` или `.venv\Scripts\python.exe`).

---

## Быстрый старт
1. Откройте Webots и загрузите мир:
   - `worlds/coloborative_world.wbt` — базовая сцена.
   - `worlds/eye-in-hand calibration.wbt` — сцена калибровки eye‑in‑hand.
2. Убедитесь, что у узлов **робота** и **камеры** выставлены нужные контроллеры (по умолчанию уже привязаны).
3. Нажмите ▶️ **Run**. Для сценариев с внешними файлами (CSV/JSON/папки) проверьте пути в начале соответствующих супервизоров.
---

## Архитектура и обмен сообщениями
Коммуникация между супервизорами, контроллером робота и камеры — через `Emitter/Receiver` (JSON). Обёртка: `extensions.webots.communication.WebotsJsonComm`.

**Шаблон сообщения**
```json
{
  "source": "<узел‑источник>",
  "type": "<тип-сообщения>",
  "data": { ... }
}
```

**Основные типы**
- `robot_position` (supervisor → robot): задать целевые углы и состояние хвата.
- `LBRiiwa7R800_current_pose` (robot → supervisor): текущие углы и состояние хвата.
- `save_image` (supervisor → camera): сохранить кадр(ы) камеры.
- `pattern_detection` (supervisor → camera): выполнить детекцию паттерна (параметры ниже).

---

## Контроллеры и супервизоры

### camera_controller/camera_controller.py
**Функция**: управление сенсорами (RGB + RangeFinder), распознавание паттернов, наложение оверлея, сохранение кадров, публикация.

**Команды**
- `save_image`:
  ```json
  {
    "source": "supervisor",
    "type": "save_image",
    "data": { "folder": "./images", "type": "rgb|depth|rgbd" }
  }
  ```
- `pattern_detection` (пример для ChArUco):
  ```json
  {
    "source": "supervisor",
    "type": "pattern_detection",
    "data": {
      "pattern_type": "charuco",
      "params": {
        "grid_cells": [5, 7],
        "cell_size_mm": 55,
        "marker_length_mm": 40,
        "aruco_dict_name": "4x4_50"
      }
    }
  }
  ```
Поддержка: `charuco`, `aruco`, `chessboard`, `circlegrid` (`asymmetric` для circlegrid).

---

### iiwa_controller/iiwa_controller.py
**Функция**: низкоуровневое управление всеми суставами и моторами хвата; чтение датчиков положений.

**Вход**: `robot_position` с полями
```json
{
  "joints": { "lbr_A1": ..., "lbr_A2": ..., "...": ... },
  "gripper": true/false
}
```
**Выход**: `LBRiiwa7R800_current_pose`
```json
{
  "joints": { "lbr_A1": <rad>, ..., "lbr_A7": <rad> },
  "gripper": { ... }   // по моторам хвата
}
```

> Семантика хвата: булево значение `gripper` трактуется как «открыть/закрыть» внутри контроллера с учётом левого/правого пальца; сценарные супервизоры ниже задают смысл `true/false` и могут отличаться (см. примечания).

---

### supervisor_camera_saver/supervisor_camera_saver.py
**Назначение**: периодическое сохранение изображений из камеры.

- Редактируйте `IMAGE_FOLDER` в начале файла.
- Каждые ~100 тиков отправляет `save_image` с `type: "rgbd"` (по умолчанию).
- Переводит робота к целевой позе `qr` при старте.

**Типичный запуск**: просто запустите мир с этим супервизором; изображения появятся в `IMAGE_FOLDER`.

---

### supervisor_cartesian_move/supervisor_cartesian_move.py
**Назначение**: движение по **декартовым** командам из JSON через IK и планировщик (`TrajectoryPlannerComponent`).

- Путь к сценарию: `JSON_CARTESIAN_PATH`.
- Планировщик: параметры `speed_scale` (напр., `0.5`) и `dt` (по умолчанию `timestep/1000`).

**Команды JSON**
```json
{ "command": "move", "args": [x, y, z, roll, pitch, yaw] }
{ "command": "grab", "args": true }   // true = ЗАКРЫТЬ хват
```

---

### supervisor_csv_moving/supervisor_csv_moving.py
**Назначение**: движение по CSV‑траектории **в суставах**.

- Путь и разделитель: `CSV_PATH`, `SEPARATION`.
- CSV: 7 колонок (радианы), рекомендуется заголовок `A1,A2,...,A7`.

---

### supervisor_gizmo_move/supervisor_gizmo_move.py
**Назначение**: следование за целью `TARGET_GRIPPER` (гизма в Scene Tree). Позицию/ориентацию задаёте мышью в Webots.

- IK: `solve_ik(model, qc, xyz, rpy)`.
- Достижение цели: `MotionTracker`.
- **Семантика хвата**: `target.gripper = true` ⇒ **ОТКРЫТЬ** хват; `false` ⇒ закрыть.

---

### supervisor_pattern_collection/supervisor_pattern_collection.py
**Назначение**: построение траектории от `qz` к `qr`, выполнение последовательности `move/collect` и сохранение кадров.

- Пути/параметры: `JSON_CARTESIAN_PATH`, `IMAGE_FOLDER`, `dt` (по умолчанию 10 мс).
- В момент `collect` отправляет `save_image` в `IMAGE_FOLDER` (тип — RGB).
- Сохраняет пройденную траекторию в `controllers/supervisor_pattern_collection/trajectory_collect.csv`.

**JSON команды**
```json
{ "command": "move",    "args": [x,y,z,r,p,y] }
{ "command": "collect", "args": true }
```

---

### supervisor_pattern_detection/supervisor_pattern_detection.py
**Назначение**: следование за гизмой + **онлайн‑детекция паттернов**.

- По умолчанию включена детекция **ChArUco** 5×7 (`55 мм`, `40 мм`, словарь `4x4_50`). Для ArUco/Chessboard/CircleGrid раскомментируйте соответствующие блоки в конце файла.
- При переходе состояния `gripper: false → true` текущая поза логируется в `controllers/supervisor_pattern_detection/robot_poses.json` как:
  ```json
  { "command": "move", "args": [x,y,z,r,p,y] }
  { "command": "collect", "args": true }
  ```

---

## Управление камерой и сохранение данных
**Сохранение кадров**
```json
{
  "source": "supervisor",
  "type": "save_image",
  "data": { "folder": "./images", "type": "rgb|depth|rgbd" }
}
```
- `rgb` — цветное изображение
- `depth` — глубина
- `rgbd` — оба варианта

**Детекция паттернов**
```json
{
  "source": "supervisor",
  "type": "pattern_detection",
  "data": { "pattern_type": "...", "params": { ... } }
}
```
Поддерживаемые `pattern_type`: `charuco`, `aruco`, `chessboard`, `circlegrid` (`asymmetric` для circlegrid).

Наложенный оверлей предназначен для визуализации; сохранение выполняется стандартной командой `save_image` (сохраняются «сырые» сенсоры).

---

## Генерация калибровочного паттерна
Утилита `generate_calibration_pattern.py` создаёт PNG и **обновляет** `protos/CalibrationPattern.proto` (поля `textureUrl` и `size`).

**Пример (ChArUco, A3):**
```bash
python generate_calibration_pattern.py \
  --type charuco \
  -oi protos/textures/handeye_pattern.png \
  -gc 5 7 -cs 55 -ml 40 --dpi 50 -pf A3 -ad 4x4_50
```
Ключевые опции:
- `--type/-t`: `charuco | aruco | chessboard | circlegrid`
- `--output_image/-oi`: путь к PNG
- `--proto_path/-pp`: путь к `.proto` (по умолчанию `./protos/CalibrationPattern.proto`)
- `--grid_cells/-gc`: (ширина, высота)
- `--cell_size/-cs`: размер клетки в мм
- ChArUco: `--marker_length` (мм), `--aruco_dict`
- ArUco: `--marker_length` (мм), `--aruco_dict`
- CircleGrid: `--asymmetric`

После генерации **перезагрузите мир** или узел шаблона, чтобы подтянулись новая текстура и размеры.

---

## Форматы файлов (JSON/CSV)

### JSON — декартовое движение
Массив объектов:
```json
[
  { "command": "move", "args": [x, y, z, roll, pitch, yaw] },
  { "command": "grab", "args": true }
]
```
Координаты в **метрах/радианах** относительно базы робота (см. IK‑модель).

### JSON — robot_poses.json (детектор поз)
Сохраняется `supervisor_pattern_detection` при событии `gripper false→true`:
```json
{ "command": "move", "args": [x,y,z,r,p,y] }
{ "command": "collect", "args": true }
```

### CSV — суставная траектория
- 7 колонок (радианы): `A1,A2,A3,A4,A5,A6,A7`
- Разделитель задаётся в `SEPARATION` (по умолчанию `,`).

---

## Пути и переменные окружения
В начале некоторых супервизоров есть константы для путей:
- `JSON_CARTESIAN_PATH`, `CSV_PATH`
- `IMAGE_FOLDER`
Файлы используют `os.path.expandvars`, поэтому можно писать `${HOME}/...`. Рекомендуется делать пути **относительными** к корню проекта, если запускаете Webots из каталога проекта.

---

## Реализованные модули (кратко)

### `extensions/board_generation`
- `ArucoBoard`, `CharucoBoard`, `ChessBoard`, `CircleGridBoard` — генерация PNG, учёт DPI и форматов бумаги, словари ArUco.

### `extensions/core`
- `CommandBuilder` — формирование команд `robot_position` (целевые суставы + хват).
- `MotionTracker` — проверка достижения цели по допускам.
- `communication` — JSON‑шина поверх Emitter/Receiver.
- `state`, `target`, `ik` — вспомогательная логика.

### `extensions/kinematics`
- `robot_models` — `LBRiiwaR800Model` (позы `qz`, `qr`).
- `solvers` — `solve_ik` (обратная кинематика), `solve_pzk` (прямая).
- `planner` — `TrajectoryPlannerComponent` (планирование суставных траекторий под декартовые цели, `speed_scale`, `dt`).

### `extensions/webots`
- `communication` — обёртка для обмена сообщениями.
- `target` — интерфейс работы с гизмой `TARGET_GRIPPER`.
- `camera/sensors` — драйверы RGB/Depth.
- `camera/processor` — распознавание паттернов + оверлей.
- `camera/publisher` — публикация изображений/метаданных.

### `extensions/utils`
- `device_search` — распознавание моторов хвата по имени.
- `console` — прогресс‑бар.
- `math` и др. утилиты.

---

## Советы и устранение неполадок
- **Траектория не строится**: сообщение вида `[WARN] Не удалось построить траекторию ...`. Проверьте достижимость позы, коллизии, уменьшите `speed_scale`, увеличьте `dt`, задайте корректное стартовое состояние `current_q`.
- **Изображения не сохраняются**: проверьте, что мир содержит узел камеры с контроллером `camera_controller`, а супервизор отправляет корректную `save_image` с существующей папкой. В Linux убедитесь в правах на запись.
- **Хват «перепутан»**: в разных супервизорах разная семантика булева значения для удобства сценариев:
  - `supervisor_cartesian_move`: `"grab": true` ⇒ **закрыть**.
  - `supervisor_gizmo_move`: `target.gripper = true` ⇒ **открыть**.
- **Паттерн не детектируется**: проверьте соответствие параметров реально сгенерированному изображению (`grid_cells`, размеры клеток/маркеров, словарь ArUco) и масштаб паттерна в кадре.
- **VS Code не видит API Webots**: добавьте путь к каталогу с модулем `controller` в `python.analysis.extraPaths` и `python.autoComplete.extraPaths` (см. раздел настройки).
- **Webots запускает не тот Python**: укажите интерпретатор venv в `Tools → Preferences → Python command`.
