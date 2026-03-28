# [HR Tech] Test Task Autopilot #1 (UAV) — Solution

Скрипт `autopilot_rc_override.py` реалізує політ у **режимі STABILIZE** через **RC Override**:
- зліт до заданої висоти,
- політ з точки A до точки B з утриманням висоти,
- посадка з корекцією позиції в точку B.

## Вхідні умови із завдання

- Точка A (старт): `50.450739, 30.461242`
- Точка B: `50.443326, 30.448078`
- Висота: `300 м`
- Вітер у SITL:
  - `SIM_WIND_SPD = 5`
  - `SIM_WIND_DIR = 30`
  - `SIM_WIND_TURB = 3`
  - `SIM_WIND_TURB_FREQ = 0.25`

## Підготовка (Windows)

1. Встановити Python 3.10+ (з опцією `py` launcher) та Mission Planner.
2. Відкрити PowerShell у папці проєкту.
3. Дозволити запуск локальних скриптів (для поточної сесії):

```powershell
Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass
```

4. Запустити місію (SITL + автопілот):

```powershell
.\run_mission.ps1
```

5. Запустити місію разом з Mission Planner (рекомендовано):

```powershell
.\run_mission_with_mp.ps1
```

Mission Planner підключати як `UDP` на порт `14550`.

### Окремий запуск Mission Planner (PowerShell)

Запуск тільки Mission Planner:

```powershell
.\run_mission_planner.ps1
```

Якщо `MissionPlanner.exe` встановлено в нестандартне місце:

```powershell
.\run_mission_planner.ps1 -MissionPlannerExe "C:\path\to\MissionPlanner.exe"
```

Запуск місії окремо, коли Mission Planner вже відкритий:

```powershell
.\run_mission_with_mp.ps1 -SkipMissionPlannerLaunch -SkipPipInstall
```

Після цього в Mission Planner обрати `UDP` і порт `14550`, натиснути `CONNECT`.

## Прямий запуск скрипта

```powershell
.\.venv\Scripts\python.exe autopilot_rc_override.py --connect tcp:127.0.0.1:5760 --target-lat 50.443326 --target-lon 30.448078 --target-alt 300
```

## Запуск під ключ (SITL + місія однією командою)

```powershell
.\run_mission.ps1
```

Скрипт:
- піднімає SITL із home-point у точці A,
- задає параметри вітру,
- запускає місію до точки B.

Лог SITL пишеться у `sitl.log`.

## Логіка керування

- `CH3` (throttle): PID для утримання висоти при зльоті; зниження при посадці.
- `CH4` (yaw): фіксовано в нейтралі (`1500`), активне yaw-керування вимкнене.
- `CH2` (pitch): PID-корекція позиції до точки B у body-координатах.
- `CH1` (roll): PID-корекція позиції до точки B у body-координатах.

Логіка польоту:
1. **Зліт** до заданої висоти (300 м) з PID-регулятором дросселя.
2. **Крейсерський політ** до точки B з утриманням висоти ~300 м.
3. **Hover-align** над ціллю для зменшення похибки перед спуском.
4. **Посадка** — зниження з XY-корекцією позиції до точки B.

## Що надати як результат (згідно задачі)

- Код у GitHub репозиторії.
- Відео запуску скрипта та польоту в SITL (з видимими координатами/режимом/посадкою в B).

## Результати (5 окремих прогонів, поточна версія)

Останні 5 запусків `run_mission.ps1` (кожен прогін виконувався окремо):

- Run 1: `2.98 m`
- Run 2: `2.68 m`
- Run 3: `2.71 m`
- Run 4: `3.02 m`
- Run 5: `2.74 m`

Зведена статистика:

- `min`: `2.68 m`
- `max`: `3.02 m`
- `avg`: `2.83 m`
- `median`: `2.74 m`
- `std`: `0.14 m`
- `<= 2.0 m`: `0/5`
- `<= 3.0 m`: `4/5`
- `<= 5.0 m`: `5/5`
- `NO_TOUCHDOWN`: `0/5`

## Відео

Відео за вимогами задачі (запуск скрипта + політ у SITL з видимими координатами/режимом/посадкою в B):

- `sitl_flight_required.mp4`

## Примітки

- Скрипт спеціально залишає режим `STABILIZE` і не використовує `GUIDED`/`AUTO` місії.
- Для кращої точності посадки за вітру можна підлаштувати коефіцієнти в секції `landing_at_target()`.
