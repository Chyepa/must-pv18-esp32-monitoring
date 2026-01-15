# ⚡ MUST PV-18 Inverter Monitoring

🇺🇦 **Українська версія**  
🇬🇧 **English version below**

---

## 🇺🇦 Опис проєкту

Цей проєкт — це веб-панель моніторингу для інвертора **MUST PV-18**, побудована на базі **ESP32 + PHP + MySQL**.

Система отримує дані з інвертора в реальному часі, зберігає їх у базі даних та візуалізує у вигляді інтерактивного дашборду з графіками, аналітикою акумулятора, статистикою енергії та оцінкою автономності.

Проєкт розроблений як практичне рішення для домашньої сонячної електростанції з можливістю глибокого аналізу роботи інвертора та АКБ.

---

## 🔧 Основні можливості

- ☀️ Моніторинг сонячної генерації (PV)
- 🔵 Відображення поточного споживання
- 🔋 Обчислення SOC акумулятора (енергетичний метод + корекція по напрузі)
- ♻️ Підрахунок циклів батареї та **SOH (State of Health)**
- ⚡ Визначення імпорту / експорту з електромережі
- ⏳ Оцінка часу роботи АКБ до 20% та до 0%
- 📊 Інтерактивні графіки потужності (Chart.js)
- 🗂 Детальна статистика за день / тиждень / місяць / рік / весь період
- 🧠 Автоматичне визначення режиму роботи: Grid / Solar / Battery / Off-grid
- 🌡 Моніторинг температури інвертора
- 📶 Контроль онлайн-статусу пристрою

---

## 🧩 Архітектура системи

MUST PV-18 Inverter
│
│ RS485 / UART
▼
ESP32
│
│ HTTP POST
▼
PHP API → MySQL Database
│
▼
Web Dashboard (Chart.js)


## 🔋 Алгоритм розрахунку SOC

Рівень заряду акумулятора (SOC) визначається комбінованим методом:

1. **Енергетична інтеграція** — облік зарядженої та розрядженої енергії (кВт·год).
2. **Корекція по напрузі** — зіставлення з реальною напругою АКБ.
3. **Автокалібрування**:
   - 100% при завершенні заряду (float-режим).
   - 0% при глибокому розряді.

Також:
- розраховуються **приблизні цикли батареї**,
- оцінюється **SOH (State of Health)** для LiFePO₄ акумулятора.

---

## 🛠 Використані технології

- **ESP32** (збір телеметрії)
- **PHP 8+**
- **MySQL / MariaDB**
- **Chart.js**
- HTML / CSS

---

## 🚀 Встановлення

1. Клонувати репозиторій:
git clone https://github.com/USERNAME/must-pv18-monitor.git

2.Створити базу даних та необхідні таблиці:

-- inverter_live
-- inverter_log
-- inverter_stats_daily
-- inverter_battery_state

3.Налаштувати підключення до БД у PHP:

$mysqli = new mysqli("localhost", "USER", "PASSWORD", "DB_NAME");

4.Завантажити прошивку на ESP32 для передачі даних інвертора. через Ардуіно 


EN

# ⚡ MUST PV-18 Inverter Monitoring

🇺🇦 **Ukrainian version**
🇬🇧 **English version below**

---

## 🇺🇦 Project description

This project is a web monitoring panel for the **MUST PV-18** inverter, built on **ESP32 + PHP + MySQL**.

The system receives data from the inverter in real time, stores it in a database and visualizes it in the form of an interactive dashboard with graphs, battery analytics, energy statistics and autonomy assessment.

The project is designed as a practical solution for a home solar power plant with the ability to deeply analyze the operation of the inverter and battery.

---

## 🔧 Main features

- ☀️ Solar generation (PV) monitoring
- 🔵 Current consumption display
- 🔋 Battery SOC calculation (energy method + voltage correction)
- ♻️ Battery cycle count and **SOH (State of Health)**
- ⚡ Grid import/export determination
- ⏳ Battery operating time estimation up to 20% and up to 0%
- 📊 Interactive power graphs (Chart.js)
- 🗂 Detailed statistics for the day/week/month/year/all period
- 🧠 Automatic detection of operating mode: Grid/Solar/Battery/Off-grid
- 🌡 Inverter temperature monitoring
- 📶 Device online status monitoring

---

## 🧩 System architecture

MUST PV-18 Inverter
│
│ RS485/UART
▼
ESP32
│
│ HTTP POST
▼
PHP API → MySQL Database
│
▼
Web Dashboard (Chart.js)

## 🔋 SOC calculation algorithm

The battery state of charge (SOC) is determined by a combined method:

1. **Energy integration** — accounting for charged and discharged energy (kWh).
2. **Voltage correction** — comparison with the real battery voltage.
3. **Autocalibration**:
- 100% at the end of charging (float mode).
- 0% at deep discharge.

Also:
- **approximate battery cycles** are calculated,
- **SOH (State of Health)** for the LiFePO₄ battery is estimated.

---

## 🛠 Technologies used

- **ESP32** (telemetry collection)

- **PHP 8+**

- **MySQL / MariaDB**

- **Chart.js**

- HTML / CSS

---

## 🚀 Installation

1. Clone the repository:
git clone https://github.com/USERNAME/must-pv18-monitor.git

2. Create a database and the necessary tables:

-- inverter_live
-- inverter_log
-- inverter_stats_daily
-- inverter_battery_state

3. Configure the connection to the database in PHP:

$mysqli = new mysqli("localhost", "USER", "PASSWORD", "DB_NAME");

4. Upload the firmware to the ESP32 to transfer inverter data. via Arduino

