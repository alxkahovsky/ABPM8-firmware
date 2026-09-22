/*
  Ротатор для лайв сонара на Arduino Nano
  Управление: NEMA 23 + TMC2160 + BMI160 IMU


  РЕЖИМЫ (кнопки-переключатели, нажал-отпустил = вкл/выкл):
  - A2: Стабилизация курса (компенсация поворотов лодки по данным гироскопа)
  - A3: Силовое удержание вала (мотор просто заблокирован, без коррекции)
  - A0/A1: Педали ручного поворота — РАБОТАЮТ В ЛЮБОМ РЕЖИМЕ
           В режиме стабилизации после поворота педалями база автоматически
           обновляется, и компенсация продолжается от новой точки.

  КАЛИБРОВКА:
  - Автоматически при каждом включении питания
  - Светодиод питания быстро мигает = "Держите лодку неподвижно!"
*/

#include "GyverStepper.h" // https://github.com/GyverLibs/GyverStepper
#include <BMI160Gen.h>    // https://github.com/hanyazou/BMI160-Arduino
#include "MadgwickAHRS.h" // Фильтр ориентации (кватернион -> угол)
#include "GyverFilters.h" // Медианные фильтры
#include <SPI.h>

//////////////////// СЕКЦИЯ НАСТРОЕК ////////////////////////

// --- Пины кнопок и педалей ---
#define PIN_BTN_RIGHT  A0   // Педаль: поворот датчика вправо (работает всегда)
#define PIN_BTN_LEFT   A1   // Педаль: поворот датчика влево  (работает всегда)
#define PIN_BTN_STAB   A2   // Переключатель: Стабилизация курса
#define PIN_BTN_HOLD   A3   // Переключатель: Силовое удержание вала
#define PIN_POT_SPEED  A7   // Потенциометр скорости вращения

// --- Пины светодиодов ---
#define PIN_LED_POWER  A5   // LED 1: Питание / Статус калибровки
#define PIN_LED_STAB   A4   // LED 2: Активен режим стабилизации курса
#define PIN_LED_HOLD   A6   // LED 3: Активен режим силового удержания

// --- Пины драйвера шагового двигателя TMC2160 ---
#define PIN_STEP       6    // CLK
#define PIN_DIR        7    // DIR
#define PIN_EN         8    // EN

// --- Параметры мотора ---
#define MICROSTEPS     6400   // Импульсов на оборот (MRES=32: M0-OFF, M1-ON)
#define GEAR_RATIO     1      // Передаточное число редуктора
#define MOTOR_DIR      1      // 1 или -1 (поменяйте, если компенсация крутит не в ту сторону)
#define ANGLE_TO_STEP  17.78f  // 6400 / 360 = 17.78 шагов на градус

// --- Настройки стабилизации ---
#define NEUTRAL_ZONE     0.8f  // Мертвая зона в градусах
#define UPDATE_THRESHOLD 10    // Мин. изменение цели в шагах для отправки команды (~0.56°)
                               // ГЛАВНАЯ защита от микро-дерганий!
#define SPEED_MIN        50    // Мин. скорость (шаг/сек)
#define SPEED_MAX        1600  // Макс. скорость (шаг/сек)
#define ACCELERATION     0     // Ускорение (0 = мгновенный отклик)

// --- Настройки IMU ---
#define IMU_WARMUP_MS    3000  // Время прогрева фильтра Маджвика после калибровки

//////////////////// ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ ////////////////////////

GStepper<STEPPER2WIRE> stepper(MICROSTEPS, PIN_STEP, PIN_DIR, PIN_EN);
Madgwick madgwickFilter;

// Медианные фильтры для подавления выбросов IMU
GMedian3<int> filterAxelX, filterAxelY, filterAxelZ;
GMedian3<int> filterGyroX, filterGyroY, filterGyroZ;

// Состояния системы
bool stabMode = false;       // Режим стабилизации курса
bool holdMode = false;       // Режим силового удержания вала
bool imuReady = false;       // Фильтр Маджвика прогрелся
bool imuConnected = false;   // IMU физически подключен
bool pedalActive = false;    // Педаль нажата ИЛИ мотор ещё крутится после неё
bool needBaseUpdate = false; // Нужно обновить базу после отпускания педали

// Данные стабилизации
float baseYaw = 0.0f;        // Курс лодки в момент включения стабилизации / обновления базы
float currentYaw = 0.0f;     // Текущий курс лодки по гироскопу
long baseMotorPos = 0;       // Позиция мотора в момент включения стабилизации / обновления базы
long lastSetTarget = 0;      // Последняя отправленная мотору целевая позиция

// Таймеры
uint32_t imuTimer = 0;
uint32_t btnStabTimer = 0;
uint32_t btnHoldTimer = 0;
uint32_t startupTime = 0;

// Debounce состояния кнопок
bool lastBtnStabState = HIGH;
bool lastBtnHoldState = HIGH;
bool lastBtnRightState = HIGH;
bool lastBtnLeftState = HIGH;

//////////////////// ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ ////////////////////////

// Извлечение угла Yaw (курс) из кватернионов фильтра Маджвика (0...360°)
float getYawFromQuat(float q0, float q1, float q2, float q3) {
    float yaw = atan2(2.0f * (q0 * q3 + q1 * q2),
                      1.0f - 2.0f * (q2 * q2 + q3 * q3));
    float deg = yaw * 57.2957795f;
    if (deg < 0.0f) deg += 360.0f;
    return deg;
}

// Кратчайшая разница между двумя углами (с учётом перехода через 0°/360°)
// Возвращает значение в диапазоне [-180, +180]
float getAngleDiff(float base, float current) {
    float diff = current - base;
    while (diff > 180.0f)  diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;
    return diff;
}


void setup() {
    Serial.begin(115200);
    startupTime = millis();

    pinMode(PIN_BTN_RIGHT, INPUT_PULLUP);
    pinMode(PIN_BTN_LEFT,  INPUT_PULLUP);
    pinMode(PIN_BTN_STAB,  INPUT_PULLUP);
    pinMode(PIN_BTN_HOLD,  INPUT_PULLUP);
    pinMode(PIN_LED_POWER, OUTPUT);
    pinMode(PIN_LED_STAB,  OUTPUT);
    pinMode(PIN_LED_HOLD,  OUTPUT);

    stepper.autoPower(true);
    stepper.setAcceleration(ACCELERATION * GEAR_RATIO);
    stepper.setMaxSpeed(SPEED_MAX * GEAR_RATIO);
    stepper.disable();

    Serial.println(F("IMU init..."));
    if (BMI160.begin(BMI160GenClass::SPI_MODE, 10)) {
        imuConnected = true;
        BMI160.setGyroRate(50);
        BMI160.setAccelerometerRate(50);
        BMI160.setAccelDLPFMode(1);
        BMI160.setGyroDLPFMode(1);
        BMI160.setAccelerometerRange(2);
        BMI160.setGyroRange(250);

        // =============================================
        //  АВТОМАТИЧЕСКАЯ КАЛИБРОВКА ПРИ ВКЛЮЧЕНИИ
        // =============================================
        for (int i = 0; i < 15; i++) {
            digitalWrite(PIN_LED_POWER, HIGH);
            delay(35);
            digitalWrite(PIN_LED_POWER, LOW);
            delay(35);
        }

        Serial.println(F("Calibrating... Keep still!"));

        BMI160.autoCalibrateGyroOffset();
        BMI160.autoCalibrateAccelerometerOffset(X_AXIS, 0);
        BMI160.autoCalibrateAccelerometerOffset(Y_AXIS, 0);
        BMI160.autoCalibrateAccelerometerOffset(Z_AXIS, 1);

        digitalWrite(PIN_LED_POWER, HIGH);
        delay(600);
        digitalWrite(PIN_LED_POWER, LOW);

        madgwickFilter.begin(50.0f);
        Serial.println(F("Calibration done. Warming up filter..."));

    } else {
        Serial.println(F("!!! IMU NOT FOUND !!!"));
        while (true) {
            digitalWrite(PIN_LED_POWER, HIGH);
            delay(300);
            digitalWrite(PIN_LED_POWER, LOW);
            delay(300);
        }
    }
}


void loop() {
    stepper.tick();

    static uint32_t potTimer = 0;
    if (millis() - potTimer > 100) {
        potTimer = millis();
        int potVal = analogRead(PIN_POT_SPEED);
        long spd = map(potVal, 0, 1023, SPEED_MIN, SPEED_MAX) * GEAR_RATIO;
        stepper.setMaxSpeed(spd);
    }

    if (imuConnected && (millis() - imuTimer > 20)) {
        imuTimer = millis();

        int aix, aiy, aiz, gix, giy, giz;
        BMI160.readMotionSensor(aix, aiy, aiz, gix, giy, giz);

        float gx = filterGyroX.filtered(gix) * 0.01745329252f / 131.0f;
        float gy = filterGyroY.filtered(giy) * 0.01745329252f / 131.0f;
        float gz = filterGyroZ.filtered(giz) * 0.01745329252f / 131.0f;
        float ax = filterAxelX.filtered(aix);
        float ay = filterAxelY.filtered(aiy);
        float az = filterAxelZ.filtered(aiz);

        madgwickFilter.update(gx, gy, gz, ax, ay, az);
        currentYaw = getYawFromQuat(madgwickFilter.q0, madgwickFilter.q1,
                                     madgwickFilter.q2, madgwickFilter.q3);

        if (!imuReady && (millis() - startupTime > IMU_WARMUP_MS)) {
            imuReady = true;
            Serial.println(F("IMU ready."));
        }
    }

    bool curStab = digitalRead(PIN_BTN_STAB);
    if (curStab == LOW && lastBtnStabState == HIGH && (millis() - btnStabTimer > 50)) {
        btnStabTimer = millis();

        if (imuReady) {
            stabMode = !stabMode;

            if (stabMode) {
                holdMode = false;

                // Усредняем курс лодки за 100мс (5 замеров × 20мс)
                float yawSum = 0.0f;
                for (int i = 0; i < 5; i++) {
                    yawSum += currentYaw;
                    delay(20);
                }
                baseYaw = yawSum / 5.0f;
                baseMotorPos = stepper.getCurrent();
                lastSetTarget = baseMotorPos;

                stepper.setTarget(baseMotorPos, ABSOLUTE);
                stepper.enable();

                Serial.print(F("Stab ON. BaseYaw="));
                Serial.print(baseYaw);
                Serial.print(F(" BasePos="));
                Serial.println(baseMotorPos);
            } else {
                stepper.brake();
                stepper.disable();
                Serial.println(F("Stab OFF"));
            }
        } else {
            // IMU не прогрелся — коротко мигаем LED_STAB
            for (int i = 0; i < 3; i++) {
                digitalWrite(PIN_LED_STAB, HIGH);
                delay(80);
                digitalWrite(PIN_LED_STAB, LOW);
                delay(80);
            }
        }
    }
    lastBtnStabState = curStab;

    bool curHold = digitalRead(PIN_BTN_HOLD);
    if (curHold == LOW && lastBtnHoldState == HIGH && (millis() - btnHoldTimer > 50)) {
        btnHoldTimer = millis();
        holdMode = !holdMode;

        if (holdMode) {
            stabMode = false;
            stepper.enable();
            stepper.brake();
            Serial.println(F("Hold ON"));
        } else {
            stepper.disable();
            Serial.println(F("Hold OFF"));
        }
    }
    lastBtnHoldState = curHold;

    bool curRight = digitalRead(PIN_BTN_RIGHT);
    bool curLeft  = digitalRead(PIN_BTN_LEFT);

    // --- Фронт нажатия любой педали ---
    bool rightPressed = (curRight == LOW && lastBtnRightState == HIGH);
    bool leftPressed  = (curLeft  == LOW && lastBtnLeftState  == HIGH);

    if (rightPressed || leftPressed) {
        pedalActive = true;       // Приостанавливаем компенсацию на время поворота
        needBaseUpdate = false;   // Сбрасываем старый запрос на обновление базы
        stepper.enable();

        if (rightPressed) {
            // Крутим на 1 оборот вправо (RELATIVE)
            stepper.setTarget(-1L * 360L * (long)MICROSTEPS / GEAR_RATIO, RELATIVE);
        }
        if (leftPressed) {
            // Крутим на 1 оборот влево (RELATIVE)
            stepper.setTarget(1L * 360L * (long)MICROSTEPS / GEAR_RATIO, RELATIVE);
        }
    }

    // --- Фронт отпускания педали ---
    bool rightReleased = (curRight == HIGH && lastBtnRightState == LOW);
    bool leftReleased  = (curLeft  == HIGH && lastBtnLeftState  == LOW);

    if (rightReleased || leftReleased) {
        stepper.brake();          // Плавно останавливаем мотор
        needBaseUpdate = true;    // Запрашиваем обновление базы после остановки
    }

    lastBtnRightState = curRight;
    lastBtnLeftState  = curLeft;

    // --- Обновление базы после остановки мотора ---
    // Ждём, пока мотор действительно остановится, и только потом фиксируем новую точку
    if (needBaseUpdate && stepper.getState() == STEPPER_STOP) {
        if (stabMode) {
            // В режиме стабилизации: обновляем базу, чтобы компенсация
            // продолжалась уже от НОВОГО направления датчика
            baseMotorPos = stepper.getCurrent();
            // Усредняем текущий курс лодки для плавности
            float yawSum = 0.0f;
            for (int i = 0; i < 3; i++) {
                yawSum += currentYaw;
                delay(20);
            }
            baseYaw = yawSum / 3.0f;
            lastSetTarget = baseMotorPos;

            Serial.print(F("Base updated. Yaw="));
            Serial.print(baseYaw);
            Serial.print(F(" Pos="));
            Serial.println(baseMotorPos);
        }
        // В режиме силового удержания или свободном режиме — просто держим текущую позицию
        needBaseUpdate = false;
    }

    // --- Сброс флага pedalActive, когда педаль отпущена и мотор остановился ---
    if (pedalActive && curRight == HIGH && curLeft == HIGH &&
        stepper.getState() == STEPPER_STOP) {
        pedalActive = false;
    }

    if (stabMode && imuConnected && !pedalActive) {
        float yawDelta = getAngleDiff(baseYaw, currentYaw);

        long newTarget = baseMotorPos
                         - (long)round(yawDelta * ANGLE_TO_STEP * GEAR_RATIO) * MOTOR_DIR;

        if (abs(yawDelta) > NEUTRAL_ZONE) {
            if (abs(newTarget - lastSetTarget) > UPDATE_THRESHOLD) {
                stepper.setTarget(newTarget, ABSOLUTE);
                lastSetTarget = newTarget;
            }
        }
    }


    digitalWrite(PIN_LED_POWER, HIGH);                  // Питание: горит всегда
    digitalWrite(PIN_LED_STAB,  stabMode ? HIGH : LOW); // Стабилизация
    digitalWrite(PIN_LED_HOLD,  holdMode ? HIGH : LOW); // Силовое удержание
}
