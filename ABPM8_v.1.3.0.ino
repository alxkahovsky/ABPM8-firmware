/*
  Ротатор для лайв сонара на Arduino Nano
  Управление: NEMA 23 + TMC2160 + BMI160 IMU

  ИСПРАВЛЕНИЕ: stepper.autoPower(false) в режиме Hold,
  чтобы библиотека не отключала мотор автоматически.
*/

#include "GyverStepper.h" 
#include <EEPROM.h>       
#include <BMI160Gen.h>    
#include "MadgwickAHRS.h" 
#include "GyverFilters.h" 
#include <Wire.h>

//////////////////// СЕКЦИЯ НАСТРОЕК ////////////////////////

#define RightButton  A0   
#define LeftButton   A1   
#define StabButton   A2   
#define HoldButton   A3   
#define SpeedPotPin  A7   

#define LedPower     A5    // D3 - питание
#define LedStab      A4    // D4 - стабилизация
#define LedHold      4    // D5 - удержание

#define StepPin      6    
#define DirPin       7    
#define EnablePin    8    

#define MicroStep      6400   
#define GearRatio      1      
#define MotorDirection 1      
#define AngleToStep    17.78f 

#define NeutralZone     0.8f  
#define UpdateThreshold 10    
#define SpeedMin        50    
#define SpeedMax        1600  
#define Acceleration    0     

#define TO_RAD       0.01745329252f
#define imuPeriod    20       

//////////////////// ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ ////////////////////////

GStepper<STEPPER2WIRE> stepper(MicroStep, StepPin, DirPin, EnablePin);

GMedian3<int> filterAxelX, filterAxelY, filterAxelZ;
GMedian3<int> filterGyroX, filterGyroY, filterGyroZ;

bool StabMode = false;       
bool HoldMode = false;       
bool IMU_Enable = false;     
bool pedalActive = false;    
bool needBaseUpdate = false; 

float BaseYaw = 0.0f;        
float CurrentYaw = 0.0f;     
long BaseMotorPos = 0;       
long LastSetTarget = 0;      

float tdelta;
int aix, aiy, aiz;
int gix, giy, giz;
float imu[3];
float quat[4];
uint32_t imu_t = 0;

uint32_t btnStabTimer = 0;
uint32_t btnHoldTimer = 0;
uint32_t pedalReleaseTime = 0; 
bool lastStabState = HIGH;
bool lastHoldState = HIGH;
bool lastRightState = HIGH;
bool lastLeftState = HIGH;

//////////////////// ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ ////////////////////////

float getAngleDiff(float base, float current) {
    float diff = current - base;
    while (diff > 180.0f)  diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;
    return diff;
}

// Тестовое мигание всех светодиодов
void testLeds() {
    for (int i = 0; i < 3; i++) {
        digitalWrite(LedPower, HIGH); delay(150); digitalWrite(LedPower, LOW);
        digitalWrite(LedStab, HIGH);  delay(150); digitalWrite(LedStab, LOW);
        digitalWrite(LedHold, HIGH);  delay(150); digitalWrite(LedHold, LOW);
    }
}

//////////////////// SETUP ////////////////////////

void setup() {
    Serial.begin(115200);
    delay(500);

    pinMode(RightButton, INPUT_PULLUP);
    pinMode(LeftButton,  INPUT_PULLUP);
    pinMode(StabButton,  INPUT_PULLUP);
    pinMode(HoldButton,  INPUT_PULLUP);
    
    pinMode(LedPower, OUTPUT);
    pinMode(LedStab,  OUTPUT);
    pinMode(LedHold,  OUTPUT);

    // Тест светодиодов: все 3 должны мигнуть по очереди 3 раза
    Serial.println(F("Testing LEDs..."));
    testLeds();

    stepper.autoPower(true);
    stepper.setAcceleration(Acceleration * GearRatio);
    stepper.setMaxSpeed(SpeedMax * GearRatio);
    stepper.disable();

    Serial.println(F("IMU initialization..."));
    if (BMI160.begin(BMI160GenClass::SPI_MODE, 10)) {
        IMU_Enable = true;
        BMI160.setGyroRate(50);
        BMI160.setAccelerometerRate(50);
        BMI160.setAccelDLPFMode(1);
        BMI160.setGyroDLPFMode(1);
        BMI160.setAccelerometerRange(2);
        BMI160.setGyroRange(250);

        for (int i = 0; i < 15; i++) {
            digitalWrite(LedPower, HIGH);
            delay(35);
            digitalWrite(LedPower, LOW);
            delay(35);
        }

        Serial.println(F("Calibrating... Keep still!"));

        BMI160.autoCalibrateGyroOffset();
        BMI160.autoCalibrateAccelerometerOffset(X_AXIS, 0);
        BMI160.autoCalibrateAccelerometerOffset(Y_AXIS, 0);
        BMI160.autoCalibrateAccelerometerOffset(Z_AXIS, 1);

        EEPROM.put(36, BMI160.getAccelerometerOffset(X_AXIS));
        EEPROM.put(40, BMI160.getAccelerometerOffset(Y_AXIS));
        EEPROM.put(44, BMI160.getAccelerometerOffset(Z_AXIS));
        EEPROM.put(48, BMI160.getGyroOffset(X_AXIS));
        EEPROM.put(52, BMI160.getGyroOffset(Y_AXIS));
        EEPROM.put(56, BMI160.getGyroOffset(Z_AXIS));

        digitalWrite(LedPower, HIGH);
        delay(600);
        digitalWrite(LedPower, LOW);
        
        Serial.println(F("Calibration done. System ready."));

    } else {
        Serial.println(F("!!! IMU NOT FOUND !!!"));
        while (true) {
            digitalWrite(LedPower, HIGH);
            delay(300);
            digitalWrite(LedPower, LOW);
            delay(300);
        }
    }
}

//////////////////// LOOP ////////////////////////

void loop() {
    stepper.tick();

    // --- Потенциометр скорости ---
    static uint32_t potTimer = 0;
    if (millis() - potTimer > 100) {
        potTimer = millis();
        int SpeedPotValue = analogRead(SpeedPotPin);
        long spd = map(SpeedPotValue, 0, 1023, SpeedMin, SpeedMax) * GearRatio;
        stepper.setMaxSpeed(spd);
    }

    // --- Чтение IMU ---
    if (IMU_Enable && (millis() - imu_t > imuPeriod)) {
        tdelta = (millis() - imu_t) / 1000.0f;
        imu_t = millis();
        
        BMI160.readMotionSensor(aix, aiy, aiz, gix, giy, giz);

        float gx_mpu = filterGyroX.filtered(gix) * TO_RAD / 131.0;
        float gy_mpu = filterGyroY.filtered(giy) * TO_RAD / 131.0;
        float gz_mpu = filterGyroZ.filtered(giz) * TO_RAD / 131.0;
        float ax_mpu = filterAxelX.filtered(aix);
        float ay_mpu = filterAxelY.filtered(aiy);
        float az_mpu = filterAxelZ.filtered(aiz);

        MadgwickAHRSupdateIMU(tdelta, gx_mpu, gy_mpu, gz_mpu, ax_mpu, ay_mpu, az_mpu);
        quat[0] = q0; quat[1] = q1; quat[2] = q2; quat[3] = q3;
        quat2Euler(&quat[0], &imu[0]);
        CurrentYaw = imu[2] / TO_RAD;
    }

    // --- СТАБИЛИЗАЦИЯ (A2) ---
    bool curStab = digitalRead(StabButton);
    if (curStab == LOW && lastStabState == HIGH && (millis() - btnStabTimer > 50)) {
        btnStabTimer = millis();
        StabMode = !StabMode;

        if (StabMode) {
            HoldMode = false;
            digitalWrite(LedHold, LOW);
            stepper.autoPower(true); // В режиме стабилизации autoPower можно оставить
            
            float yawSum = 0.0f;
            for (int i = 0; i < 5; i++) {
                yawSum += CurrentYaw;
                delay(20);
            }
            BaseYaw = yawSum / 5.0f;
            BaseMotorPos = stepper.getCurrent();
            LastSetTarget = BaseMotorPos;

            stepper.setTarget(BaseMotorPos, ABSOLUTE);
            stepper.enable();
            Serial.print(F("Stab ON. Yaw=")); Serial.print(BaseYaw);
            Serial.print(F(" Pos=")); Serial.println(BaseMotorPos);
        } else {
            stepper.brake();
            stepper.disable();
            Serial.println(F("Stab OFF"));
        }
    }
    lastStabState = curStab;

    // --- СИЛОВОЕ УДЕРЖАНИЕ (A3) ---
    bool curHold = digitalRead(HoldButton);
    if (curHold == LOW && lastHoldState == HIGH && (millis() - btnHoldTimer > 50)) {
        btnHoldTimer = millis();
        HoldMode = !HoldMode;

        if (HoldMode) {
            StabMode = false;
            digitalWrite(LedStab, LOW);
            
            // ★ КЛЮЧЕВОЕ ИСПРАВЛЕНИЕ ★
            // Отключаем autoPower, иначе tick() мгновенно отключит мотор,
            // потому что цель == текущая позиция и библиотека решит,
            // что "мотор на месте, можно выключить"
            stepper.autoPower(false);
            stepper.enable();
            stepper.setTarget(stepper.getCurrent(), ABSOLUTE);
            
            Serial.print(F("Hold ON. Pos="));
            Serial.println(stepper.getCurrent());
        } else {
            // Возвращаем autoPower для нормальных режимов
            stepper.autoPower(true);
            stepper.disable();
            Serial.println(F("Hold OFF"));
        }
    }
    lastHoldState = curHold;

    // --- ПЕДАЛИ (A0/A1) ---
    bool curRight = digitalRead(RightButton);
    bool curLeft  = digitalRead(LeftButton);

    bool rightPressed = (curRight == LOW && lastRightState == HIGH);
    bool leftPressed  = (curLeft  == LOW && lastLeftState  == HIGH);

    if (rightPressed || leftPressed) {
        pedalActive = true;       
        needBaseUpdate = false;   
        stepper.autoPower(true); // Педали работают с autoPower
        stepper.enable();

        if (rightPressed) {
            stepper.setTarget(-1L * 360L * (long)MicroStep / GearRatio, RELATIVE);
        }
        if (leftPressed) {
            stepper.setTarget(1L * 360L * (long)MicroStep / GearRatio, RELATIVE);
        }
    }

    bool rightReleased = (curRight == HIGH && lastRightState == LOW);
    bool leftReleased  = (curLeft  == HIGH && lastLeftState  == LOW);

    if (rightReleased || leftReleased) {
        stepper.brake();          
        needBaseUpdate = true;    
        pedalReleaseTime = millis(); 
    }

    lastRightState = curRight;
    lastLeftState  = curLeft;

    // --- ОБНОВЛЕНИЕ БАЗЫ ПОСЛЕ ПЕДАЛИ ---
    if (needBaseUpdate && (millis() - pedalReleaseTime > 300)) {
        if (StabMode) {
            BaseMotorPos = stepper.getCurrent();
            float yawSum = 0.0f;
            for (int i = 0; i < 3; i++) {
                yawSum += CurrentYaw;
                delay(20);
            }
            BaseYaw = yawSum / 3.0f;
            LastSetTarget = BaseMotorPos;
            Serial.print(F("Base updated. Yaw=")); Serial.print(BaseYaw);
            Serial.print(F(" Pos=")); Serial.println(BaseMotorPos);
        }
        needBaseUpdate = false;
        pedalActive = false;
    }

    // --- ЯДРО СТАБИЛИЗАЦИИ ---
    if (StabMode && IMU_Enable && !pedalActive) {
        float yawDelta = getAngleDiff(BaseYaw, CurrentYaw);
        long newTarget = BaseMotorPos - (long)round(yawDelta * AngleToStep * GearRatio) * MotorDirection;

        if (abs(yawDelta) > NeutralZone) {
            if (abs(newTarget - LastSetTarget) > UpdateThreshold) {
                stepper.setTarget(newTarget, ABSOLUTE);
                LastSetTarget = newTarget;
            }
        }
    }

    // --- СВЕТОДИОДЫ ---
    digitalWrite(LedPower, HIGH);                  
    digitalWrite(LedStab,  StabMode ? HIGH : LOW); 
    digitalWrite(LedHold,  HoldMode ? HIGH : LOW); 
}
