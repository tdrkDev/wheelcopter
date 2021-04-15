/*
 * WheelCopter (Arduino Mega)
 * 
 * Copyright (c) Kvantorium Pskov, Roman Rihter 2021
 * Licensed by GNU GPLv3
 */

#include <LiquidCrystal.h>
#include <IBusBM.h>
#include <Servo.h>
#include <Encoder.h>

// Screen
byte db1Port = 27;
byte db2Port = 26;
byte db3Port = 24;
byte db4Port = 22;
byte rsPort = 23;
byte enPort = 25;

// Servo
byte servoPort = 5;

// PID
float pidP = 0.5;
float pidI = 0.5;
float pidD = 0.5;
float pidDT = 1;

// Motors
byte inPort[4] = { 6, 7, 8, 9 };
byte encoderInt1 = 18;
byte encoderDir1 = 19;
byte encoderInt2 = 20;
byte encoderDir2 = 21;

// Battery sensor
byte batPort = A7;
byte batIndex = 9; // Get that value by debugging on battery level get

// Buzzer
byte buzPin = 10;

// Constructors
IBusBM IBus;
Servo boxServo;
LiquidCrystal lcd = LiquidCrystal(rsPort, enPort, db1Port, db2Port, db3Port, db4Port);
Encoder enc1(encoderInt1, encoderDir1);
Encoder enc2(encoderInt2, encoderDir2);
unsigned int prevMs = millis();
unsigned int prevMs2 = millis();
bool noBat = true;
int32_t batLevel = 0;

#define THROTTLE_CH 1
#define ROTATION_CH 3
#define BOXSERVO_CH 5

struct carChData {
    int32_t throttle;
    int32_t rotation;
    int32_t box;
};

struct carMotorData {
    int32_t spd1;
    int32_t spd2;
    bool rev1;
    bool rev2;
    volatile int32_t enc1;
    volatile int32_t enc2;
    bool openBox;
};

struct carScreenData {
    char line1[16];
    char line2[16];
};

struct carData {
    struct carChData ch;
    struct carMotorData mot;
    struct carScreenData scr;
};

void setup()
{
    int i = 0;

    pinMode(batPort, INPUT);

	IBus.begin(Serial2);
    boxServo.attach(servoPort);
    lcd.begin(16, 2);
    lcd.print("Loading...");

    for (i = 0; i < 4; i++) {
        pinMode(inPort[i], OUTPUT);
    }

    Serial.begin(115200);
}

struct carChData carFillChData() 
{
    struct carChData ret;

    ret.throttle = IBus.readChannel(THROTTLE_CH);
    ret.rotation = IBus.readChannel(ROTATION_CH);
    ret.box =      IBus.readChannel(BOXSERVO_CH);

    ret.throttle -= 990;
    ret.rotation -= 990;
    ret.box -=      990;

    return ret;
}

void carHardwareMotor(struct carMotorData mot)
{
    bool inEnable[4] = { false, false, false, false };
    int i = 0;

    if (mot.rev1) {
        inEnable[0] = true;
    } else {
        inEnable[1] = true;
    }

    if (mot.rev2) {
        inEnable[2] = true;
    } else {
        inEnable[3] = true;
    }

    if (!noBat) {
        for (i = 0; i < 4; i++) {
            if (i < 2)
                analogWrite(inPort[i], inEnable[i] ? mot.spd1 : 0);
            else
                analogWrite(inPort[i], inEnable[i] ? mot.spd2 : 0);
        }
    }
}

char getBatteryIcon(byte level) {
    if (level < 90) {
        return '\x9C';
    } else if (level < 70) {
        return '\x9D';
    } else if (level < 40) {
        return '\x9E';
    } else if (level < 10) {
        return '\x9F';
    } else {
        return '\x9B';
    }
}

int computePID(float input, float setpoint, float kp, float ki, float kd, float dt, int minOut, int maxOut) {
    float err = setpoint - input;
    static float integral = 0, prevErr = 0;
    integral = constrain(integral + (float)err * dt * ki, minOut, maxOut);
    float D = (err - prevErr) / dt;
    prevErr = err;
    return constrain(err * kp + integral + D * kd, minOut, maxOut);
}

struct carMotorData computeSpeed(int setpoint, struct carMotorData mot) {
    struct carMotorData ret = mot;

    ret.spd1 = computePID(mot.enc1 / 70, setpoint, pidP, pidI, pidD, pidDT, 0, 255);
    ret.spd2 = computePID(mot.enc2 / 70, setpoint, pidP, pidI, pidD, pidDT, 0, 255);

    return ret;
}

int32_t prevMs3 = 0;

int32_t oldPos[2] = { 0, 0 }, oldTime = 0;

struct carMotorData carGetEncodersValue() {
    struct carMotorData ret;

    int32_t newTime = millis();
    int32_t spd;

    volatile int32_t val1 = enc1.read();
    volatile int32_t val2 = enc2.read();

    if (oldTime + 10 < newTime) {
        ret.enc1 = (val1 - oldPos[0]) * 10 / (newTime - oldTime);
        ret.enc2 = (val2 - oldPos[0]) * 10 / (newTime - oldTime);

        oldPos[0] = val1;
        oldPos[0] = val2;

        if (ret.enc1 < 0)
            ret.enc1 = -(ret.enc1);

        if (ret.enc2 < 0)
            ret.enc2 = -(ret.enc2);

        oldTime = newTime;
    }
    
    return ret;
}

void loop()
{
	struct carData car;
    int i;
    int ms = millis();
    bool noRc = false;

    char buf1[16], buf2[16];

    car.ch = carFillChData();
    car.mot = carGetEncodersValue();

    // Box handler
    if (car.ch.box > 400) {
        car.mot.openBox = true;
    } else {
        car.mot.openBox = false;
    }

    // Throttle handler
    if (car.ch.throttle > 550) {
        car.ch.throttle -= 500;
        car.mot = computeSpeed(car.ch.throttle / 2, car.mot);
        car.mot.rev1 = false;
        car.mot.rev2 = false;
    } else if (car.ch.throttle < 450) {
        car.ch.throttle = 500 - car.ch.throttle;
        car.mot = computeSpeed(car.ch.throttle / 2, car.mot);
        car.mot.rev1 = true;
        car.mot.rev2 = true;
    } else {
        car.mot.spd1 = 0;
        car.mot.spd2 = 0;
        car.mot.rev1 = false;
        car.mot.rev2 = false;
    }

    // Rotation handler
    if (car.ch.rotation > 570) {
        car.mot.rev1 = false;
        car.mot.rev2 = true;
        if (car.mot.spd2 < 50)
            car.mot = computeSpeed(70, car.mot);
    } else if (car.ch.rotation < 430) {
        car.mot.rev1 = true;
        car.mot.rev2 = false;
        if (car.mot.spd1 < 50)
            car.mot = computeSpeed(70, car.mot);
    }

    if (car.ch.throttle > 2050) {
        noRc = true;
    }

    if (car.mot.openBox && !noRc) {
        boxServo.write(90);
    } else if (!noRc) {
        boxServo.write(0);
    }

    int temp;

    if (prevMs2 + 1000 < ms || ms < 2000) { // ms < 2000 for checking battery on start
        if (analogRead(batPort) < 400) {
            batLevel = 0;
            noBat = true;
        } else {
            batLevel = analogRead(batPort) - 420;
            noBat = false;
        }

        float fixBatLvl = batLevel / 10;
        batLevel = round(fixBatLvl) * 10;
        prevMs2 = millis();
    }

    if (prevMs + 100 < ms) {
        // Print to screen
        lcd.setCursor(0, 0);
        if (noBat) {
            lcd.print("x 0% \x80 ");
        } else {
            lcd.print(getBatteryIcon(batLevel));
            lcd.print(" ");
            lcd.print(batLevel);
            lcd.print("% \x80 ");
        }

        if (!noRc) {
            lcd.print(car.mot.spd1);
            lcd.print("/255     ");
        } else {
            lcd.print("no RC!");
        }


        lcd.setCursor(0, 1);
        if (batLevel < 10 && !noBat)
            lcd.print("CRITICAL BATT!  ");
        else if (batLevel < 30 && !noBat)
            lcd.print("Low battery.    ");
        else if (noBat)
            lcd.print("No battery.     ");
        else
            lcd.print("COEX Tech (c)   ");

        prevMs = millis();
    }

    if (!noBat && !noRc)
        carHardwareMotor(car.mot);
}
