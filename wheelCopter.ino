#include <EEPROM.h>
#include <GyverEncoder.h>
#include <IBusBM.h>
#include <LiquidCrystal.h>

enum lcd_pin {
  LCD_D7 = 22,
  LCD_D6,
  LCD_D5,
  LCD_D4,
  LCD_D3,
  LCD_D2,
  LCD_D1,
  LCD_D0,
  LCD_EN,
  LCD_RS,
};

enum encoder_pin {
  ENC_SW = 51,
  ENC_CLK,
  ENC_DT,
};

struct saved_settings {
  int32_t dz;
  int32_t lk;
};

struct rc_data {
  int32_t throttle;
  int32_t angle;
};

LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D0, LCD_D1, LCD_D2, LCD_D3, LCD_D4,
                  LCD_D5, LCD_D6, LCD_D7);
Encoder enc(ENC_DT, ENC_CLK, ENC_SW);

struct saved_settings eedata;
struct rc_data rcdata;

long rootEnc = 0, prevRootEnc = -1, prevMillis = 0;

int selectedItem = 0;
bool inItem = false;

IBusBM IBus;

long throttle = 1000;

void setup() {
  lcd.begin(16, 2);
  Serial.begin(9600);
  IBus.begin(Serial3);

  EEPROM.get(10, eedata);
}

bool inited = false;

void escInit() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("[Init ESC]");
  lcd.setCursor(0, 1);
  lcd.print("Please wait...");
  pinMode(12, OUTPUT);

  TCCR1A = _BV(COM1B1);
  TCCR1B = _BV(WGM13) | _BV(CS11);

  ICR1 = 2040;

  OCR1B = 1000;
  delay(1500);
  lcd.clear();

  inited = true;
}

bool curPrinted = false;

void loop() {
  enc.tick();
  /* if (enc.isRight()) {
    rootEnc++;
  } else if (enc.isLeft()) {
    rootEnc--;
  } else if (enc.isFastR()) {
    rootEnc += 10;
  } else if (enc.isFastL()) {
    rootEnc -= 10;
  }

  rootEnc = constrain(rootEnc, 1000, 2000);

  if (millis() >= prevMillis + 50) {
    lcd.setCursor(0, 0);
    lcd.print(rootEnc);
    lcd.print("    ");

    prevMillis = millis();
  }

  OCR1B = rootEnc; */

  // RC get
  if (eedata.lk == 0) {
    rcdata.throttle = IBus.readChannel(2) - 1000;
    rcdata.angle = IBus.readChannel(3) - 1000;

    // RC: apply dead zones
    rcdata.throttle = constrain(rcdata.throttle, 0, 1000);
    if (rcdata.throttle < eedata.dz) {
      rcdata.throttle = 0;
    }

    rcdata.angle = constrain(rcdata.angle, 0, 1000);
    if (rcdata.angle > 500 - eedata.dz && rcdata.angle < 500 + eedata.dz) {
      rcdata.angle = 500;
    }
  } else {
    rcdata.throttle = 1000;
    rcdata.angle = 1500;
  }

  if (inItem && selectedItem == 0) {
    if (enc.isHolded()) {
      inItem = false;
      OCR1B = 1000;
      throttle = 1000;
    }

    if (enc.isRight()) {
      throttle++;
    } else if (enc.isLeft()) {
      throttle--;
    } else if (enc.isFastR()) {
      throttle += 10;
    } else if (enc.isFastL()) {
      throttle -= 10;
    }

    throttle = constrain(throttle, 1000, 2000);
    OCR1B = throttle;
  }

  if (inItem && selectedItem == 2) {
    if (enc.isHolded()) {
      inItem = false;
      EEPROM.put(10, eedata);
    }

    if (enc.isRight()) {
      eedata.dz++;
    } else if (enc.isLeft()) {
      eedata.dz--;
    } else if (enc.isFastR()) {
      eedata.dz += 10;
    } else if (enc.isFastL()) {
      eedata.dz -= 10;
    }

    eedata.dz = constrain(eedata.dz, 0, 200);
  }

  if (inItem && selectedItem == 3) {
    if (enc.isHolded()) {
      inItem = false;
      EEPROM.put(10, eedata);
    }

    if (enc.isRight()) {
      eedata.lk = 1;
    } else if (enc.isLeft()) {
      eedata.lk = 0;
    }
  }

  if (inItem && selectedItem == 5) {
    if (enc.isHolded()) {
      inItem = false;
    }

    if (inited && !eedata.lk)
      OCR1B = rcdata.throttle + 1000;

    if (eedata.lk)
      OCR1B = 1000;
  }

  if (!inItem) {
    if (inited && !eedata.lk)
      OCR1B = rcdata.throttle + 1000;

    if (eedata.lk)
      OCR1B = 1000;

    if (enc.isLeft()) {
      rootEnc--;
    } else if (enc.isRight()) {
      rootEnc++;
    }

    rootEnc = constrain(rootEnc, 0, 5);
  }

  if (millis() > prevMillis + 100) {
    printMenu();
    prevMillis = millis();
  }
}

char getBatIcon() { return 'x'; }

int32_t prevMillisCursor = millis();

// WCMenu v1
void printMenu() {
  if (inItem) {
    switch (selectedItem) {
    case 0:
      if (!inited)
        escInit();

      lcd.setCursor(0, 0);
      lcd.print("[Hold: exit]    ");
      lcd.setCursor(0, 1);
      lcd.print("Throttle: ");
      lcd.print(throttle);
      lcd.print("    ");

      break;
    case 1:
      if (inited) {
        lcd.setCursor(0, 0);
        lcd.print("ESC already     ");
        lcd.setCursor(0, 1);
        lcd.print("inited. Error.   ");
        delay(2000);
        inItem = false;
        return;
      }

      lcd.setCursor(0, 0);
      lcd.print("[DO NOT EXIT]    ");
      lcd.setCursor(0, 1);
      lcd.print("Calibrating... ");

      pinMode(12, OUTPUT);

      TCCR1A = _BV(COM1B1);
      TCCR1B = _BV(WGM13) | _BV(CS11);

      ICR1 = 2040;

      OCR1B = 2000;
      delay(5000);
      OCR1B = 1000;
      delay(7000);

      inited = true;
      inItem = false;
      break;
    case 2:
      lcd.setCursor(0, 0);
      lcd.print("[Hold: save]         ");
      lcd.setCursor(0, 1);
      lcd.print("Value: ");
      lcd.print(eedata.dz);
      lcd.print("        ");
      break;
    case 3:
      lcd.setCursor(0, 0);
      lcd.print("[Hold: save]         ");
      lcd.setCursor(0, 1);
      lcd.print("Lock: ");
      lcd.print(eedata.lk ? "Enabled        " : "Disabled      ");
      break;
    case 4:
      escInit();
      inItem = false;
      break;
    case 5:
      lcd.setCursor(0, 0);
      lcd.print(rcdata.throttle);
      lcd.print(" | ");
      lcd.print(rcdata.angle);
      lcd.print("       ");
      lcd.setCursor(0, 1);
      lcd.print(getBatIcon());
      lcd.print("             ");
      break;

    default:
      inItem = false;
      break;
    }

    return;
  }

  if (rootEnc < 2) {
    lcd.setCursor(2, 0);
    lcd.print("ESC test         ");
    lcd.setCursor(2, 1);
    lcd.print("ESC calibrate    ");

    lcd.setCursor(0, rootEnc > 0);
    curPrinted ? lcd.print("  ") : lcd.print("\x13");
    lcd.setCursor(0, rootEnc == 0);
    lcd.print("  ");
  } else if (rootEnc < 4) {
    lcd.setCursor(2, 0);
    lcd.print("Dead zones           ");
    lcd.setCursor(2, 1);
    lcd.print("Lock RC        ");

    lcd.setCursor(0, rootEnc > 2);
    curPrinted ? lcd.print("  ") : lcd.print("\x13");
    lcd.setCursor(0, rootEnc == 2);
    lcd.print("  ");
  } else {
    lcd.setCursor(2, 0);
    lcd.print("ESC init           ");
    lcd.setCursor(2, 1);
    lcd.print("Monitor        ");

    lcd.setCursor(0, rootEnc > 4);
    curPrinted ? lcd.print("  ") : lcd.print("\x13");
    lcd.setCursor(0, rootEnc == 4);
    lcd.print("  ");
  }

  if (prevMillisCursor + 400 < millis()) {
    curPrinted = !curPrinted;
    prevMillisCursor = millis();
  }

  if (enc.isClick()) {
    inItem = true;
    selectedItem = rootEnc;
  }
}