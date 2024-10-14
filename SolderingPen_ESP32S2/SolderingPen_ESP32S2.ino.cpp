# 1 "C:\\Users\\benji\\AppData\\Local\\Temp\\tmpi_1ym_a1"
#include <Arduino.h>
# 1 "D:/Documents/Songguo-PTS200/SolderingPen_ESP32S2/SolderingPen_ESP32S2.ino"

#include "config.h"


#include <Button2.h>
#include <QC3Control.h>


#include "FirmwareMSC.h"
#include "Languages.h"
#include "USB.h"
#include "UtilsEEPROM.h"

QC3Control QC(14, 13);


#include <U8g2lib.h>

#include "PTS200_16.h"



#include <ESP32AnalogRead.h>

#include "esp_adc_cal.h"

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

#include <PID_v1.h>


#include <EEPROM.h>



#define LIS 






#include "SparkFun_LIS2DH12.h"
SPARKFUN_LIS2DH12 accel;





int16_t gx = 0, gy = 0, gz = 0;
uint16_t accels[32][3];
uint8_t accelIndex = 0;
#define ACCEL_SAMPLES 32


double aggKp = 11, aggKi = 0.5, aggKd = 1;
double consKp = 11, consKi = 3, consKd = 5;


uint16_t DefaultTemp = TEMP_DEFAULT;
uint16_t SleepTemp = TEMP_SLEEP;
uint8_t BoostTemp = TEMP_BOOST;
uint16_t time2sleep = TIME2SLEEP;
uint8_t time2off = TIME2OFF;
uint8_t timeOfBoost = TIMEOFBOOST;
uint8_t MainScrType = MAINSCREEN;
bool PIDenable = PID_ENABLE;
bool beepEnable = BEEP_ENABLE;
volatile uint8_t VoltageValue = VOLTAGE_VALUE;
bool QCEnable = QC_ENABLE;
uint8_t WAKEUPthreshold = WAKEUP_THRESHOLD;
bool restore_default_config = false;


uint16_t CalTemp[TIPMAX][4] = {TEMP200, TEMP280, TEMP360, TEMPCHP};
char TipName[TIPMAX][TIPNAMELENGTH] = {TIPNAME};
uint8_t CurrentTip = 0;
uint8_t NumberOfTips = 1;


volatile uint8_t a0, b0, c0, d0;
volatile bool ab0;
volatile int count, countMin, countMax, countStep;
volatile bool handleMoved;


uint16_t SetTemp, ShowTemp, gap, Step;
double Input, Output, Setpoint, RawTemp, CurrentTemp, ChipTemp;


uint16_t Vcc, Vin;


bool inLockMode = true;
bool inSleepMode = false;
bool inOffMode = false;
bool inBoostMode = false;
bool inCalibMode = false;
bool isWorky = true;
bool beepIfWorky = true;
bool TipIsPresent = true;
bool OledClear;


uint32_t sleepmillis;
uint32_t boostmillis;
uint32_t buttonmillis;
uint32_t goneMinutes;
uint32_t goneSeconds;
uint8_t SensorCounter = 0;



PID ctrl(&Input, &Output, &Setpoint, aggKp, aggKi, aggKd, REVERSE);



#if defined(SSD1306)
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE,
                                                    22, 21);
#elif defined(SH1107)
U8G2_SH1107_64X128_F_HW_I2C u8g2(U8G2_R1, 7);
#else
#error Wrong OLED controller type!
#endif


char F_Buffer[20];

float lastSENSORTmp = 0;
float newSENSORTmp = 0;
uint8_t SENSORTmpTime = 0;


uint16_t vref_adc0, vref_adc1;
ESP32AnalogRead adc_sensor;
ESP32AnalogRead adc_vin;


uint8_t language = 0;


uint8_t hand_side = 0;


FirmwareMSC MSC_Update;
bool MSC_Updating_Flag = false;


Button2 btn;

float limit = 0.0;
void setup();
void loop();
void ROTARYCheck();
void SLEEPCheck();
void SENSORCheck();
void calculateTemp();
void Thermostat();
void beep(uint32_t us, uint16_t frequency);
void setRotary(int rmin, int rmax, int rstep, int rvalue);
int getRotary();
void getEEPROM();
void updateEEPROM();
void MainScreen();
void SetupScreen();
void TipScreen();
void TempScreen();
void TimerScreen();
uint8_t MenuScreen(const char *Items[][language_types], uint8_t numberOfItems,
                   uint8_t selected);
void MessageScreen(const char *Items[][language_types], uint8_t numberOfItems);
uint16_t InputScreen(const char *Items[][language_types]);
void InfoScreen();
void ChangeTipScreen();
void CalibrationScreen();
void InputNameScreen();
void DeleteTipScreen();
void AddTipScreen();
uint16_t denoiseAnalog(byte port);
double getChipTemp();
float getMPUTemp();
uint16_t getVIN();
int32_t variance(int16_t a[]);
void Button_loop();
void PD_Update();
static void usbEventCallback(void *arg, esp_event_base_t event_base,
                             int32_t event_id, void *event_data);
void turnOffHeater(Button2 &b);
void heatWithLimit();
#line 159 "D:/Documents/Songguo-PTS200/SolderingPen_ESP32S2/SolderingPen_ESP32S2.ino"
void setup() {
  digitalWrite(PD_CFG_0, LOW);
  digitalWrite(PD_CFG_1, HIGH);
  digitalWrite(PD_CFG_2, LOW);




  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);





  adc_sensor.attach(SENSOR_PIN);
  adc_vin.attach(VIN_PIN);
# 184 "D:/Documents/Songguo-PTS200/SolderingPen_ESP32S2/SolderingPen_ESP32S2.ino"
  pinMode(SENSOR_PIN, INPUT_PULLUP);

  pinMode(BUZZER_PIN, OUTPUT);

  pinMode(BUTTON_P_PIN, INPUT_PULLUP);
  pinMode(BUTTON_N_PIN, INPUT_PULLUP);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
# 201 "D:/Documents/Songguo-PTS200/SolderingPen_ESP32S2/SolderingPen_ESP32S2.ino"
  init_EEPROM();
  if (digitalRead(BUTTON_P_PIN) == LOW && digitalRead(BUTTON_N_PIN) == LOW &&
      digitalRead(BUTTON_PIN) == HIGH) {
    write_default_EEPROM();
  }
  getEEPROM();

  pinMode(PD_CFG_0, OUTPUT);
  pinMode(PD_CFG_1, OUTPUT);
  pinMode(PD_CFG_2, OUTPUT);

  if (QCEnable) {
    QC.begin();
    delay(100);
    switch (VoltageValue) {
      case 0: {
        QC.set9V();
      } break;
      case 1: {
        QC.set12V();
      } break;
      case 2: {
        QC.set12V();
      } break;
      case 3: {
        QC.set20V();
      } break;
      case 4: {
        QC.set20V();
      } break;
      default:
        break;
    }
  }

  PD_Update();


  delay(100);
  Vin = getVIN();


  SetTemp = DefaultTemp;
  RawTemp = denoiseAnalog(SENSOR_PIN);

  calculateTemp();



  limit = POWER_LIMIT_20;
  if (VoltageValue < 3) {
    limit = POWER_LIMIT_15;
  }
  if (((CurrentTemp + 20) < DefaultTemp) && !inLockMode)
    ledcWrite(CONTROL_CHANNEL, constrain(HEATER_ON, 0, limit));



  ctrl.SetOutputLimits(0, 255);
  ctrl.SetMode(AUTOMATIC);



  a0 = 0;
  b0 = 0;
  setRotary(TEMP_MIN, TEMP_MAX, TEMP_STEP, DefaultTemp);


  sleepmillis = millis();


  beep(500000, 880);

  Serial.println("Soldering Pen");


  Wire.begin();
  Wire.setClock(100000);
  if (accel.begin() == false) {
    delay(500);
    Serial.println("Accelerometer not detected.");
  }






  ChipTemp = getChipTemp();
  lastSENSORTmp = getMPUTemp();
  u8g2.initDisplay();
  u8g2.begin();
  u8g2.sendF("ca", 0xa8, 0x3f);
  u8g2.enableUTF8Print();
  if(hand_side){
    u8g2.setDisplayRotation(U8G2_R3);
  }else{
    u8g2.setDisplayRotation(U8G2_R1);
  }





}

int SENSORCheckTimes = 0;
long lastMillis = 0;

void loop() {
  long timems = millis();
  ROTARYCheck();

  SLEEPCheck();


  if (SENSORCheckTimes > 1) {

    SENSORCheck();



    SENSORCheckTimes = 0;
  }
  SENSORCheckTimes++;

  Thermostat();
  MainScreen();
  lastMillis = millis() - timems;
  Serial.println(lastMillis);
}



void ROTARYCheck() {


  SetTemp = getRotary();

  uint8_t c = digitalRead(BUTTON_PIN);
  if (!c && c0) {
    delay(10);
    if (digitalRead(BUTTON_PIN) == c) {
      beep(100, 880);
      buttonmillis = millis();
      delay(10);
      while ((!digitalRead(BUTTON_PIN)) && ((millis() - buttonmillis) < 500))
        ;
      delay(10);
      if ((millis() - buttonmillis) >= 500) {
        SetupScreen();
      } else {
        if (inLockMode) {
          inLockMode = false;
        } else {
          buttonmillis = millis();
          while ((digitalRead(BUTTON_PIN)) && ((millis() - buttonmillis) < 200))
            delay(10);
          if ((millis() - buttonmillis) >= 200) {
            if (inOffMode) {
              inOffMode = false;
            } else {
              inBoostMode = !inBoostMode;
              if (inBoostMode) {
                boostmillis = millis();
              }
              handleMoved = true;
            }
          } else {
            inOffMode = true;
          }
        }
      }
    }
  }
  c0 = c;


  if (inBoostMode && timeOfBoost) {
    goneSeconds = (millis() - boostmillis) / 1000;
    if (goneSeconds >= timeOfBoost) {
      inBoostMode = false;
      beep(100, 880);
      beepIfWorky = true;

    }
  }
}


void SLEEPCheck() {
  if (inLockMode) {
    ;
  } else {
    if (handleMoved) {
      u8g2.setPowerSave(0);
      if (inSleepMode) {
        limit = POWER_LIMIT_20;
        if (VoltageValue < 3) {
          limit = POWER_LIMIT_15;
        }
        if ((CurrentTemp + 20) <
            SetTemp)
          ledcWrite(
              CONTROL_CHANNEL,
              constrain(HEATER_ON, 0, limit));

        beep(100, 880);
        delayMicroseconds(100);
        beep(100, 880);
        delayMicroseconds(100);
        beepIfWorky = true;

      }
      handleMoved = false;
      inSleepMode = false;

      sleepmillis = millis();
    }


    goneSeconds = (millis() - sleepmillis) / 1000;
    if ((!inSleepMode) && (time2sleep > 0) && (goneSeconds >= time2sleep)) {
      inSleepMode = true;
      beep(100, 880);
    } else if ((!inOffMode) && (time2off > 0) &&
               ((goneSeconds / 60) >= time2off)) {
      inOffMode = true;
      u8g2.setPowerSave(1);
      beep(100, 880);
    }
  }
}



void SENSORCheck() {
# 464 "D:/Documents/Songguo-PTS200/SolderingPen_ESP32S2/SolderingPen_ESP32S2.ino"
  if (accel.available()) {
    accels[accelIndex][0] = accel.getRawX() + 32768;
    accels[accelIndex][1] = accel.getRawY() + 32768;
    accels[accelIndex][2] = accel.getRawZ() + 32768;
    accelIndex++;
# 478 "D:/Documents/Songguo-PTS200/SolderingPen_ESP32S2/SolderingPen_ESP32S2.ino"
    if (accelIndex >= ACCEL_SAMPLES) {
      accelIndex = 0;

      uint64_t avg[3] = {0, 0, 0};
      for (int i = 0; i < ACCEL_SAMPLES; i++) {
        avg[0] += accels[i][0];
        avg[1] += accels[i][1];
        avg[2] += accels[i][2];
      }
      avg[0] /= ACCEL_SAMPLES;
      avg[1] /= ACCEL_SAMPLES;
      avg[2] /= ACCEL_SAMPLES;
      uint64_t var[3] = {0, 0, 0};
      for (int i = 0; i < ACCEL_SAMPLES; i++) {
        var[0] += (accels[i][0] - avg[0]) * (accels[i][0] - avg[0]);
        var[1] += (accels[i][1] - avg[1]) * (accels[i][1] - avg[1]);
        var[2] += (accels[i][2] - avg[2]) * (accels[i][2] - avg[2]);
      }
      var[0] /= ACCEL_SAMPLES;
      var[1] /= ACCEL_SAMPLES;
      var[2] /= ACCEL_SAMPLES;
# 507 "D:/Documents/Songguo-PTS200/SolderingPen_ESP32S2/SolderingPen_ESP32S2.ino"
      int varThreshold = WAKEUPthreshold * 10000;

      if (var[0] > varThreshold || var[1] > varThreshold ||
          var[2] > varThreshold) {
        handleMoved = true;

      }
    }
  }



  ledcWrite(CONTROL_CHANNEL,
            HEATER_OFF);

  if (VoltageValue == 3) {
    delayMicroseconds(TIME2SETTLE_20V);
  } else {
    delayMicroseconds(TIME2SETTLE);
  }
  long timems = millis();
  double temp = denoiseAnalog(SENSOR_PIN);
  lastMillis = millis() - timems;


  if (SensorCounter++ > 10) {
    Vin = getVIN();
    SensorCounter = 0;
  }

  if (!inLockMode) {
    limit = POWER_LIMIT_20;
    if (VoltageValue < 3) {
      limit = POWER_LIMIT_15;
    }
    ledcWrite(CONTROL_CHANNEL,
              constrain(HEATER_PWM, 0,
                        limit));
  }

  RawTemp += (temp - RawTemp) *
             SMOOTHIE;
  calculateTemp();



  if ((ShowTemp != Setpoint) || (abs(ShowTemp - CurrentTemp) > 5))
    ShowTemp = CurrentTemp;
  if (abs(ShowTemp - Setpoint) <= 1) ShowTemp = Setpoint;
  if (inLockMode) {
    ShowTemp = 0;
  }




  gap = abs(SetTemp - CurrentTemp);
  if (gap < 5) {
    if (!isWorky && beepIfWorky) beep(100, 932);
    isWorky = true;
    beepIfWorky = false;
  } else
    isWorky = false;



  if (ShowTemp > 500) TipIsPresent = false;
  if (!TipIsPresent &&
      (ShowTemp < 500)) {
    ledcWrite(CONTROL_CHANNEL, HEATER_OFF);
    beep(100, 880);
    TipIsPresent = true;
    ChangeTipScreen();
    updateEEPROM();
    handleMoved = true;
    RawTemp = denoiseAnalog(
        SENSOR_PIN);
    c0 = LOW;
    setRotary(TEMP_MIN, TEMP_MAX, TEMP_STEP,
              SetTemp);
  }
}



void calculateTemp() {
  if (RawTemp < 200)
    CurrentTemp = map(RawTemp, 0, 200, 15, CalTemp[CurrentTip][0]);
  else if (RawTemp < 280)
    CurrentTemp =
        map(RawTemp, 200, 280, CalTemp[CurrentTip][0], CalTemp[CurrentTip][1]);
  else
    CurrentTemp =
        map(RawTemp, 280, 360, CalTemp[CurrentTip][1], CalTemp[CurrentTip][2]);
}


void Thermostat() {


  if (inOffMode || inLockMode)
    Setpoint = 0;
  else if (inSleepMode)
    Setpoint = SleepTemp;
  else if (inBoostMode) {
    Setpoint = constrain(SetTemp + BoostTemp, 0, 450);
  } else
    Setpoint = SetTemp;

  if (SetTemp != DefaultTemp) {
    DefaultTemp = SetTemp;
    update_default_temp_EEPROM();
  }


  gap = abs(Setpoint - CurrentTemp);
  if (PIDenable) {
    Input = CurrentTemp;
    if (gap < 30)
      ctrl.SetTunings(consKp, consKi, consKd);
    else
      ctrl.SetTunings(aggKp, aggKi, aggKd);
    ctrl.Compute();
  } else {


    if ((CurrentTemp + 0.5) < Setpoint)
      Output = 0;
    else
      Output = 255;
  }
  limit = POWER_LIMIT_20;
  if (VoltageValue < 3) {
    limit = POWER_LIMIT_15;
  } else if(VoltageValue == 3){
    limit = POWER_LIMIT_20_2;
  }
  ledcWrite(CONTROL_CHANNEL,
            constrain((HEATER_PWM), 0, limit));
}


void beep(uint32_t us, uint16_t frequency) {
  if (beepEnable) {
    uint32_t period = 1000000 / ((uint32_t)frequency * 2);
    for (uint32_t t = 0; t < us; t += period * 2) {
      digitalWrite(BUZZER_PIN, HIGH);
      delayMicroseconds(period);
      digitalWrite(BUZZER_PIN, LOW);
      delayMicroseconds(period);
    }
  }
}


void setRotary(int rmin, int rmax, int rstep, int rvalue) {
  countMin = rmin << ROTARY_TYPE;
  countMax = rmax << ROTARY_TYPE;
  countStep = rstep;
  count = rvalue << ROTARY_TYPE;
}


int getRotary() {
  Button_loop();
  return (count >> ROTARY_TYPE);
}



void getEEPROM() { read_EEPROM(); }



void updateEEPROM() { update_EEPROM(); }


void MainScreen() {
  u8g2.firstPage();
  switch(MainScrType) {
  case 2:
    do {
      float fVin = (float)Vin / 1000;
      newSENSORTmp = newSENSORTmp + 0.01 * getMPUTemp();
      SENSORTmpTime++;
      if (SENSORTmpTime >= 100) {
        lastSENSORTmp = newSENSORTmp;
        newSENSORTmp = 0;
        SENSORTmpTime = 0;
      }


      u8g2.setFont(u8g2_font_unifont_t_chinese3);
      u8g2.setFontPosTop();


      u8g2.drawUTF8(0, 0 + SCREEN_OFFSET, "Tmcu         C");
      u8g2.drawUTF8(0, 16 + SCREEN_OFFSET, "Ttip         C");
      u8g2.drawUTF8(0, 32 + SCREEN_OFFSET, "             C");
      u8g2.drawUTF8(0, 48 + SCREEN_OFFSET, "Vbus         V");


      u8g2.setFont(language > 1 ? u8g2_font_unifont_t_chinese3 : PTS200_16);
      char *which_str = (char*)txt_hold[language];
      if (ShowTemp > 500) {
        which_str = (char*)txt_error[language];
      }
      else if (inOffMode || inLockMode) {
        which_str = (char*)txt_off[language];
      }
      else if (inSleepMode) {
        which_str = (char*)txt_sleep[language];
      }
      else if (inBoostMode) {
        which_str = (char*)txt_boost[language];
      }
      else if (isWorky) {
        which_str = (char*)txt_worky[language];
      }
      else if (Output < 180) {
        which_str = (char*)txt_on[language];
      }
      u8g2.drawUTF8(0, 32 + SCREEN_OFFSET, which_str);

      u8g2.setFont(u8g2_font_unifont_t_chinese3);
      u8g2.setCursor(56, 0 + SCREEN_OFFSET);
      u8g2.print(lastSENSORTmp, 2);
      u8g2.setCursor(56, 16 + SCREEN_OFFSET);
      u8g2.print(ShowTemp, 0);
      u8g2.setCursor(56, 32 + SCREEN_OFFSET);
      u8g2.print(Setpoint, 0);
      u8g2.setCursor(56, 48 + SCREEN_OFFSET);
      u8g2.print(fVin, 2);

    } while(u8g2.nextPage());
    break;

  default:
    do {



      u8g2.setFont(PTS200_16);
      if(language == 2){
        u8g2.setFont(u8g2_font_unifont_t_chinese3);
      }
      u8g2.setFontPosTop();

      u8g2.drawUTF8(0, 0 + SCREEN_OFFSET, txt_set_temp[language]);
      u8g2.setCursor(40, 0 + SCREEN_OFFSET);
      u8g2.setFont(u8g2_font_unifont_t_chinese3);
      u8g2.print(Setpoint, 0);

      u8g2.setFont(PTS200_16);
      if(language == 2){
        u8g2.setFont(u8g2_font_unifont_t_chinese3);
      }

      u8g2.setCursor(96, 0 + SCREEN_OFFSET);
      if (ShowTemp > 500)
        u8g2.print(txt_error[language]);
      else if (inOffMode || inLockMode)
        u8g2.print(txt_off[language]);
      else if (inSleepMode)
        u8g2.print(txt_sleep[language]);
      else if (inBoostMode)
        u8g2.print(txt_boost[language]);
      else if (isWorky)
        u8g2.print(txt_worky[language]);
      else if (Output < 180)
        u8g2.print(txt_on[language]);
      else
        u8g2.print(txt_hold[language]);

      u8g2.setFont(u8g2_font_unifont_t_chinese3);

      if (MainScrType) {

        float fVin = (float)Vin / 1000;
        newSENSORTmp = newSENSORTmp + 0.01 * getMPUTemp();
        SENSORTmpTime++;
        if (SENSORTmpTime >= 100) {
          lastSENSORTmp = newSENSORTmp;
          newSENSORTmp = 0;
          SENSORTmpTime = 0;
        }
        u8g2.setCursor(0, 50);
        u8g2.print(lastSENSORTmp, 1);
        u8g2.print(F("C"));
        u8g2.setCursor(83, 50);
        u8g2.print(fVin, 1);
        u8g2.print(F("V"));

        u8g2.setFont(u8g2_font_freedoomr25_tn);
        u8g2.setFontPosTop();
        u8g2.setCursor(37, 18);
        if (ShowTemp > 500)
          u8g2.print(F("000"));
        else
          u8g2.printf("%03d", ShowTemp);
      } else {

        u8g2.setFont(u8g2_font_fub42_tn);
        u8g2.setFontPosTop();
        u8g2.setCursor(15, 20);
        if (ShowTemp > 500)
          u8g2.print(F("000"));
        else
          u8g2.printf("%03d", ShowTemp);
      }
    } while (u8g2.nextPage());
    break;
  }
}


void SetupScreen() {
  ledcWrite(CONTROL_CHANNEL, HEATER_OFF);
  beep(100000, 880);
  uint16_t SaveSetTemp = SetTemp;
  uint8_t selection = 0;
  bool repeat = true;

  while (repeat) {
    selection = MenuScreen(SetupItems, sizeof(SetupItems), selection);
    switch (selection) {
      case 0: {
        TipScreen();
        repeat = false;
      } break;
      case 1: {
        TempScreen();
      } break;
      case 2: {
        TimerScreen();
      } break;



      case 3: {
        MainScrType =
            MenuScreen(MainScreenItems, sizeof(MainScreenItems), MainScrType);
      } break;
      case 4: {
        InfoScreen();
      } break;
      case 5:
        VoltageValue =
            MenuScreen(VoltageItems, sizeof(VoltageItems), VoltageValue);
        PD_Update();
        break;
      case 6:
        QCEnable = MenuScreen(QCItems, sizeof(QCItems), QCEnable);
        break;
      case 7:
        beepEnable = MenuScreen(BuzzerItems, sizeof(BuzzerItems), beepEnable);
        break;
      case 8: {
        restore_default_config = MenuScreen(DefaultItems, sizeof(DefaultItems),
                                            restore_default_config);
        if (restore_default_config) {
          restore_default_config = false;
          write_default_EEPROM();
          read_EEPROM();
        }
      } break;
      case 9: {
        bool lastbutton = (!digitalRead(BUTTON_PIN));
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_ncenB08_tr);
        u8g2.drawStr(0, 10,
                     "MSC Update");
        u8g2.sendBuffer();
        delay(1000);
        do {
          MSC_Update.onEvent(usbEventCallback);
          MSC_Update.begin();
          if (lastbutton && digitalRead(BUTTON_PIN)) {
            delay(10);
            lastbutton = false;
          }
        } while (digitalRead(BUTTON_PIN) || lastbutton);

        MSC_Update.end();
      } break;
      case 10: {
        Serial.println(language);
        language = MenuScreen(LanguagesItems, sizeof(LanguagesItems), language);
        Serial.println(language);
        repeat = false;
      } break;
      case 11: {
        if(hand_side == 0){
          u8g2.setDisplayRotation(U8G2_R3);
          hand_side = 1;
        }else{
          u8g2.setDisplayRotation(U8G2_R1);
          hand_side = 0;
        }
        repeat = false;
      } break;
      default:
        repeat = false;
        break;
    }
  }
  updateEEPROM();
  handleMoved = true;
  SetTemp = SaveSetTemp;
  setRotary(TEMP_MIN, TEMP_MAX, TEMP_STEP, SetTemp);
}


void TipScreen() {
  uint8_t selection = 0;
  bool repeat = true;
  while (repeat) {
    selection = MenuScreen(TipItems, sizeof(TipItems), selection);
    switch (selection) {
      case 0:
        ChangeTipScreen();
        break;
      case 1:
        CalibrationScreen();
        break;
      case 2:
        InputNameScreen();
        break;
      case 3:
        DeleteTipScreen();
        break;
      case 4:
        AddTipScreen();
        break;
      default:
        repeat = false;
        break;
    }
  }
}


void TempScreen() {
  uint8_t selection = 0;
  bool repeat = true;
  while (repeat) {
    selection = MenuScreen(TempItems, sizeof(TempItems), selection);
    switch (selection) {
      case 0:
        setRotary(TEMP_MIN, TEMP_MAX, TEMP_STEP, DefaultTemp);
        DefaultTemp = InputScreen(DefaultTempItems);
        break;
      case 1:
        setRotary(50, TEMP_MAX, TEMP_STEP, SleepTemp);
        SleepTemp = InputScreen(SleepTempItems);
        break;
      case 2:
        setRotary(10, 100, TEMP_STEP, BoostTemp);
        BoostTemp = InputScreen(BoostTempItems);
        break;
      default:
        repeat = false;
        break;
    }
  }
}


void TimerScreen() {
  uint8_t selection = 0;
  bool repeat = true;
  while (repeat) {
    selection = MenuScreen(TimerItems, sizeof(TimerItems), selection);
    switch (selection) {
      case 0:
        setRotary(0, 600, 10, time2sleep);
        time2sleep = InputScreen(SleepTimerItems);
        break;
      case 1:
        setRotary(0, 60, 1, time2off);
        time2off = InputScreen(OffTimerItems);
        break;
      case 2:
        setRotary(0, 180, 10, timeOfBoost);
        timeOfBoost = InputScreen(BoostTimerItems);
        break;
      case 3:
        setRotary(0, 50, 5, WAKEUPthreshold);
        WAKEUPthreshold = InputScreen(WAKEUPthresholdItems);
        break;
      default:
        repeat = false;
        break;
    }
  }
}


uint8_t MenuScreen(const char *Items[][language_types], uint8_t numberOfItems,
                   uint8_t selected) {

  bool isTipScreen = ((strcmp(Items[0][language], "烙铁头:") == 0) ||
                      (strcmp(Items[0][language], "Tip:") == 0) ||
                      (strcmp(Items[0][language], "烙鐵頭:") == 0));
  uint8_t lastselected = selected;
  int8_t arrow = 0;
  if (selected) arrow = 1;
  numberOfItems = numberOfItems / language_types;
  numberOfItems >>= 2;


#if defined(SSD1306)
  setRotary(0, numberOfItems + 3, 1, selected);
#elif defined(SH1107)
  setRotary(0, numberOfItems - 2, 1, selected);
#else
#error Wrong OLED controller type!
#endif

  bool lastbutton = (!digitalRead(BUTTON_PIN));

  do {
    selected = getRotary();
    arrow = constrain(arrow + selected - lastselected, 0, 2);
    lastselected = selected;
    u8g2.firstPage();
    do {
      u8g2.setFont(PTS200_16);
    if(language == 2){
      u8g2.setFont(u8g2_font_unifont_t_chinese3);
    }
      u8g2.setFontPosTop();
      u8g2.drawUTF8(0, 0 + SCREEN_OFFSET, Items[0][language]);
      if (isTipScreen)
        u8g2.drawUTF8(54, 0 + SCREEN_OFFSET, TipName[CurrentTip]);
      u8g2.drawUTF8(0, 16 * (arrow + 1) + SCREEN_OFFSET, ">");
      for (uint8_t i = 0; i < 3; i++) {
        uint8_t drawnumber = selected + i + 1 - arrow;
        if (drawnumber < numberOfItems)
          u8g2.drawUTF8(12, 16 * (i + 1) + SCREEN_OFFSET,
                        Items[selected + i + 1 - arrow][language]);
      }
    } while (u8g2.nextPage());
    if (lastbutton && digitalRead(BUTTON_PIN)) {
      delay(10);
      lastbutton = false;
    }
  } while (digitalRead(BUTTON_PIN) || lastbutton);

  beep(100000, 880);
  return selected;
}

void MessageScreen(const char *Items[][language_types], uint8_t numberOfItems) {
  numberOfItems = numberOfItems / language_types;
  bool lastbutton = (!digitalRead(BUTTON_PIN));
  u8g2.firstPage();
  do {
    u8g2.setFont(PTS200_16);
        if(language == 2){
      u8g2.setFont(u8g2_font_unifont_t_chinese3);
    }
    u8g2.setFontPosTop();
    for (uint8_t i = 0; i < numberOfItems; i++)
      u8g2.drawUTF8(0, i * 16, Items[i][language]);
  } while (u8g2.nextPage());
  do {
    if (lastbutton && digitalRead(BUTTON_PIN)) {
      delay(10);
      lastbutton = false;
    }
  } while (digitalRead(BUTTON_PIN) || lastbutton);
  beep(100000, 880);
}


uint16_t InputScreen(const char *Items[][language_types]) {
  uint16_t value;
  bool lastbutton = (!digitalRead(BUTTON_PIN));

  do {
    value = getRotary();
    u8g2.firstPage();
    do {
      u8g2.setFont(PTS200_16);
          if(language == 2){
      u8g2.setFont(u8g2_font_unifont_t_chinese3);
    }
      u8g2.setFontPosTop();
      u8g2.drawUTF8(0, 0 + SCREEN_OFFSET, Items[0][language]);
      u8g2.setCursor(0, 32);
      u8g2.print(">");
      u8g2.setCursor(10, 32);
      if (value == 0)
        u8g2.print(txt_Deactivated[language]);
      else {
        u8g2.print(value);
        u8g2.print(" ");
        u8g2.print(Items[1][language]);
      }
    } while (u8g2.nextPage());
    if (lastbutton && digitalRead(BUTTON_PIN)) {
      delay(10);
      lastbutton = false;
    }
  } while (digitalRead(BUTTON_PIN) || lastbutton);

  beep(100000, 880);
  return value;
}


void InfoScreen() {
  bool lastbutton = (!digitalRead(BUTTON_PIN));

  do {
    Vin = getVIN();
    float fVin = (float)Vin / 1000;
    float fTmp = getChipTemp();
    u8g2.firstPage();
    do {
      u8g2.setFont(PTS200_16);
          if(language == 2){
      u8g2.setFont(u8g2_font_unifont_t_chinese3);
    }
      u8g2.setFontPosTop();
      u8g2.setCursor(0, 0 + SCREEN_OFFSET);
      u8g2.print(txt_temp[language]);
      u8g2.print(fTmp, 1);
      u8g2.print(F(" C"));
      u8g2.setCursor(0, 16 + SCREEN_OFFSET);
      u8g2.print(txt_voltage[language]);
      u8g2.print(fVin, 1);
      u8g2.print(F(" V"));
      u8g2.setCursor(0, 16 * 2 + SCREEN_OFFSET);
      u8g2.print(txt_Version[language]);
      u8g2.print(VERSION);


    } while (u8g2.nextPage());
    if (lastbutton && digitalRead(BUTTON_PIN)) {
      delay(10);
      lastbutton = false;
    }
  } while (digitalRead(BUTTON_PIN) || lastbutton);

  beep(100000, 880);
}


void ChangeTipScreen() {
  uint8_t selected = CurrentTip;
  uint8_t lastselected = selected;
  int8_t arrow = 0;
  if (selected) arrow = 1;
  setRotary(0, NumberOfTips - 1, 1, selected);
  bool lastbutton = (!digitalRead(BUTTON_PIN));

  Serial.print("selected: ");
  Serial.println(selected);
  Serial.print("lastselected: \n");
  Serial.println(lastselected);
  Serial.print("NumberOfTips: \n");
  Serial.println(NumberOfTips);

  do {
    selected = getRotary();
    arrow = constrain(arrow + selected - lastselected, 0, 2);
    lastselected = selected;
    u8g2.firstPage();
    do {
      u8g2.setFont(PTS200_16);
          if(language == 2){
      u8g2.setFont(u8g2_font_unifont_t_chinese3);
    }
      u8g2.setFontPosTop();

      u8g2.drawUTF8(0, 0 + SCREEN_OFFSET, txt_select_tip[language]);
      u8g2.drawUTF8(0, 16 * (arrow + 1) + SCREEN_OFFSET, ">");
      for (uint8_t i = 0; i < 3; i++) {
        uint8_t drawnumber = selected + i - arrow;
        if (drawnumber < NumberOfTips)
          u8g2.drawUTF8(12, 16 * (i + 1) + SCREEN_OFFSET,
                        TipName[selected + i - arrow]);
      }
    } while (u8g2.nextPage());
    if (lastbutton && digitalRead(BUTTON_PIN)) {
      delay(10);
      lastbutton = false;
    }
  } while (digitalRead(BUTTON_PIN) || lastbutton);

  beep(100000, 880);
  CurrentTip = selected;
}


void CalibrationScreen() {
  uint16_t CalTempNew[4];
  uint16_t tempSetTemp = SetTemp;
  for (uint8_t CalStep = 0; CalStep < 3; CalStep++) {
    SetTemp = CalTemp[CurrentTip][CalStep];
    Serial.print("SetTemp: ");
    Serial.println(SetTemp);
    setRotary(100, 500, 1, SetTemp);
    beepIfWorky = true;
    bool lastbutton = (!digitalRead(BUTTON_PIN));

    do {
      SENSORCheck();

      Thermostat();

      u8g2.firstPage();
      do {
        u8g2.setFont(PTS200_16);
            if(language == 2){
      u8g2.setFont(u8g2_font_unifont_t_chinese3);
    }
        u8g2.setFontPosTop();

        u8g2.drawUTF8(0, 0 + SCREEN_OFFSET, txt_calibrate[language]);
        u8g2.setCursor(0, 16 + SCREEN_OFFSET);
        u8g2.print(txt_step[language]);
        u8g2.print(CalStep + 1);
        u8g2.print(" of 3");
        if (isWorky) {
          u8g2.setCursor(0, 32 + SCREEN_OFFSET);
          u8g2.print(txt_set_measured[language]);
          u8g2.setCursor(0, 48 + SCREEN_OFFSET);
          u8g2.print(txt_s_temp[language]);
          u8g2.print(getRotary());
        } else {
          u8g2.setCursor(0, 32 + SCREEN_OFFSET);
          u8g2.print(txt_temp_2[language]);
          u8g2.print(uint16_t(RawTemp));
          u8g2.setCursor(0, 48 + SCREEN_OFFSET);
          u8g2.print(txt_wait_pls[language]);
        }
      } while (u8g2.nextPage());
      if (lastbutton && digitalRead(BUTTON_PIN)) {
        delay(10);
        lastbutton = false;
      }
    } while (digitalRead(BUTTON_PIN) || lastbutton);

    CalTempNew[CalStep] = getRotary();
    beep(100000, 880);
    delay(10);
  }

  ledcWrite(CONTROL_CHANNEL, HEATER_OFF);
  if (VoltageValue == 3) {
    delayMicroseconds(TIME2SETTLE_20V);
  } else {
    delayMicroseconds(TIME2SETTLE);
  }
  CalTempNew[3] = getChipTemp();
  if ((CalTempNew[0] + 10 < CalTempNew[1]) &&
      (CalTempNew[1] + 10 < CalTempNew[2])) {
    if (MenuScreen(StoreItems, sizeof(StoreItems), 0)) {
      for (uint8_t i = 0; i < 4; i++) CalTemp[CurrentTip][i] = CalTempNew[i];
    }
  }

  SetTemp = tempSetTemp;
  update_EEPROM();
}


void InputNameScreen() {
  uint8_t value;

  for (uint8_t digit = 0; digit < (TIPNAMELENGTH - 1); digit++) {
    bool lastbutton = (!digitalRead(BUTTON_PIN));
    setRotary(31, 96, 1, 65);
    do {
      value = getRotary();
      if (value == 31) {
        value = 95;
        setRotary(31, 96, 1, 95);
      }
      if (value == 96) {
        value = 32;
        setRotary(31, 96, 1, 32);
      }
      u8g2.firstPage();
      do {
        u8g2.setFont(PTS200_16);
            if(language == 2){
      u8g2.setFont(u8g2_font_unifont_t_chinese3);
    }
        u8g2.setFontPosTop();
        u8g2.drawUTF8(0, 0 + SCREEN_OFFSET, txt_enter_tip_name[language]);
        u8g2.setCursor(12 * digit, 48 + SCREEN_OFFSET);
        u8g2.print(char(94));
        u8g2.setCursor(0, 32 + SCREEN_OFFSET);
        for (uint8_t i = 0; i < digit; i++) u8g2.print(TipName[CurrentTip][i]);
        u8g2.setCursor(12 * digit, 32 + SCREEN_OFFSET);
        u8g2.print(char(value));
      } while (u8g2.nextPage());
      if (lastbutton && digitalRead(BUTTON_PIN)) {
        delay(10);
        lastbutton = false;
      }
    } while (digitalRead(BUTTON_PIN) || lastbutton);
    TipName[CurrentTip][digit] = value;
    beep(100000, 880);
    delay(10);
  }
  TipName[CurrentTip][TIPNAMELENGTH - 1] = 0;
  return;
}


void DeleteTipScreen() {
  if (NumberOfTips == 1) {
    MessageScreen(DeleteMessage, sizeof(DeleteMessage));
  } else if (MenuScreen(SureItems, sizeof(SureItems), 0)) {
    if (CurrentTip == (NumberOfTips - 1)) {
      CurrentTip--;
    } else {
      for (uint8_t i = CurrentTip; i < (NumberOfTips - 1); i++) {
        for (uint8_t j = 0; j < TIPNAMELENGTH; j++)
          TipName[i][j] = TipName[i + 1][j];
        for (uint8_t j = 0; j < 4; j++) CalTemp[i][j] = CalTemp[i + 1][j];
      }
    }
    NumberOfTips--;
  }
}


void AddTipScreen() {
  if (NumberOfTips < TIPMAX) {
    CurrentTip = NumberOfTips++;
    InputNameScreen();
    CalTemp[CurrentTip][0] = TEMP200;
    CalTemp[CurrentTip][1] = TEMP280;
    CalTemp[CurrentTip][2] = TEMP360;
    CalTemp[CurrentTip][3] = TEMPCHP;
  } else
    MessageScreen(MaxTipMessage, sizeof(MaxTipMessage));
}



uint16_t denoiseAnalog(byte port) {
  uint32_t result = 0;
  float maxValue, minValue;
  int resultArray[8];

  for (uint8_t i = 0; i < 8; i++) {

    float value, raw_adc;

    raw_adc = adc_sensor.readMiliVolts();

    value = constrain(0.5378 * raw_adc + 6.3959, 20, 1000);

    resultArray[i] = value;
  }



  for (uint8_t i = 0; i < 8; i++) {
    for (uint8_t j = i + 1; j < 8; j++) {
      if (resultArray[i] > resultArray[j]) {
        int temp = resultArray[i];
        resultArray[i] = resultArray[j];
        resultArray[j] = temp;
      }
    }
  }


  for (uint8_t i = 2; i < 6; i++) {
    result += resultArray[i];
  }





  return (result / 4);
}


double getChipTemp() {
#if defined(MPU)
  mpu6050.update();
  int16_t Temp = mpu6050.getTemp();
#elif defined(LIS)
  int16_t Temp = accel.getTemperature();
#endif

  return Temp;
}


float getMPUTemp() {
#if defined(MPU)
  mpu6050.update();
  int16_t Temp = mpu6050.getTemp();
#elif defined(LIS)
  int16_t Temp = accel.getTemperature();
#endif

  return Temp;
}


uint16_t getVIN() {
  long value;
  long voltage;
  long result = 0;

  for (uint8_t i = 0; i < 4; i++) {

    long val = adc_vin.readMiliVolts();

    result += val;
  }

  value = (result / 4);
# 1457 "D:/Documents/Songguo-PTS200/SolderingPen_ESP32S2/SolderingPen_ESP32S2.ino"
  voltage = value * 31.3;

  return voltage;

}

int32_t variance(int16_t a[]) {

  int32_t sum = 0;

  for (int i = 0; i < 32; i++) sum += a[i];
  int16_t mean = (int32_t)sum / 32;

  int32_t sqDiff = 0;
  for (int i = 0; i < 32; i++) sqDiff += (a[i] - mean) * (a[i] - mean);
  return (int32_t)sqDiff / 32;
}

unsigned int Button_Time1 = 0, Button_Time2 = 0;

void Button_loop() {
  if (!digitalRead(BUTTON_N_PIN) && a0 == 1) {
    delay(BUTTON_DELAY);
    if (!digitalRead(BUTTON_N_PIN)) {
      int count0 = count;
      count = constrain(count + countStep, countMin, countMax);
      if (!(countMin == TEMP_MIN && countMax == TEMP_MAX)) {
        if (count0 + countStep > countMax) {
          count = countMin;
        }
      }
      a0 = 0;
    }
  } else if (!digitalRead(BUTTON_N_PIN) && a0 == 0) {
    delay(BUTTON_DELAY);
    if (Button_Time1 > 10)
      count = constrain(count + countStep, countMin, countMax);
    else
      Button_Time1++;
  } else if (digitalRead(BUTTON_N_PIN)) {
    Button_Time1 = 0;
    a0 = 1;
  }

  if (!digitalRead(BUTTON_P_PIN) && b0 == 1) {
    delay(BUTTON_DELAY);
    if (!digitalRead(BUTTON_P_PIN)) {
      int count0 = count;
      count = constrain(count - countStep, countMin, countMax);
      if (!(countMin == TEMP_MIN && countMax == TEMP_MAX)) {
        if (count0 - countStep < countMin) {
          count = countMax;
        }
      }
      b0 = 0;
    }
  } else if (!digitalRead(BUTTON_P_PIN) && b0 == 0) {
    delay(BUTTON_DELAY);
    if (Button_Time2 > 10)
      count = constrain(count - countStep, countMin, countMax);
    else
      Button_Time2++;
  } else if (digitalRead(BUTTON_P_PIN)) {
    Button_Time2 = 0;
    b0 = 1;
  }
}

void PD_Update() {
  switch (VoltageValue) {
    case 0: {
      digitalWrite(PD_CFG_0, LOW);
      digitalWrite(PD_CFG_1, LOW);
      digitalWrite(PD_CFG_2, LOW);
    } break;
    case 1: {
      digitalWrite(PD_CFG_0, LOW);
      digitalWrite(PD_CFG_1, LOW);
      digitalWrite(PD_CFG_2, HIGH);
    } break;
    case 2: {
      digitalWrite(PD_CFG_0, LOW);
      digitalWrite(PD_CFG_1, HIGH);
      digitalWrite(PD_CFG_2, HIGH);
    } break;
    case 3: {
      digitalWrite(PD_CFG_0, LOW);
      digitalWrite(PD_CFG_1, HIGH);
      digitalWrite(PD_CFG_2, LOW);
    } break;
    case 4: {
      digitalWrite(PD_CFG_0, LOW);
      digitalWrite(PD_CFG_1, HIGH);
      digitalWrite(PD_CFG_2, LOW);
    } break;
    default:
      break;
  }

  if (VoltageValue == 3) {
    ledcSetup(CONTROL_CHANNEL, CONTROL_FREQ_20V, CONTROL_RES);
  } else {
    ledcSetup(CONTROL_CHANNEL, CONTROL_FREQ, CONTROL_RES);
  }

  ledcAttachPin(CONTROL_PIN, CONTROL_CHANNEL);



  ledcWrite(CONTROL_CHANNEL, HEATER_OFF);
}

static void usbEventCallback(void *arg, esp_event_base_t event_base,
                             int32_t event_id, void *event_data) {
  if (event_base == ARDUINO_USB_EVENTS) {

    switch (event_id) {
      case ARDUINO_USB_STARTED_EVENT:

        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_ncenB08_tr);
        u8g2.drawStr(0, 10,
                     "USB PLUGGED");
        u8g2.sendBuffer();
        break;
      case ARDUINO_USB_STOPPED_EVENT:

        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_ncenB08_tr);
        u8g2.drawStr(
            0, 10, "USB UNPLUGGED");
        u8g2.sendBuffer();
        break;
      case ARDUINO_USB_SUSPEND_EVENT:


        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_ncenB08_tr);
        u8g2.drawStr(
            0, 10, "USB SUSPENDED");
        u8g2.sendBuffer();
        break;
      case ARDUINO_USB_RESUME_EVENT:

        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_ncenB08_tr);
        u8g2.drawStr(0, 10,
                     "USB RESUMED");
        u8g2.sendBuffer();
        break;

      default:
        break;
    }
  } else if (event_base == ARDUINO_FIRMWARE_MSC_EVENTS) {


    switch (event_id) {
      case ARDUINO_FIRMWARE_MSC_START_EVENT:

        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_ncenB08_tr);
        u8g2.drawStr(
            0, 10,
            "MSC Update Start");
        u8g2.sendBuffer();
        break;
      case ARDUINO_FIRMWARE_MSC_WRITE_EVENT:



        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_ncenB08_tr);
        u8g2.drawStr(0, 10,
                     "MSC Updating");
        u8g2.sendBuffer();
        break;
      case ARDUINO_FIRMWARE_MSC_END_EVENT:

        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_ncenB08_tr);
        u8g2.drawStr(
            0, 10, "MSC Update End");
        u8g2.sendBuffer();
        break;
      case ARDUINO_FIRMWARE_MSC_ERROR_EVENT:


        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_ncenB08_tr);
        u8g2.drawStr(
            0, 10,
            "MSC Update ERROR!");
        u8g2.sendBuffer();
        break;
      case ARDUINO_FIRMWARE_MSC_POWER_EVENT:



        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_ncenB08_tr);
        u8g2.drawStr(
            0, 10,
            "MSC Update Power");
        u8g2.sendBuffer();
        break;

      default:
        break;
    }
  }
}

void turnOffHeater(Button2 &b) { inOffMode = true; }
# 1692 "D:/Documents/Songguo-PTS200/SolderingPen_ESP32S2/SolderingPen_ESP32S2.ino"
void heatWithLimit() {



  limit = 0;
  if (VoltageValue < 3) {
    limit = POWER_LIMIT_15;
  } else if (VoltageValue == 3) {
    limit = POWER_LIMIT_20;
  }
  ledcWrite(
      CONTROL_CHANNEL,
      constrain(HEATER_PWM, 0, limit));
}