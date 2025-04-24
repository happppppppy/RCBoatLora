// include the library
#include <RadioLib.h>
#include <SPI.h>
#include <Arduino.h>
#include <IWatchdog.h>
#include <cmath>


const int HV_BATT_VOLT_pin = PA0;
const uint32_t adcResolution = 4096; // For 12-bit ADC (typical for STM32)
const uint32_t vref_mv = 3300;    // Reference voltage in mV (check your STM32 board)

// Define voltage divider resistor values (in ohms)
const uint32_t r1 = 220000;
const uint32_t r2 = 30000;

// BEGIN RADIO DEFINITIONS
#define BOAT_CONTROLLER
#define BOAT_ADDRESS 0x01

// SX1262 has the following connections:
int DIO1_pin = PA10;
int NRST_pin = PB5;
int BUSY_pin = PB3;
int RXEN_pin = PA11;
int TXEN_pin = PA12;
int MOSI_pin = PB15;
int MISO_pin = PB14;
int SCK_pin = PB13;
int NSS_pin = PB12;

#define SERVO_ANGLE_pin PB8
uint32_t servoAngle_us = 0;


int BLDC_SPEED_pin = PA7;
int BLDC_DIRECTION_pin = PA6;
int BLDC_TACHO_pin = PC7;
int BLDC_ALARM_pin = PB6;
int BLDC_ENABLE_pin = PA9;

SPIClass SPI_2(MOSI_pin, MISO_pin, SCK_pin);
SPISettings spiSettings(2000000, MSBFIRST, SPI_MODE0);
SX1262 radio = new Module(NSS_pin, DIO1_pin, NRST_pin, BUSY_pin, SPI_2, spiSettings);

// flag to indicate that a packet was sent or received
volatile bool receivedFlag = false;

// this function is called when a complete packet
// is transmitted or received by the module
// IMPORTANT: this function MUST be 'void' type
//            and MUST NOT have any arguments!
#if defined(ESP8266) || defined(ESP32)
ICACHE_RAM_ATTR
#endif
void setFlag(void) {
  // we received a packet, set the flag
  receivedFlag = true;
}

// END RADIO DEFINITIONS

// BEGIN SERVO DEFINITIONS

#define SERVO_MIN_ANGLE_DEGREES 0
#define SERVO_MAX_ANGLE_DEGREES 180
#define SERVO_ANGLE_LIMIT_LOWER_DEGREES 45
#define SERVO_ANGLE_CENTER_DEGREES 90
#define SERVO_ANGLE_LIMIT_UPPER_DEGREES 135
#define SERVO_MAX_MICROSECOND 2500  //(12.5% duty at 50Hz = 2.5ms)
#define SERVO_MID_MICROSECOND 1500  //Used for setting direction to zero on timeout
#define SERVO_MIN_MICROSECOND 500   //(2.5% duty at 50Hz = 0.5ms)

int upperMicrosecondLimit = map(SERVO_ANGLE_LIMIT_UPPER_DEGREES, SERVO_MIN_ANGLE_DEGREES, SERVO_MAX_ANGLE_DEGREES, SERVO_MIN_MICROSECOND, SERVO_MAX_MICROSECOND);
int lowerMicrosecondLimit = map(SERVO_ANGLE_LIMIT_LOWER_DEGREES, SERVO_MIN_ANGLE_DEGREES, SERVO_MAX_ANGLE_DEGREES, SERVO_MIN_MICROSECOND, SERVO_MAX_MICROSECOND);
int middleMicrosecond = map(SERVO_ANGLE_CENTER_DEGREES, SERVO_MIN_ANGLE_DEGREES, SERVO_MAX_ANGLE_DEGREES, SERVO_MIN_MICROSECOND, SERVO_MAX_MICROSECOND);

HardwareTimer *servoPWM;
uint32_t servo_channel;

int scale_angle(int angle) {
  if (angle < SERVO_ANGLE_LIMIT_LOWER_DEGREES || angle > SERVO_ANGLE_LIMIT_UPPER_DEGREES) {
    //"Input angle should be between 0 and 180 degrees.\n"
    return SERVO_MIN_MICROSECOND + (SERVO_MAX_MICROSECOND - SERVO_MIN_MICROSECOND) / 2;
  }

  int scaled_value = SERVO_MIN_MICROSECOND + (SERVO_MAX_MICROSECOND - SERVO_MIN_MICROSECOND) * angle / SERVO_MAX_ANGLE_DEGREES;
  return scaled_value;
}

// END SERVO DEFINITIONS

// BEGIN DRIVE MOTOR DEFINITIONS#define SERVO_MIN_ANGLE_DEGREES			      0
#define BLDC_MAX_SPEED_PCT 100
#define BLDC_MIN_SPEED_PCT 0


HardwareTimer *bldcPWM;
uint32_t bldc_channel;
// END DRIVE MOTOR DEFINITIONS

uint8_t boatAddress = BOAT_ADDRESS;

uint32_t mapParabolicCentered(uint8_t inputVal, uint32_t inputMin, uint32_t inputMid, uint32_t inputMax, uint32_t outputMin, uint32_t outputMid, uint32_t outputMax) {
  float parabolaCurveFactor = 1.0; // You can adjust this factor (though in the simplest parabola it's 1)
  float normalizedDistanceFromCenter;
  float parabolicOutput;

  if (inputVal <= inputMid) {
    // Map the lower half (0 to 64)
    normalizedDistanceFromCenter = static_cast<float>(inputMid - inputVal) / (inputMid - inputMin);
    parabolicOutput = normalizedDistanceFromCenter * normalizedDistanceFromCenter; // y = x^2
    return static_cast<uint32_t>(outputMid - parabolicOutput * (outputMid - outputMin));
  } else {
    // Map the upper half (65 to 127)
    normalizedDistanceFromCenter = static_cast<float>(inputVal - inputMid) / (inputMax - inputMid);
    parabolicOutput = normalizedDistanceFromCenter * normalizedDistanceFromCenter; // y = x^2
    return static_cast<uint32_t>(outputMid + parabolicOutput * (outputMax - outputMid));
  }
}

uint32_t mapParabolicNonCentered(uint32_t inputValue, uint32_t inputMin, uint32_t inputMax, uint32_t outputMin, uint32_t outputMax) {
  float parabolaCurveFactor = 1.0; // You can adjust this factor (though in the simplest parabola it's 1)
  float normalizedInput = static_cast<float>(inputValue - inputMin) / (inputMax - inputMin);
  float parabolicOutput = pow(normalizedInput, parabolaCurveFactor);
  return static_cast<uint32_t>(parabolicOutput * (outputMax - outputMin) + outputMin);
}

volatile bool RxWatchdogTimeout = false;
void RxWatchdogTimer_IT_callback(void){
  RxWatchdogTimeout = true;
}

HardwareTimer *RxWatchdogTimer;

void setup() {
  IWatchdog.begin(4000000);

  RxWatchdogTimer = new HardwareTimer(TIM5);
  RxWatchdogTimer->setMode(1,TIMER_OUTPUT_DISABLED);
  RxWatchdogTimer->setPrescaleFactor(0);
  RxWatchdogTimer->setOverflow(4000000, MICROSEC_FORMAT);
  RxWatchdogTimer->attachInterrupt(RxWatchdogTimer_IT_callback);
  RxWatchdogTimer->resume();
  //SERVO PWM
  TIM_TypeDef *servoInstance = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(SERVO_ANGLE_pin), PinMap_PWM);
  servo_channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(SERVO_ANGLE_pin), PinMap_PWM));
  servoPWM = new HardwareTimer(servoInstance);

  servoPWM->setMode(servo_channel, TIMER_OUTPUT_COMPARE_PWM1, SERVO_ANGLE_pin);
  servoPWM->setOverflow(50, HERTZ_FORMAT);
  servoPWM->setCaptureCompare(servo_channel, 1500, MICROSEC_COMPARE_FORMAT);  // 7.5%
  servoPWM->resume();

  //BLDC PWM
  TIM_TypeDef *bldcInstance = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(BLDC_SPEED_pin), PinMap_PWM);
  bldc_channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(BLDC_SPEED_pin), PinMap_PWM));
  bldcPWM = new HardwareTimer(bldcInstance);

  bldcPWM->setMode(bldc_channel, TIMER_OUTPUT_COMPARE_PWM1, BLDC_SPEED_pin);
  bldcPWM->setOverflow(10000, HERTZ_FORMAT);
  bldcPWM->setCaptureCompare(bldc_channel, 0, PERCENT_COMPARE_FORMAT);
  bldcPWM->resume();

  // pinMode(BLDC_SPEED_pin, OUTPUT);
  pinMode(BLDC_DIRECTION_pin, OUTPUT);
  pinMode(BLDC_ENABLE_pin, OUTPUT);
  pinMode(BLDC_TACHO_pin, INPUT);
  pinMode(BLDC_ALARM_pin, INPUT);

  // pinMode(HV_BATT_VOLT_pin, INPUT);

  digitalWrite(BLDC_DIRECTION_pin, LOW);
  digitalWrite(BLDC_ENABLE_pin, HIGH);  //LOW is enabled

  Serial.begin(9600);
  delay(500);
  Serial.println(F("Starting SPI Bus"));
  SPI_2.begin();


  // initialize SX1262 with default settings
  Serial.println(F("[SX1262] Initializing ... "));
  int state = radio.begin();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true) { delay(10); }
  }

  // set the function that will be called
  // when new packet is received
  radio.setPacketReceivedAction(setFlag);
  Serial.println(F("Setting RF Switch Pins"));
  radio.setRfSwitchPins(RXEN_pin, TXEN_pin);
  radio.setRegulatorLDO();
  Serial.println(F("Setting Output Power"));
  radio.setOutputPower(22);
  Serial.println(F("Setting Frequency"));
  radio.setFrequency(915);
  Serial.println(F("Setting Bandwidth"));
  radio.setBandwidth(62.5);
  Serial.println(F("Setting Spreading Factor"));
  radio.setSpreadingFactor(7);


  state = radio.startReceive();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("Ready to receive"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true) { delay(10); }
  }
}

void loop() {
  IWatchdog.reload();
  if (RxWatchdogTimeout){
    servoPWM->setCaptureCompare(servo_channel, SERVO_MID_MICROSECOND, MICROSEC_COMPARE_FORMAT);
    bldcPWM->setCaptureCompare(bldc_channel, 0, PERCENT_COMPARE_FORMAT);

  }
  // uint32_t battery_voltage = analogRead(HV_BATT_VOLT_pin);
  // uint32_t adcRawValue = analogRead(HV_BATT_VOLT_pin);
  // uint32_t vAdc_mV_numerator = adcRawValue * vref_mv;
  // uint32_t vAdc_mV = vAdc_mV_numerator / adcResolution;

  // uint32_t vBattery_mV_numerator = vAdc_mV * (r1 + r2);
  // uint32_t vBattery_mV = vBattery_mV_numerator / r2;

  // Serial.println(vBattery_mV);
  // check if the flag is set
  if (receivedFlag) {
    RxWatchdogTimer->setCount(0, MICROSEC_FORMAT);

    // you can read received data as an Arduino String
    byte packetData[2];
    int state = radio.readData(packetData, 2);

    if (state == RADIOLIB_ERR_NONE) {
      RxWatchdogTimeout = false;
      servoAngle_us = mapParabolicCentered((packetData[1] >> 1), 0, 64, 127, lowerMicrosecondLimit, middleMicrosecond, upperMicrosecondLimit);
      servoPWM->setCaptureCompare(servo_channel, servoAngle_us, MICROSEC_COMPARE_FORMAT);  // 7.5%

      //Set the bldc PWM speed value
      // uint16_t bldcPct = map((packetData[0] >> 1), 0, 127, 0, 100);
      uint32_t bldcPct = mapParabolicNonCentered((packetData[0] >> 1), 0, 127, 0, 100);
      
      bldcPWM->setCaptureCompare(bldc_channel, bldcPct, PERCENT_COMPARE_FORMAT);

      //Set the motor direction (bit 1)
      bool directionBool = (packetData[0] & 0b00000001) != 0;
      if (directionBool) {
        digitalWrite(BLDC_DIRECTION_pin, HIGH);
      } else {
        digitalWrite(BLDC_DIRECTION_pin, LOW);
      }

      //Set the motor enable state (bit 2)
      bool enableBool = (packetData[1] & 0b00000001) != 0;
      if (enableBool) {
        digitalWrite(BLDC_ENABLE_pin, LOW);} else {
        digitalWrite(BLDC_ENABLE_pin, HIGH);
      }

      // //Send back the boat status
      byte statusPacketData[2];

      // //Byte 1 is the boat address
      // //Byte 2 is the mode (UNUSED)
      // //Byte 3 is the alarm state
      // //Byte 4 is the tacho
      // //Byte 5 is RSSI
      // //Byte 6 is SNR

      // statusPacketData[0] = (uint8_t)BOAT_ADDRESS;
      // statusPacketData[1] = 0;
      // statusPacketData[2] = (uint8_t)digitalRead(BLDC_ALARM_pin);
      // statusPacketData[3] = 0;  //UNIMPLEMENTED
      statusPacketData[0] = (uint8_t)abs(radio.getRSSI());
      statusPacketData[1] = (uint8_t)abs(radio.getSNR());

      state = radio.transmit(statusPacketData, 2);
      if (state != RADIOLIB_ERR_NONE) {
        Serial.print(F("failed, code "));
        Serial.println(state);
      }
      
      state = radio.startReceive();

    } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
      Serial.println(F("CRC error!"));

    } else if (state == RADIOLIB_ERR_SPI_CMD_TIMEOUT) {
      Serial.print(F("failed, code "));
      Serial.println(state);
      while(true){}
    } else {
      // some other error occurred
      Serial.print(F("failed, code "));
      Serial.println(state);
      while(true){}
    }

    receivedFlag = false;
  }
}
