// include the library
#include <RadioLib.h>
#include <SPI.h>
#include <Arduino.h>
#include <IWatchdog.h>


// BEGIN RADIO DEFINITIONS
#define BOAT_CONTROLLER
#define BOAT_ADDRESS 0x01

// SX1262 has the following connections:
int DIO1_pin  = PC8;
int NRST_pin  = PC5;
int BUSY_pin  = PC6;
int RXEN_pin  = PA12;
int TXEN_pin  = PA11;
int MOSI_pin  = PB15;
int MISO_pin  = PB14;
int SCK_pin   = PB13;
int NSS_pin   = PB12;

SPIClass SPI_2(MOSI_pin, MISO_pin, SCK_pin);
SPISettings spiSettings(2000000, MSBFIRST, SPI_MODE0);
SX1262 radio = new Module(NSS_pin, DIO1_pin, NRST_pin, BUSY_pin, SPI_2, spiSettings);

// save transmission states between loops
int transmissionState = RADIOLIB_ERR_NONE;

// flag to indicate transmission or reception state
bool transmitFlag = false;

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
#define SERVO_ANGLE_pin PB6

#define SERVO_MIN_ANGLE_DEGREES			      0
#define SERVO_MAX_ANGLE_DEGREES			      180
#define SERVO_ANGLE_LIMIT_LOWER_DEGREES	  45
#define SERVO_ANGLE_LIMIT_UPPER_DEGREES	  135
#define SERVO_MAX_MICROSECOND							2500 //(12.5% duty at 50Hz = 2.5ms)
#define SERVO_MIN_MICROSECOND							500 //(2.5% duty at 50Hz = 0.5ms)

HardwareTimer *servoPWM;
uint32_t servo_channel;

int scale_angle(int angle) {
    if (angle < SERVO_ANGLE_LIMIT_LOWER_DEGREES || angle > SERVO_ANGLE_LIMIT_UPPER_DEGREES) {
        //"Input angle should be between 0 and 180 degrees.\n"
        return SERVO_MIN_MICROSECOND + (SERVO_MAX_MICROSECOND - SERVO_MIN_MICROSECOND)/2;
    }

    int scaled_value = SERVO_MIN_MICROSECOND + (SERVO_MAX_MICROSECOND - SERVO_MIN_MICROSECOND) * angle / SERVO_MAX_ANGLE_DEGREES;
    return scaled_value;
}

// END SERVO DEFINITIONS

// BEGIN DRIVE MOTOR DEFINITIONS#define SERVO_MIN_ANGLE_DEGREES			      0
#define BLDC_MAX_SPEED_PCT			      100
#define BLDC_MIN_SPEED_PCT			      0

int BLDC_SPEED_pin = PA6;
int BLDC_DIRECTION_pin = PB5;
int BLDC_TACHO_pin = PB10;
int BLDC_ALARM_pin = PA9;
int BLDC_ENABLE_pin = PA8;

HardwareTimer *bldcPWM;
uint32_t bldc_channel;
// END DRIVE MOTOR DEFINITIONS

uint8_t boatAddress = BOAT_ADDRESS;

void setup() {
  IWatchdog.begin(4000000);
  //SERVO PWM
  TIM_TypeDef *servoInstance = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(SERVO_ANGLE_pin), PinMap_PWM);
  servo_channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(SERVO_ANGLE_pin), PinMap_PWM));
  servoPWM = new HardwareTimer(servoInstance);

  servoPWM->setMode(servo_channel, TIMER_OUTPUT_COMPARE_PWM1, SERVO_ANGLE_pin);
  servoPWM->setOverflow(50, HERTZ_FORMAT);
  servoPWM->setCaptureCompare(servo_channel, 1500, MICROSEC_COMPARE_FORMAT); // 7.5%
  servoPWM->resume();

  //BLDC PWM
  TIM_TypeDef *bldcInstance = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(BLDC_SPEED_pin), PinMap_PWM);
  bldc_channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(BLDC_SPEED_pin), PinMap_PWM));
  bldcPWM = new HardwareTimer(bldcInstance);

  bldcPWM->setMode(bldc_channel, TIMER_OUTPUT_COMPARE_PWM1, BLDC_SPEED_pin);
  bldcPWM->setOverflow(50000, HERTZ_FORMAT);
  bldcPWM->setCaptureCompare(bldc_channel, 0, PERCENT_COMPARE_FORMAT);
  bldcPWM->resume();

  // pinMode(BLDC_SPEED_pin, OUTPUT);
  pinMode(BLDC_DIRECTION_pin, OUTPUT);
  pinMode(BLDC_ENABLE_pin, OUTPUT);
  pinMode(BLDC_TACHO_pin, INPUT);
  pinMode(BLDC_ALARM_pin, INPUT);

  digitalWrite(BLDC_DIRECTION_pin, LOW);
  digitalWrite(BLDC_ENABLE_pin, HIGH); //LOW is enabled
  
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


  Serial.println(F("Setting Output Power"));
  radio.setOutputPower(20);
  Serial.println(F("Setting Frequency"));
  radio.setFrequency(915);
  Serial.println(F("Setting Bandwidth"));
  radio.setBandwidth(250);
  Serial.println(F("Setting RF Switch Pins"));
  radio.setRfSwitchPins(RXEN_pin, TXEN_pin);
  Serial.println(F("Setting Spreading Factor"));
  radio.setSpreadingFactor(8);
  // Serial.println(F("Setting Coding Rate"));
  // radio.setCodingRate(7);

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
  // check if the flag is set
  if(receivedFlag) {

    // you can read received data as an Arduino String
    byte packetData[2];
    int numBytes = radio.getPacketLength();
    int state = radio.readData(packetData, numBytes);


    if (state == RADIOLIB_ERR_NONE) {
      // packet was successfully received
      Serial.println(F("[SX1262] Received packet!"));
      Serial.print(F("[SX1262] Data:\t\t"));
      Serial.print(packetData[0]);
      Serial.print(",");
      Serial.println(packetData[1]);

      // if(packetData[0] == boatAddress){
        //Set the servo angle by first mapping the input to angle then angle to microseconds
  
        uint32_t servoAngle_us = map(map((packetData[1] >> 1), 0, 127, SERVO_MIN_ANGLE_DEGREES, SERVO_MAX_ANGLE_DEGREES) , SERVO_MIN_ANGLE_DEGREES, SERVO_MAX_ANGLE_DEGREES, SERVO_MIN_MICROSECOND, SERVO_MAX_MICROSECOND);
        servoPWM->setCaptureCompare(servo_channel, servoAngle_us, MICROSEC_COMPARE_FORMAT); // 7.5%
        
        //Set the bldc PWM speed value
        bldcPWM->setCaptureCompare(bldc_channel, map((packetData[0] >> 1), 0,127,0,100), PERCENT_COMPARE_FORMAT);

        //Set the motor direction (bit 1)
        bool directionBool = (packetData[0] & 0b00000001) != 0;
        if(directionBool){
          digitalWrite(BLDC_DIRECTION_pin, HIGH);
        }
        else{
          digitalWrite(BLDC_DIRECTION_pin, LOW);
        }

        //Set the motor enable state (bit 2)
        bool enableBool = (packetData[1] & 0b00000010) != 0;
        if(enableBool){
          digitalWrite(BLDC_ENABLE_pin, LOW);
        }
        else{
          digitalWrite(BLDC_ENABLE_pin, HIGH);
        }

        // print RSSI (Received Signal Strength Indicator)
        Serial.print(F("[SX1262] RSSI:\t\t"));
        Serial.print(radio.getRSSI());
        Serial.println(F(" dBm"));

        // print SNR (Signal-to-Noise Ratio)
        Serial.print(F("[SX1262] SNR:\t\t"));
        Serial.print(radio.getSNR());
        Serial.println(F(" dB"));

        //Send back the boat status
        byte statusPacketData[5];

        //Byte 1 is the boat address
        //Byte 2 is the mode (UNUSED)
        //Byte 3 is the alarm state
        //Byte 4 is the tacho
        //Byte 5 is RSSI
        //Byte 6 is SNR 

        statusPacketData[0] = (uint8_t)BOAT_ADDRESS;
        statusPacketData[1] = 0;
        statusPacketData[2] = (uint8_t)digitalRead(BLDC_ALARM_pin);
        statusPacketData[3] = 0; //UNIMPLEMENTED
        statusPacketData[4] = (uint8_t)abs(radio.getRSSI());
        statusPacketData[5] = (uint8_t)abs(radio.getSNR());

        state = radio.transmit(statusPacketData, 7);
        if (transmissionState == RADIOLIB_ERR_NONE) {
          // packet was successfully sent
          Serial.println(F("Transmission finished."));
        } else {
          Serial.print(F("failed, code "));
          Serial.println(transmissionState);
          while (true) { delay(10); }
        }

        state = radio.startReceive();
        if (state == RADIOLIB_ERR_NONE) {
          Serial.println(F("Ready to receive."));
        } else {
          Serial.print(F("failed, code "));
          Serial.println(state);
          while (true) { delay(10); }
        }

      // }else{
      //   Serial.println(F("Incorrect boat controller address received"));
      // }


    } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
      // packet was received, but is malformed
      Serial.println(F("CRC error!"));

    } else {
      // some other error occurred
      Serial.print(F("failed, code "));
      Serial.println(state);

    }
    
    // reset flag
    receivedFlag = false;
  }
}
