// include the library
#include <RadioLib.h>
#include <SPI.h>
#include <Arduino.h>
#include <IWatchdog.h>

//Only change these macros to not break stuff :D
#define BASE_STATION
#define BOAT_ADDRESS 0x01 //address can be in the range 0->3
#define MIN_RUDDER_ANGLE 45 //Max angle - min angle must be <127 (To fit into 7 bits)
#define MAX_RUDDER_ANGLE 135

// Joystick Pins
int Left_Joystick_Y_pin = A2;
int Left_Joystick_X_pin = A3;
int Right_Joystick_Y_pin = A0;
int Right_Joystick_X_pin = A1;
int Right_Joystick_pushbutton_pin = PC15;


// BEGIN RADIO DEFINITIONS
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
int receiveState = RADIOLIB_ERR_NONE;

// flag to indicate transmission or reception state
bool transmitFlag = false;

// flag to indicate that a packet was sent or received
volatile bool operationDone = false;

// this function is called when a complete packet
// is transmitted or received by the module
// IMPORTANT: this function MUST be 'void' type
//            and MUST NOT have any arguments!
#if defined(ESP8266) || defined(ESP32)
  ICACHE_RAM_ATTR
#endif
void setFlag(void) {
  // we sent or received a packet, set the flag
  operationDone = true;
}

// VARIABLE DECLARATIONS
uint8_t motorPower;
uint8_t rudderAngle;
uint8_t direction;
uint8_t enable = 0x01;
uint8_t address;

const int16_t motorJoystickVal_midpoint = 525;
int16_t motorJoystickVal;

void setup() {
  IWatchdog.begin(4000000);
  pinMode(Left_Joystick_X_pin, INPUT);
  pinMode(Left_Joystick_Y_pin, INPUT);
  pinMode(Right_Joystick_X_pin, INPUT);
  pinMode(Right_Joystick_Y_pin, INPUT);
  pinMode(Right_Joystick_pushbutton_pin, INPUT_PULLUP);

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
  radio.setDio1Action(setFlag);

  Serial.println(F("Setting Output Power"));
  radio.setOutputPower(20);
  Serial.println(F("Setting Frequency"));
  radio.setFrequency(915);
  Serial.println(F("Setting Bandwidth"));
  radio.setBandwidth(500);
  Serial.println(F("Setting RF Switch Pins"));
  radio.setRfSwitchPins(RXEN_pin, TXEN_pin);
  Serial.println(F("Setting Spreading Factor"));
  radio.setSpreadingFactor(8);

  // start listening for LoRa packets on this node
  Serial.print(F("[SX1262] Starting to listen ... "));
  state = radio.startReceive();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true) { delay(10); }
  }

  Serial.println("Enter: Address,Mode,DirectionEnable,Speed,Angle (comma-separated)");
  Serial.println("Example: 1,2,3,100,90");

}

void loop() {
  IWatchdog.reload();

  motorJoystickVal = analogRead(Left_Joystick_Y_pin);

  if(motorJoystickVal < motorJoystickVal_midpoint){
    direction = 0x01;
    motorPower = map(motorJoystickVal, motorJoystickVal_midpoint - 1, 0, 0, 127);
  } else {
    direction = 0x00;
    motorPower = map(motorJoystickVal, motorJoystickVal_midpoint, 1023, 0, 127);
  }

  Serial.println(motorPower);
  
  rudderAngle = map(analogRead(Right_Joystick_X_pin), 0, 1023, 127, 0);

  uint8_t packet[2];
  packet[0] = (motorPower << 1) | direction;
  packet[1] = (rudderAngle << 1) | enable;
  
  
  Serial.print("Raw: ");
  Serial.print(motorJoystickVal);
  Serial.print(" Scaled: ");
  Serial.print(motorPower);
  Serial.print(" Dir: ");
  Serial.println(direction);
  Serial.print(" Rudder: ");
  Serial.println(rudderAngle);
  Serial.print(" Enable: ");
  Serial.println(enable);

  transmissionState = radio.transmit(packet, 2);
  if (transmissionState == RADIOLIB_ERR_NONE) {
    // packet was successfully sent
    Serial.println(F("Transmission finished."));
    operationDone = false;

  } else {
    Serial.print(F("failed, code "));
    Serial.println(transmissionState);
    while (true) { delay(10); } //Holding in this delay loop allows the watchdog to reset the controller.
  }

  // check if the flag is set
  if(operationDone) {

    int numBytes = radio.getPacketLength();
    byte packetData[numBytes];
    int receiveState = radio.readData(packetData, numBytes);
    Serial.print(F("[SX1262] Data:\t\t"));
    Serial.print(packetData[0]);
    Serial.print(",");
    Serial.print(packetData[1]);
    Serial.print(",");
    Serial.print(packetData[2]);
    Serial.print(",");
    Serial.print(packetData[3]);
    Serial.print(",");
    Serial.print(packetData[4]);
    Serial.print(",");
    Serial.println(packetData[5]);

    operationDone = false;
    receiveState = radio.startReceive();
      if (receiveState == RADIOLIB_ERR_NONE) {
    Serial.println(F("Ready to receive"));
    } else {
      Serial.print(F("failed, code "));
      Serial.println(receiveState);
      while (true) { delay(10); } //Holding in this delay loop allows the watchdog to reset the controller.
    }
  }
}
