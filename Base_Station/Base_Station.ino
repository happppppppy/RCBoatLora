// include the library
#include <RadioLib.h>
#include <SPI.h>
#include <Arduino.h>
#include <IWatchdog.h>

//Only change these macros to not break stuff :D
#define BASE_STATION
#define BOAT_ADDRESS 0x01


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

// END RADIO DEFINITIONS

// BEGIN SERVO DEFINITIONS

// END SERVO DEFINITIONS

// BEGIN DRIVE MOTOR DEFINITIONS

// END DRIVE MOTOR DEFINITIONS

void intTo2ByteArray(int value, uint8_t byteArray[2]) {
  // Input validation: Ensure the input is within the expected range.
  if (value < 0) {
    value = 0;
  } else if (value > 180) {
    value = 180;
  }

  // Convert the integer to a 2-byte array (little-endian)
  byteArray[0] = (uint8_t)(value & 0xFF); // Low byte
  byteArray[1] = (uint8_t)((value >> 8) & 0xFF); // High byte (will be 0 for values 0-180)
}

void combineArrays(const uint8_t arr1[2], const uint8_t arr2[2], uint8_t combined[4]) {
  memcpy(combined, arr1, 2);
  memcpy(combined + 2, arr2, 2);
}

bool isNumeric(String str) {
  if (str.length() == 0) {
    return false;
  }
  for (int i = 0; i < str.length(); i++) {
    if (isDigit(str.charAt(i)) == false) {
      return false;
    }
  }
  return true;
}

void setup() {
  IWatchdog.begin(4000000);  
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
  radio.setBandwidth(250);
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
  byte byteArray[5];
  if (Serial.available() > 0) {
    String inputString = Serial.readStringUntil('\n');
    inputString.trim(); // Remove whitespace

    // Parse the comma-separated string
    int commaIndex1 = inputString.indexOf(',');
    int commaIndex2 = inputString.indexOf(',', commaIndex1 + 1);
    int commaIndex3 = inputString.indexOf(',', commaIndex2 + 1);
    int commaIndex4 = inputString.indexOf(',', commaIndex3 + 1);

    if (commaIndex1 != -1 && commaIndex2 != -1 && commaIndex3 != -1 && commaIndex4 != -1) {
      String addressStr = inputString.substring(0, commaIndex1);
      String modeStr = inputString.substring(commaIndex1 + 1, commaIndex2);
      String directionEnableStr = inputString.substring(commaIndex2 + 1, commaIndex3);
      String speedStr = inputString.substring(commaIndex3 + 1, commaIndex4);
      String angleStr = inputString.substring(commaIndex4 + 1);
      if (isNumeric(addressStr) && isNumeric(modeStr) && isNumeric(directionEnableStr) && isNumeric(speedStr) && isNumeric(angleStr)) {
        byte address = addressStr.toInt();
        byte mode = modeStr.toInt();
        byte directionEnable = directionEnableStr.toInt();
        byte speed = speedStr.toInt();
        byte angle = angleStr.toInt();

        // Validate directionEnable
        if (directionEnable >= 0 && directionEnable <= 3) { // 0-3 allows all 2bit combinations.
          byte direction = (directionEnable & 0b00000001); // extract bit 1
          byte enable = (directionEnable & 0b00000010) >> 1; // extract bit 2 and shift it.
          byte directionEnableByte = (enable << 1) | direction; //construct the directionEnable byte

          // Store values in the array
          
          byteArray[0] = address;
          byteArray[1] = mode;
          byteArray[2] = directionEnableByte;
          byteArray[3] = speed;
          byteArray[4] = angle;

          Serial.print("Byte Array:\t\t");
          Serial.print(byteArray[0]); 
          Serial.print(",");
          Serial.print(byteArray[1]); 
          Serial.print(",");
          Serial.print(byteArray[2]); 
          Serial.print(",");
          Serial.print(byteArray[3]); 
          Serial.print(",");
          Serial.print(byteArray[4]); 
          Serial.println();

          transmissionState = radio.transmit(byteArray, 5);
          if (transmissionState == RADIOLIB_ERR_NONE) {
            // packet was successfully sent
            Serial.println(F("Transmission finished."));
            operationDone = false;

          } else {
            Serial.print(F("failed, code "));
            Serial.println(transmissionState);
          }

          receiveState = radio.startReceive();
          if (receiveState == RADIOLIB_ERR_NONE) {
            Serial.println(F("Ready to receive"));
          } else {
            Serial.print(F("failed, code "));
            Serial.println(receiveState);
            while (true) { delay(10); }
          }

        }
      }
    }
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
  }
}
