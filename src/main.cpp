#include <Arduino.h>

#define ESP32_CAN_TX_PIN GPIO_NUM_22
#define ESP32_CAN_RX_PIN GPIO_NUM_21

#include <NMEA2000_CAN.h>
#include <N2kMessages.h>
#include <N2kMessagesEnumToStr.h>


#include <DHT.h>
#define DHT_PIN 15
DHT dht(DHT_PIN, DHT22);
int chk;
float temp;
float hum;

#define AC_RELAY_PIN 19

void SendN2kTemperature();
void SendN2kACState();

const unsigned long TransmitMessages[] PROGMEM={
  130311L,
  127501L,
  0
};

void setup() {
  Serial.begin(115200); delay(500);

  dht.begin();

  NMEA2000.SetProductInformation("00000001");
  NMEA2000.SetDeviceInformation(1, // Unique number. Use e.g. Serial number.
                                130, // Device function=Temperature. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                75, // Device class=Sensor Communication Interface. See codes on  http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                2040 // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf                               
                               );

  NMEA2000.SetForwardStream(&Serial);
  NMEA2000.SetForwardType(tNMEA2000::fwdt_Text);
  NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode, 42);
  NMEA2000.EnableForward(true);
  NMEA2000.ExtendTransmitMessages(TransmitMessages);
  NMEA2000.Open();

  pinMode(DHT_PIN, INPUT);
  pinMode(AC_RELAY_PIN, INPUT);
}

void loop() {
  NMEA2000.ParseMessages();
  SendN2kTemperature();
  SendN2kACState();
}

double ReadTemp() {
  return CToKelvin(dht.readTemperature());
}

double ReadHumidity() {
  return dht.readHumidity();
}

tN2kOnOff ReadACState() {
  return (digitalRead(AC_RELAY_PIN)) ? N2kOnOff_Off : N2kOnOff_On;
}


#define ACStateUpdateInterval 2500

void SendN2kACState() {
  tN2kMsg N2kMsg;

  static unsigned long updated = millis();

  if ( updated + ACStateUpdateInterval < millis() ) {
    updated = millis();

    SetN2kBinaryStatus(N2kMsg, 1, ReadACState());
    NMEA2000.SendMsg(N2kMsg);
    Serial.print(millis()); Serial.println(", AC State send ready");
  }
}


#define TempUpdatePeriod 2500

void SendN2kTemperature() {
  static unsigned long TempUpdated=millis();
  tN2kMsg N2kMsg;
  
  if ( TempUpdated+TempUpdatePeriod<millis() ) {
    TempUpdated=millis();

    SetN2kEnvironmentalParameters(N2kMsg, 1, N2kts_InsideTemperature, ReadTemp(), N2khs_InsideHumidity, ReadHumidity());
    NMEA2000.SendMsg(N2kMsg);

    Serial.print(millis()); Serial.println(", Temperature send ready");
  }
}