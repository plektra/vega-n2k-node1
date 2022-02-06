#include <Arduino.h>
#include "ota.h"

#define ESP32_CAN_TX_PIN GPIO_NUM_32
#define ESP32_CAN_RX_PIN GPIO_NUM_34
#include <NMEA2000_CAN.h>
#include <N2kMessages.h>
#include <N2kMessagesEnumToStr.h>

#define LED_PIN 2
#define SDA_PIN 16
#define SCL_PIN 17
#define ONEWIRE_PIN 4
#define DC_RELAY_1_PIN 12
#define DC_RELAY_2_PIN 13
#define DC_RELAY_3_PIN 14
#define AC_RELAY_PIN 15

#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "OneWire.h"
#include "DallasTemperature.h"

Adafruit_BME280 bme;
TwoWire* i2c;
OneWire oneWire(ONEWIRE_PIN);
DallasTemperature* ds;

const unsigned long TransmitMessages[] PROGMEM={
  130311L,
  127501L,
  61200L,
  0
};

typedef struct {
  unsigned long PGN;
  void (*Handler)(const tN2kMsg &N2kMsg); 
} tNMEA2000Handler;

void SendN2kACState();
void SendN2kTemperature();
void SendBinaryStatus(boolean immediately = false);
void BinaryStatusHandler(const tN2kMsg &N2kMsg);
void HandleNMEA2000Msg(const tN2kMsg &N2kMsg);
void RebootHandler(const tN2kMsg &N2kMsg);

tNMEA2000Handler NMEA2000Handlers[]={
  {127501L,&BinaryStatusHandler},
  {61200L,&RebootHandler},
  {0,0}
};

void setup() {

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  Serial.begin(115200); delay(500);

  if (!OTAUpdate()) {
    for (int i=0; i<=10; i++) {
      digitalWrite(LED_PIN, LOW);
      delay(100);
      digitalWrite(LED_PIN, HIGH);
      delay(100);
    }
  }

  digitalWrite(LED_PIN, LOW);

  NMEA2000.SetProductInformation("00000001");
  NMEA2000.SetDeviceInformation(1, // Unique number. Use e.g. Serial number.
                                130, // Device function=Temperature. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                75, // Device class=Sensor Communication Interface. See codes on  http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                2040 // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf                               
                               );

  NMEA2000.SetForwardStream(&Serial);
  NMEA2000.SetForwardType(tNMEA2000::fwdt_Text);
  NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode, 1);
  NMEA2000.EnableForward(false);
  NMEA2000.ExtendTransmitMessages(TransmitMessages);
  NMEA2000.SetMsgHandler(HandleNMEA2000Msg);
  NMEA2000.Open();

  i2c = new TwoWire(0);
  i2c->begin(SDA_PIN, SCL_PIN);

  if (!bme.begin(0x76, i2c)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
  }
  bme.setTemperatureCompensation(-1.0);

  ds = new DallasTemperature(&oneWire);
  ds->begin();

  pinMode(DC_RELAY_1_PIN, OUTPUT);
  pinMode(DC_RELAY_2_PIN, OUTPUT);
  pinMode(AC_RELAY_PIN, INPUT_PULLDOWN);
}

void loop() {
  NMEA2000.ParseMessages();
  SendN2kTemperature();
  SendN2kACState();
  SendBinaryStatus();
}

double ReadEnclosureTemp() {
  ds->requestTemperatures();
  return CToKelvin(ds->getTempCByIndex(0));
}

double ReadCabinTemp() {
  return CToKelvin(bme.readTemperature());
}

double ReadHumidity() {
  return bme.readHumidity();
}

double ReadPressure() {
  return bme.readPressure();
}

tN2kOnOff ReadACState() {
  return (digitalRead(AC_RELAY_PIN)) ? N2kOnOff_On : N2kOnOff_Off;
}

#define ACStateUpdateInterval 2500

void SendN2kACState() {
  tN2kMsg N2kMsg;

  static unsigned long updated = millis();

  if ( updated + ACStateUpdateInterval < millis() ) {
    updated = millis();

    SetN2kBinaryStatus(N2kMsg, 2, ReadACState());
    NMEA2000.SendMsg(N2kMsg);
    Serial.print(millis()); Serial.println(", AC State send ready");
  }
}


#define TempUpdatePeriod 2500

void SendN2kTemperature() {
  static unsigned long TempUpdated=millis();
  tN2kMsg EnvMsg;
  tN2kMsg TempMsg;
  
  if (TempUpdated+TempUpdatePeriod<millis()) {
    TempUpdated=millis();

    SetN2kEnvironmentalParameters(EnvMsg, 1, N2kts_EngineRoomTemperature, ReadCabinTemp(), N2khs_InsideHumidity, ReadHumidity(), ReadPressure());
    NMEA2000.SendMsg(EnvMsg);
    
    SetN2kTemperature(TempMsg, 1, 1, N2kts_InsideTemperature, ReadEnclosureTemp());
    NMEA2000.SendMsg(TempMsg);

    Serial.print(millis()); Serial.println(", Temperature send ready");
  }
}

void BinaryStatusHandler(const tN2kMsg &inboundMsg) {
  unsigned char BankInstance = 1;
  tN2kOnOff Status1,Status2,Status3,Status4;

  if (ParseN2kBinaryStatus(inboundMsg,BankInstance,Status1,Status2,Status3,Status4) ) {
    if (Status1 == N2kOnOff_On || Status1 == N2kOnOff_Off) digitalWrite(DC_RELAY_1_PIN, (Status1 == N2kOnOff_On) ? HIGH : LOW);
    if (Status2 == N2kOnOff_On || Status2 == N2kOnOff_Off) digitalWrite(DC_RELAY_2_PIN, (Status2 == N2kOnOff_On) ? HIGH : LOW);
    delay(2);
    SendBinaryStatus(true);
  }
}

#define BinaryStatusUpdateInterval 2500

void SendBinaryStatus(boolean immediately) {
  tN2kMsg outboundMsg;

  static unsigned long updated = millis();

  if (immediately || updated + ACStateUpdateInterval < millis()) {
    updated = millis();
    SetN2kBinaryStatus(outboundMsg, 1,
      (digitalRead(DC_RELAY_1_PIN) == HIGH) ? N2kOnOff_On : N2kOnOff_Off,
      (digitalRead(DC_RELAY_2_PIN) == HIGH) ? N2kOnOff_On : N2kOnOff_Off,
      N2kOnOff_Unavailable,
      N2kOnOff_Unavailable);
    NMEA2000.SendMsg(outboundMsg);

    Serial.print(millis()); Serial.println(", Binary switch bank status send ready");
  }
}

void RebootHandler(const tN2kMsg &inboundMsg) {
  Serial.printf("Received message in PGN %ld\n", inboundMsg.PGN);
  Serial.println("Rebooting!");
  esp_restart();
}

void HandleNMEA2000Msg(const tN2kMsg &N2kMsg) {
  int iHandler;
  
  // Find handler
  //Serial.print("In Main Handler: "); Serial.println(N2kMsg.PGN);
  for (iHandler=0; NMEA2000Handlers[iHandler].PGN!=0 && !(N2kMsg.PGN==NMEA2000Handlers[iHandler].PGN); iHandler++);
  
  if (NMEA2000Handlers[iHandler].PGN!=0) {
    NMEA2000Handlers[iHandler].Handler(N2kMsg); 
  }
}
