#include <Arduino.h>
#include "ota.h"

#include <ReactESP.h>
using namespace reactesp;
ReactESP app;

#define CAN_TX_PIN GPIO_NUM_32
#define CAN_RX_PIN GPIO_NUM_34
#include <NMEA2000_esp32.h>
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
#define RPI_POWER_STATE_PIN 26
#define RPI_SHUTDOWN_PIN 27

#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "OneWire.h"
#include "DallasTemperature.h"

tNMEA2000* n2k;
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

void SendEnvironmentData();
void SendEnclosureTemperature();
void SendSwitchBankStatus(unsigned char DeviceBankInstance);
void SwitchBankStatusHandler(const tN2kMsg &N2kMsg);
void HandleNMEA2000Msg(const tN2kMsg &N2kMsg);
void RebootHandler(const tN2kMsg &N2kMsg);

tNMEA2000Handler NMEA2000Handlers[]={
  {127501L,&SwitchBankStatusHandler},
  {61200L,&RebootHandler},
  {0,0}
};

void setup() {

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  Serial.begin(115200); delay(500);

  // Download latest firmware and upgrade automatically
  if (!OTAUpdate()) {
    for (int i=0; i<=10; i++) {
      digitalWrite(LED_PIN, LOW);
      delay(100);
      digitalWrite(LED_PIN, HIGH);
      delay(100);
    }
  }
  digitalWrite(LED_PIN, LOW);

  // Set up CAN bus and NMEA2000
  n2k = new tNMEA2000_esp32(CAN_TX_PIN, CAN_RX_PIN);
  n2k->SetProductInformation("00000001");
  n2k->SetDeviceInformation(1, // Unique number. Use e.g. Serial number.
                                130, // Device function=Temperature. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                75, // Device class=Sensor Communication Interface. See codes on  http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                2040 // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf                               
                               );

  n2k->SetForwardStream(&Serial);
  n2k->SetForwardType(tNMEA2000::fwdt_Text);
  n2k->SetMode(tNMEA2000::N2km_ListenAndNode, 1);
  n2k->EnableForward(false);
  n2k->ExtendTransmitMessages(TransmitMessages);
  n2k->SetMsgHandler(HandleNMEA2000Msg);
  n2k->Open();

  // Set up i2c
  i2c = new TwoWire(0);
  i2c->begin(SDA_PIN, SCL_PIN);

  // Set up BME280 sensor using i2c
  if (!bme.begin(0x76, i2c)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
  }
  bme.setTemperatureCompensation(-1.0);

  // Set up DS18B20 temperature sensor using 1-wire
  ds = new DallasTemperature(&oneWire);
  ds->begin();

  // Set up GPIO pins
  pinMode(DC_RELAY_1_PIN, OUTPUT);
  pinMode(DC_RELAY_2_PIN, OUTPUT);
  pinMode(DC_RELAY_3_PIN, INPUT_PULLDOWN);
  pinMode(AC_RELAY_PIN, INPUT_PULLDOWN);
  pinMode(RPI_POWER_STATE_PIN, INPUT_PULLDOWN);
  pinMode(RPI_SHUTDOWN_PIN, OUTPUT);

  // Parse incoming messages every 1ms
  app.onRepeat(1, []() {
    n2k->ParseMessages();
  });

  app.onRepeat(3000, []() {
    if (digitalRead(RPI_POWER_STATE_PIN) == HIGH) {
      Serial.print("Screen on\n");
    } else {
      Serial.print("Screen off\n");
    }
  });

  // Send data every 2,5 sec
  app.onRepeat(2500, []() {
    SendEnclosureTemperature();
    SendEnvironmentData();
    SendSwitchBankStatus('1');
    SendSwitchBankStatus('2');
    Serial.println();
  });
}

void loop() {
  app.tick();
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

// NMEA2000 message senders

void SendEnclosureTemperature() {
  tN2kMsg N2kMsg;

  SetN2kTemperature(N2kMsg, 1, 1, N2kts_InsideTemperature, ReadEnclosureTemp());
  if (n2k->SendMsg(N2kMsg)) {
    Serial.print(millis()); Serial.println(", Enclosure temperature send ready");
  } else {
    Serial.print(millis()); Serial.println(", Enclosure temperature send failed");
  }
}

void SendEnvironmentData() {
  tN2kMsg N2kMsg;

  SetN2kEnvironmentalParameters(N2kMsg, 1, N2kts_EngineRoomTemperature, ReadCabinTemp(), N2khs_InsideHumidity, ReadHumidity(), ReadPressure());
  if (n2k->SendMsg(N2kMsg)) {
    Serial.print(millis()); Serial.println(", Environment data send ready");
  } else {
    Serial.print(millis()); Serial.println(", Environment data send failed");
  }
}

void SendSwitchBankStatus(unsigned char DeviceBankInstance) {
  tN2kMsg N2kMsg;

  switch (DeviceBankInstance) {
    case '1':
      SetN2kBinaryStatus(N2kMsg, DeviceBankInstance,
        (digitalRead(DC_RELAY_1_PIN) == HIGH) ? N2kOnOff_On : N2kOnOff_Off,
        (digitalRead(DC_RELAY_2_PIN) == HIGH) ? N2kOnOff_On : N2kOnOff_Off,
        (digitalRead(DC_RELAY_3_PIN) == HIGH) ? N2kOnOff_On : N2kOnOff_Off,
        (digitalRead(AC_RELAY_PIN) == HIGH) ? N2kOnOff_On : N2kOnOff_Off);
      break;
    case '2':
      SetN2kBinaryStatus(N2kMsg, DeviceBankInstance,
        (digitalRead(RPI_POWER_STATE_PIN) == HIGH) ? N2kOnOff_On : N2kOnOff_Off);
  }

  if (n2k->SendMsg(N2kMsg)) {
    Serial.print(millis()); Serial.printf(", Binary switch bank %c status send ready\n", DeviceBankInstance);
  } else {
    Serial.print(millis()); Serial.printf(", Binary switch bank %c status send failed\n", DeviceBankInstance);
  }
}

// NMEA2000 incoming message handlers

void SwitchBankStatusHandler(const tN2kMsg &inboundMsg) {
  unsigned char BankInstance;
  tN2kOnOff Status1,Status2,Status3,Status4;

  if (ParseN2kBinaryStatus(inboundMsg,BankInstance,Status1,Status2,Status3,Status4) ) {
    switch (BankInstance) {
      case 1:
        if (Status1 == N2kOnOff_On || Status1 == N2kOnOff_Off) digitalWrite(DC_RELAY_1_PIN, (Status1 == N2kOnOff_On) ? HIGH : LOW);
        if (Status2 == N2kOnOff_On || Status2 == N2kOnOff_Off) digitalWrite(DC_RELAY_2_PIN, (Status2 == N2kOnOff_On) ? HIGH : LOW);
        break;
      case 2:
        if (Status1 == N2kOnOff_On || Status1 == N2kOnOff_Off) digitalWrite(RPI_SHUTDOWN_PIN, (Status1 == N2kOnOff_On) ? HIGH : LOW);
        break;
    }
    delay(2);
    SendSwitchBankStatus(BankInstance);
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
