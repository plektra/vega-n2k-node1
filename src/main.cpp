#include <Arduino.h>
#include <esp_https_ota.h>
#include <esp_task_wdt.h>

extern const uint8_t S3CA[] asm("_binary_src_s3_ca_pem_start");

#define ESP32_CAN_TX_PIN GPIO_NUM_22
#define ESP32_CAN_RX_PIN GPIO_NUM_21

#include <NMEA2000_CAN.h>
#include <N2kMessages.h>
#include <N2kMessagesEnumToStr.h>

#include <DHT.h>
#define DHT_PIN 15
DHT dht(DHT_PIN, DHT22);

#define AC_RELAY_PIN 19

#define DC_RELAY_PIN 18

const unsigned long TransmitMessages[] PROGMEM={
  130311L,
  127501L,
  0
};

typedef struct {
  unsigned long PGN;
  void (*Handler)(const tN2kMsg &N2kMsg); 
} tNMEA2000Handler;

void SendN2kACState();
void SendN2kTemperature();
void BinaryStatus(const tN2kMsg &N2kMsg);
void HandleNMEA2000Msg(const tN2kMsg &N2kMsg);

tNMEA2000Handler NMEA2000Handlers[]={
  {127501L,&BinaryStatus},
  {0,0}
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
  NMEA2000.SetMsgHandler(HandleNMEA2000Msg);
  NMEA2000.Open();

  pinMode(AC_RELAY_PIN, INPUT_PULLDOWN);

  pinMode(DC_RELAY_PIN, OUTPUT);
  digitalWrite(DC_RELAY_PIN, HIGH);
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

void controlDCRelay(bool toggle) {
  digitalWrite(DC_RELAY_PIN, !toggle);
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

void BinaryStatus(const tN2kMsg &inboundMsg) {
  unsigned char BankInstance;
  tN2kOnOff Status1,Status2,Status3,Status4;
  tN2kMsg outboundMsg;

  if (ParseN2kBinaryStatus(inboundMsg,BankInstance,Status1,Status2,Status3,Status4) ) {
    controlDCRelay((Status2 == N2kOnOff_On) ? true : false);
  }
  SetN2kBinaryStatus(outboundMsg, BankInstance, Status1, Status2, Status3, Status4);
  NMEA2000.SendMsg(outboundMsg);
}

void HandleNMEA2000Msg(const tN2kMsg &N2kMsg) {
  int iHandler;
  
  // Find handler
  Serial.print("In Main Handler: "); Serial.println(N2kMsg.PGN);
  for (iHandler=0; NMEA2000Handlers[iHandler].PGN!=0 && !(N2kMsg.PGN==NMEA2000Handlers[iHandler].PGN); iHandler++);
  
  if (NMEA2000Handlers[iHandler].PGN!=0) {
    NMEA2000Handlers[iHandler].Handler(N2kMsg); 
  }
}

esp_err_t doUpdate() {
  esp_http_client_config_t http_conf = {
    .url = "https://vega-ota.s3.eu-north-1.amazonaws.com/vega-n2k-node1/firmware-latest.bin",
    .host = NULL,
    .port = 443,
    .username = NULL,
    .password = NULL,
    .auth_type = HTTP_AUTH_TYPE_NONE,
    .path = NULL,
    .query = NULL,
    .cert_pem = (char *)S3CA,
  };

  esp_task_wdt_init(15,0);
  esp_err_t ret = esp_https_ota(&http_conf);
  esp_task_wdt_init(5,0);
  if (ret == ESP_OK) {
    Serial.println("Update successful!");
    esp_restart();
  } else {
    Serial.println("Update failed!");
    return ESP_FAIL;
  }
  return ESP_OK;
}