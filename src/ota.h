#include <Arduino.h>
#include <EEPROM.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <esp_https_ota.h>
#include <esp_task_wdt.h>

extern const uint8_t S3CA[] asm("_binary_src_s3_ca_pem_start");

boolean OTAUpdate();
boolean connectWiFi(char *SSID, char *password);
boolean getLatestVersion(const char *imageUrlFormat, char *node, char *latestVersion);
void readStr(int offset, char *data, size_t len);
void writeStr(char *data, int offset);
boolean doUpdate(char *imageUrl);