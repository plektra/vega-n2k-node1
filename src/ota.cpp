#include "ota.h"

//extern const uint8_t S3CA[] asm("_binary_src_s3_ca_pem_start");

void OTAUpdate() {
  const size_t strLen = 32;
  const int SSIDPos = 0;
  const int passwordPos = 32;
  const int nodePos = 64;
  const int versionPos = 96;
  char SSID[strLen];
  char password[strLen];
  char node[strLen]; // e.g. vega-n2k-node1
  char currentVersion[16]; // semver
  char latestVersion[16];
  // semver_t currentVersionSemVer = {};
  // semver_t latestVersionSemVer = {};

  const char *imageUrlFormat = "https://vega-ota.s3.eu-north-1.amazonaws.com/%s/%s";
  char imageUrl[128];

  Serial.println("OTA update routine triggered.");

  EEPROM.begin(512);
  readStr(SSIDPos, SSID, strLen);
  readStr(passwordPos, password, strLen);
  readStr(nodePos, node, strLen);
  readStr(versionPos, currentVersion, 16);
  // semver_parse(currentVersion, &currentVersionSemVer);

  connectWiFi(SSID, password);

  getLatestVersion(imageUrlFormat, node, latestVersion);
  // int c = strcmp(currentVersion, latestVersion);
  // Serial.printf("Current: %s\nLatest: %s\nCompare: %i\n", currentVersion, latestVersion, c);

  if (strcmp(currentVersion, latestVersion) == 0) {
    Serial.printf("Current version %s is the latest, disconnecting WiFi and skipping update.\n", currentVersion);
    WiFi.disconnect();
    return;
  }
  // semver_parse(latestVersion, &latestVersionSemVer);
  // if (semver_compare(latestVersionSemVer, currentVersionSemVer) == 0) {
  //   Serial.printf("Current version %s is the latest, rebooting.\n", currentVersion);
  //   esp_restart();
  // } else {
  //   semver_free(&currentVersionSemVer);
  //   semver_free(&latestVersionSemVer);
  // }

  char imageName[32];
  sprintf(imageName, "firmware-%s.bin", latestVersion);
  sprintf(imageUrl, imageUrlFormat, node, imageName);

  if (doUpdate(imageUrl) == ESP_OK) {
    Serial.printf("Update successful! Writing new version %s to flash and rebooting.\n", latestVersion);
    writeStr(latestVersion, versionPos);
    esp_restart();
  } else {
    Serial.println("Update failed, rebooting.");
    esp_restart();
  }
}

void connectWiFi(char *SSID, char *password) {
  Serial.print("Connecting WiFi");

  WiFi.begin(SSID, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print('.');
  }

  Serial.print("\nConnected, IP address: "); Serial.println(WiFi.localIP());
}

void getLatestVersion(const char *imageUrlFormat, char *node, char *latestVersion) {
  char versionUrl[128];
  sprintf(versionUrl, imageUrlFormat, node, "latest.txt");

  Serial.printf("Fetching latest version number from %s\n", versionUrl);

  HTTPClient https;

  if (https.begin(versionUrl, (char *)S3CA)) {
    int httpCode = https.GET();

    if (httpCode == HTTP_CODE_OK) {
      String ver = https.getString();
      ver.trim();
      ver.toCharArray(latestVersion, 16);
      Serial.printf("Got version %s\n", latestVersion);
    } else {
      Serial.printf("Version fetch failed, HTTP code %i\n", httpCode);
    }
  }
}

boolean doUpdate(char *imageUrl) {
  esp_http_client_config_t http_conf = {
    .url = imageUrl,
    .host = NULL,
    .port = 443,
    .username = NULL,
    .password = NULL,
    .auth_type = HTTP_AUTH_TYPE_NONE,
    .path = NULL,
    .query = NULL,
    .cert_pem = (char *)S3CA,
  };

  Serial.println("OTA update started");
  esp_task_wdt_init(15,0);
  esp_err_t ret = esp_https_ota(&http_conf);
  esp_task_wdt_init(5,0);

  return ret;
}

void readStr(int offset, char *data, const size_t len) {
  for (int i=0; i<len; i++) data[i] = EEPROM.read(offset+i);
}

void writeStr(char *data, int offset) {
  int i=0;
  for (; i<strlen(data); i++) {
    EEPROM.write(offset+i, data[i]);
  }
  EEPROM.write(offset+i, '\0');
  EEPROM.commit();
}