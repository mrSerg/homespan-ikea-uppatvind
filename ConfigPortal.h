#pragma once

#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <HomeSpan.h>
#include <qrcode.h>

const char *AP_SSID = "IKEA-Uppatvind";
const char *AP_PASSWORD = "uppatvind";

struct DeviceConfig {
  String ssid;
  String password;
  String pairingCode;
  bool pairingCodeApplied;
};

Preferences configStore;

DeviceConfig loadConfig() {
  configStore.begin("uppatvind", true);
  DeviceConfig config = {
    configStore.getString("ssid", ""),
    configStore.getString("password", ""),
    configStore.getString("code", ""),
    configStore.getBool("codeApplied", false),
  };
  configStore.end();
  return config;
}

void saveConfig(const String &ssid, const String &password, const String &pairingCode) {
  configStore.begin("uppatvind", false);
  configStore.putString("ssid", ssid);
  configStore.putString("password", password);
  configStore.putString("code", pairingCode);
  configStore.putBool("codeApplied", false);
  configStore.end();
}

void markPairingCodeApplied() {
  configStore.begin("uppatvind", false);
  configStore.putBool("codeApplied", true);
  configStore.end();
}

void clearConfig() {
  configStore.begin("uppatvind", false);
  configStore.clear();
  configStore.end();
}

WebServer portalServer(80);
DNSServer portalDns;
String portalPairingCode;
String portalNetworkOptions;

bool isAllowedPairingCode(const String &code) {
  if (code.length() != 8 || code == "12345678" || code == "87654321") return false;
  for (int i = 0; i < 8; i++) {
    if (!isDigit(code[i])) return false;
  }
  for (int i = 1; i < 8; i++) {
    if (code[i] != code[0]) return true;
  }
  return false;
}

String generatePairingCode() {
  char code[9];
  do {
    snprintf(code, sizeof(code), "%08lu", (unsigned long)(esp_random() % 100000000UL));
  } while (!isAllowedPairingCode(code));
  return code;
}

String formatPairingCode(const String &code) {
  return code.substring(0, 3) + "-" + code.substring(3, 5) + "-" + code.substring(5);
}

String *qrSvgTarget = nullptr;

void appendQrSvg(esp_qrcode_handle_t qrcode) {
  int size = esp_qrcode_get_size(qrcode);
  String path;
  for (int y = 0; y < size; y++) {
    for (int x = 0; x < size; x++) {
      if (esp_qrcode_get_module(qrcode, x, y)) path += "M" + String(x) + " " + String(y) + "h1v1h-1z";
    }
  }
  *qrSvgTarget += "<svg class=qr viewBox='-4 -4 " + String(size + 8) + " " + String(size + 8) +
    "' shape-rendering=crispEdges><rect x=-4 y=-4 width=100% height=100% fill=#fff /><path d='" + path + "'/></svg>";
}

String pairingQrSvg(const String &code) {
  String svg;
  qrSvgTarget = &svg;
  esp_qrcode_config_t config = {appendQrSvg, 10, ESP_QRCODE_ECC_MED};
  esp_qrcode_generate(&config, HapQR().get(code.toInt(), DEFAULT_QR_ID, (uint8_t)Category::AirPurifiers));
  qrSvgTarget = nullptr;
  return svg;
}

String htmlEscape(const String &text) {
  String escaped;
  for (char c : text) {
    switch (c) {
      case '&': escaped += "&amp;"; break;
      case '<': escaped += "&lt;"; break;
      case '>': escaped += "&gt;"; break;
      case '"': escaped += "&quot;"; break;
      default: escaped += c;
    }
  }
  return escaped;
}

const char *PORTAL_HEAD =
  "<!doctype html><html><head><meta charset=utf-8>"
  "<meta name=viewport content='width=device-width,initial-scale=1'>"
  "<title>IKEA Uppatvind</title><style>"
  "body{font-family:-apple-system,system-ui,sans-serif;max-width:420px;margin:24px auto;padding:0 16px;color:#111}"
  "h2{font-size:18px;margin:24px 0 8px}"
  ".code{font-size:40px;font-weight:600;letter-spacing:2px;text-align:center;margin:12px 0}"
  ".qr{display:block;width:220px;margin:0 auto}"
  ".warning{font-weight:700;text-align:center;color:#c00;margin:12px 0}"
  "input,button{width:100%;font-size:17px;padding:12px;margin:6px 0;box-sizing:border-box;border-radius:8px;border:1px solid #bbb}"
  "button{background:#0071e3;color:#fff;border:0}.secondary{background:#e8e8ed;color:#111}"
  "small{display:block;color:#666;margin-top:4px}"
  "</style></head><body>";

String pairingCodeBlock(const String &code) {
  return pairingQrSvg(code) +
    "<div class=code>" + formatPairingCode(code) + "</div>"
    "<div class=warning>SAVE THIS BEFORE CONNECTING</div>"
    "<small>Take a screenshot or write the code down. After the purifier joins your Wi-Fi, "
    "add it in the Home app by scanning the QR code or entering the code.</small>";
}

String portalPage() {
  return String(PORTAL_HEAD) +
    "<h2>Apple Home setup code</h2>" + pairingCodeBlock(portalPairingCode) +
    "<form method=post action=/new-code><button class=secondary>Generate new code</button></form>"
    "<h2>Wi-Fi network</h2>"
    "<form method=post action=/save>"
    "<input type=hidden name=code value=" + portalPairingCode + ">"
    "<input name=ssid list=networks placeholder='Network name' autocapitalize=none required>"
    "<datalist id=networks>" + portalNetworkOptions + "</datalist>"
    "<input name=password type=password placeholder=Password>"
    "<button>Save and connect</button></form></body></html>";
}

void handlePortalSave() {
  String ssid = portalServer.arg("ssid");
  String code = portalServer.arg("code");
  if (ssid.isEmpty() || !isAllowedPairingCode(code)) {
    portalServer.sendHeader("Location", "/");
    portalServer.send(303);
    return;
  }

  saveConfig(ssid, portalServer.arg("password"), code);
  portalServer.send(200, "text/html", String(PORTAL_HEAD) +
    "<h2>Saved</h2><p>The purifier is connecting to <b>" + htmlEscape(ssid) + "</b>.</p>"
    "<p>Add it in the Home app with this code:</p>" + pairingCodeBlock(code) +
    "<small>Wrong Wi-Fi password? Hold the purifier button for 5 seconds to start over.</small></body></html>");
  delay(1000);
  ESP.restart();
}

void startConfigPortal() {
  portalPairingCode = generatePairingCode();

  WiFi.mode(WIFI_STA);
  int count = WiFi.scanNetworks();
  for (int i = 0; i < count; i++) {
    portalNetworkOptions += "<option value=\"" + htmlEscape(WiFi.SSID(i)) + "\">";
  }
  WiFi.scanDelete();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  portalDns.start(53, "*", WiFi.softAPIP());

  portalServer.on("/", HTTP_GET, []() { portalServer.send(200, "text/html", portalPage()); });
  portalServer.on("/new-code", HTTP_POST, []() {
    portalPairingCode = generatePairingCode();
    portalServer.sendHeader("Location", "/");
    portalServer.send(303);
  });
  portalServer.on("/save", HTTP_POST, handlePortalSave);
  // Redirecting every unknown URL makes phones open the page automatically as a captive portal
  portalServer.onNotFound([]() {
    portalServer.sendHeader("Location", "http://" + WiFi.softAPIP().toString() + "/");
    portalServer.send(302);
  });
  portalServer.begin();

  Serial.printf("Config portal started: Wi-Fi \"%s\", password \"%s\"\n", AP_SSID, AP_PASSWORD);
}

void handleConfigPortal() {
  portalDns.processNextRequest();
  portalServer.handleClient();
}
