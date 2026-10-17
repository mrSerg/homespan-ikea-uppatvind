#pragma once

#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <Update.h>
#include <HomeSpan.h>
#include <qrcode.h>
#include <vector>
#include <algorithm>

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
bool portalScanning = false;

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
  "<title>UPPÅTVIND setup</title><style>"
  "*{box-sizing:border-box}"
  "body{margin:0;background:#fff;color:#111;font:16px/1.5 'Noto Sans',-apple-system,system-ui,sans-serif}"
  "header{background:#0058a3;color:#fff;padding:20px 20px 16px;border-bottom:6px solid #ffdb00}"
  "header b{display:block;font-size:24px;font-weight:800;letter-spacing:.04em}"
  "header span{font-size:14px;opacity:.85}"
  "main{max-width:440px;margin:0 auto;padding:8px 20px 40px}"
  "h2{font-size:20px;font-weight:700;margin:32px 0 12px;display:flex;align-items:center;gap:10px}"
  "h2 i{font-style:normal;width:28px;height:28px;border-radius:50%;background:#111;color:#fff;font-size:14px;display:grid;place-items:center}"
  ".qr{display:block;width:200px;margin:8px auto;border:1px solid #dfdfdf;padding:8px}"
  ".code{font-size:36px;font-weight:700;letter-spacing:.06em;text-align:center;font-variant-numeric:tabular-nums}"
  ".warning{background:#ffdb00;font-weight:800;text-align:center;padding:12px;margin:12px 0}"
  "p,small{color:#484848}small{display:block;font-size:14px}"
  "input,select{width:100%;height:56px;font-size:16px;padding:0 16px;margin:6px 0;border:1px solid #929292;border-radius:4px;background:#fff;color:#111;-webkit-appearance:none;appearance:none}"
  "select{background:#fff url(\"data:image/svg+xml,%3Csvg xmlns='http://www.w3.org/2000/svg' width='12' height='8'%3E%3Cpath d='M1 1l5 5 5-5' fill='none' stroke='%23111' stroke-width='2'/%3E%3C/svg%3E\") no-repeat right 16px center}"
  "input:focus,select:focus{outline:0;border:2px solid #0058a3}"
  "label{display:flex;align-items:center;gap:10px;margin:10px 0;font-size:14px}"
  "label input{width:20px;height:20px;margin:0;-webkit-appearance:checkbox;appearance:auto;accent-color:#0058a3}"
  "button{width:100%;height:56px;font-size:16px;font-weight:700;margin:8px 0;border:1px solid #111;border-radius:64px;background:#111;color:#fff}"
  "button.secondary{background:#fff;color:#111;border-color:#dfdfdf}"
  "input[type=file]{height:auto;padding:14px 16px}"
  "hr{border:0;border-top:1px solid #dfdfdf;margin:40px 0 0}"
  "</style></head><body><header><b>UPPÅTVIND</b><span>Apple Home setup</span></header><main>";

const char *PORTAL_FOOT = "</main></body></html>";

String pairingCodeBlock(const String &code) {
  return pairingQrSvg(code) +
    "<div class=code>" + formatPairingCode(code) + "</div>"
    "<div class=warning>SAVE THIS BEFORE CONNECTING</div>"
    "<small>Take a screenshot or write the code down. After the purifier joins your Wi-Fi, "
    "add it in the Home app by scanning the QR code or entering the code.</small>";
}

String networkSelectOptions() {
  if (portalScanning) {
    return "<option disabled selected>Searching for networks...</option><option value=''>Other network...</option>";
  }
  return portalNetworkOptions + "<option value=''" + (portalNetworkOptions.isEmpty() ? " selected" : "") + ">Other network...</option>";
}

const char *NETWORKS_POLL_SCRIPT =
  "<script>(function poll(){fetch('/networks').then(r=>r.status==200"
  "?r.text().then(h=>{network.innerHTML=h;other.hidden=network.value!=''})"
  ":setTimeout(poll,1000))})()</script>";

String portalPage() {
  return String(PORTAL_HEAD) +
    "<h2><i>1</i>Save your setup code</h2>" + pairingCodeBlock(portalPairingCode) +
    "<form method=post action=/new-code><button class=secondary>Generate new code</button></form>"
    "<h2><i>2</i>Connect to Wi-Fi</h2>"
    "<form method=post action=/save>"
    "<input type=hidden name=code value=" + portalPairingCode + ">"
    "<select id=network name=network onchange=\"other.hidden=this.value!=''\">" + networkSelectOptions() + "</select>"
    "<input id=other name=other_ssid placeholder='Network name' autocapitalize=none" +
    (portalNetworkOptions.isEmpty() && !portalScanning ? "" : " hidden") + ">"
    "<input id=password name=password type=password placeholder=Password>"
    "<label><input type=checkbox onclick=\"password.type=this.checked?'text':'password'\">Show password</label>"
    "<button>Save and connect</button></form>"
    "<form method=post action=/scan><button class=secondary>Scan again</button></form>"
    "<hr><h2>Firmware update</h2>"
    "<form method=post action=/update enctype=multipart/form-data>"
    "<input type=file name=firmware accept=.bin required>"
    "<small>Firmware .bin file (Arduino IDE: Sketch → Export Compiled Binary).</small>"
    "<button class=secondary>Upload and install</button></form>" +
    (portalScanning ? NETWORKS_POLL_SCRIPT : "") + PORTAL_FOOT;
}

void handlePortalSave() {
  String ssid = portalServer.arg("network");
  if (ssid.isEmpty()) ssid = portalServer.arg("other_ssid");
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
    "<p>Wrong Wi-Fi password? Hold the purifier button for 5 seconds to start over.</p>" + PORTAL_FOOT);
  delay(1000);
  ESP.restart();
}

void handleFirmwareUpload() {
  HTTPUpload &upload = portalServer.upload();
  if (upload.status == UPLOAD_FILE_START) {
    Serial.printf("Firmware update: %s\n", upload.filename.c_str());
    Update.begin(UPDATE_SIZE_UNKNOWN);
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    Update.write(upload.buf, upload.currentSize);
  } else if (upload.status == UPLOAD_FILE_END) {
    Update.end(true);
  } else if (upload.status == UPLOAD_FILE_ABORTED) {
    Update.abort();
  }
}

void handleFirmwareUploadDone() {
  if (Update.hasError()) {
    portalServer.send(200, "text/html", String(PORTAL_HEAD) +
      "<h2>Update failed</h2><p>" + htmlEscape(Update.errorString()) + "</p>"
      "<form action=/><button>Back</button></form>" + PORTAL_FOOT);
    return;
  }
  portalServer.send(200, "text/html", String(PORTAL_HEAD) +
    "<h2>Update installed</h2><p>The purifier is restarting. Reconnect to the setup Wi-Fi in a few seconds.</p>" + PORTAL_FOOT);
  delay(1000);
  ESP.restart();
}

void startNetworkScan() {
  WiFi.scanNetworks(true);
  portalScanning = true;
}

void collectScanResults(int count) {
  std::vector<int> order;
  for (int i = 0; i < count; i++) order.push_back(i);
  std::sort(order.begin(), order.end(), [](int a, int b) { return WiFi.RSSI(a) > WiFi.RSSI(b); });

  std::vector<String> seen;
  portalNetworkOptions = "";
  for (int i : order) {
    String ssid = WiFi.SSID(i);
    if (ssid.isEmpty() || std::find(seen.begin(), seen.end(), ssid) != seen.end()) continue;
    seen.push_back(ssid);
    portalNetworkOptions += "<option>" + htmlEscape(ssid) + "</option>";
  }
  WiFi.scanDelete();
  portalScanning = false;
}

void startConfigPortal() {
  portalPairingCode = generatePairingCode();

  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  startNetworkScan();
  portalDns.start(53, "*", WiFi.softAPIP());

  portalServer.on("/", HTTP_GET, []() { portalServer.send(200, "text/html", portalPage()); });
  portalServer.on("/new-code", HTTP_POST, []() {
    portalPairingCode = generatePairingCode();
    portalServer.sendHeader("Location", "/");
    portalServer.send(303);
  });
  portalServer.on("/networks", HTTP_GET, []() {
    if (portalScanning) portalServer.send(204);
    else portalServer.send(200, "text/html", networkSelectOptions());
  });
  portalServer.on("/scan", HTTP_POST, []() {
    if (!portalScanning) startNetworkScan();
    portalServer.sendHeader("Location", "/");
    portalServer.send(303);
  });
  portalServer.on("/save", HTTP_POST, handlePortalSave);
  portalServer.on("/update", HTTP_POST, handleFirmwareUploadDone, handleFirmwareUpload);
  // Redirecting every unknown URL makes phones open the page automatically as a captive portal
  portalServer.onNotFound([]() {
    portalServer.sendHeader("Location", "http://" + WiFi.softAPIP().toString() + "/");
    portalServer.send(302);
  });
  portalServer.begin();

  Serial.printf("Config portal started: Wi-Fi \"%s\", password \"%s\"\n", AP_SSID, AP_PASSWORD);
}

void handleConfigPortal() {
  int scanResult = WiFi.scanComplete();
  if (portalScanning && scanResult != WIFI_SCAN_RUNNING) collectScanResults(max(scanResult, 0));
  portalDns.processNextRequest();
  portalServer.handleClient();
}
