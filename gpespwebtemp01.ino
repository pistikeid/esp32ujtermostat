/*
  esp_heating_controller.ino
  - ESP32, 2x DS18x20 OneWire
  - WebServer + WiFiManager + MDNS
  - EEPROM tárolás (thresholds, check interval, preRun, manual pump states)
  - OneWire ROM CRC + scratchpad CRC ellenőrzés
  - Manuális szivattyú kapcsolók (padló / fal) – párhuzamos, "force ON"
  - Non-blocking állapotgép: IDLE -> PRE_RUN -> RUNNING
*/

#include <WebServer.h>
#include <WiFiManager.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <EEPROM.h>
#include <ESPmDNS.h>

MDNSResponder mdns;

// === HARDWARE ===
#define ONE_WIRE_BUS 4   // OneWire GPIO
const int outputp = 2;   // padló pumpa
const int outputf = 15;  // fal pumpa
const int outputk = 16;  // kazán

// === EEPROM helpers ===
// Simple string write/read like earlier
int writeStringToEEPROM(int addrOffset, const String &strToWrite) {
  byte len = strToWrite.length();
  if (len > 255) len = 255;
  EEPROM.write(addrOffset, len);
  EEPROM.commit();
  for (int i = 0; i < len; i++) {
    EEPROM.write(addrOffset + 1 + i, strToWrite[i]);
  }
  EEPROM.commit();
  return addrOffset + 1 + len;
}
int readStringFromEEPROM(int addrOffset, String *strToRead) {
  byte newStrLen = EEPROM.read(addrOffset);
  if (newStrLen == 0xFF) { // not initialized
    *strToRead = String("");
    return addrOffset + 1;
  }
  if (newStrLen == 0) { *strToRead = String(""); return addrOffset + 1; }
  char data[newStrLen + 1];
  for (int i = 0; i < newStrLen; i++) {
    data[i] = EEPROM.read(addrOffset + 1 + i);
  }
  data[newStrLen] = '\0';
  *strToRead = String(data);
  return addrOffset + 1 + newStrLen;
}

// === STATE / CONFIG VARIABLES ===
// Web / state variables (defaults)
String inputMessage = "21.0";     // padló threshold (C)
String inputMessagef = "21.0";    // fal threshold (C)
String inputMessagecheck = "60";  // check interval (sec) default
unsigned long intervalp = 60000;  // computed from check
unsigned long intervalf = 60000;
unsigned long previousMillis = 0;
const long intervalDefault = 1000; // general loop tick
bool inputMessage2_enabled = true; // padló enabled
bool inputMessage4_enabled = true; // fal enabled

// pre-run (pump warmup) — állítható, mentve EEPROM
unsigned int preRunSeconds = 60; // default 60s

// manual override states (force ON)
bool manualPadlo = false;
bool manualFal = false;

// Ugyanaz string formában mentéshez
String manualPadloStr = "false";
String manualFalStr = "false";

// last manual for edge detection
bool lastManualPadlo = false;
bool lastManualFal = false;

// kazán/pumpa control flags (strings kept for compatibility)
String kazanp = "false";
String kazanf = "false";
String kazankapcsp = "false";
String kazankapcsf = "false";

// temperature display strings
String lastTemperature = "--";
String lastTemperaturef = "--";

// OneWire / Dallas
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// Addresses for 2 sensors (padló=index0, fal=index1)
DeviceAddress addrPadlo;
DeviceAddress addrFal;
bool sensorPadloFound = false;
bool sensorFalFound = false;

// server
WebServer server(80);

// Remaining seconds to display (for UI)
long remainingPadlo = 0;
long remainingFal = 0;

// State machine per zone
enum ZoneState { Z_IDLE=0, Z_PRE_RUN=1, Z_RUNNING=2 };

ZoneState statePadlo = Z_IDLE;
ZoneState stateFal = Z_IDLE;
unsigned long stateStartPadlo = 0;
unsigned long stateStartFal = 0;

// timing for pump auto timers (non-blocking)
unsigned long previousMillisp = 0;
unsigned long previousMillisf = 0;
bool triggertimer = true;
bool triggertimerf = true;

// helper for invalid reading detection
const float INVALID_TEMP = -127.0;

// === HTML template (UTF-8) ===
const char* index_html = R"rawliteral(
<!DOCTYPE HTML><html><head><meta charset="UTF-8"><title>teszt</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>body{font-family:Arial;padding:8px;background:#fafafa;color:#222}h2{margin:6px 0}input[type=number]{width:120px;padding:4px}.row{margin:6px 0}.small{font-size:0.9em;color:#555}</style>
</head><body>
<h2>teszt (DS18x20)</h2>
<h3>Padló</h3>
<div id="temperature-padlo">%TEMPERATURE% &deg;C</div>
<div>Következő mérésig: <span id="remaining-padlo">--</span> s</div>
<form action="/get" method="get">
<div class="row">Padló küszöb: <input type="number" step="0.1" name="threshold_input" value="%THRESHOLD%" required></div>
<div class="row">Fűtés engedélyezés: <input type="checkbox" name="enable_arm_input" value="true" %ENABLE_ARM_INPUT%></div>
<div class="row">Szivattyú kézi (Manual): <input type="checkbox" name="manual_padlo" value="true" %MANUAL_PADLO%></div>

<h3>Fal</h3>
<div id="temperature-fal">%TEMPERATUREF% &deg;C</div>
<div>Következő mérésig: <span id="remaining-fal">--</span> s</div>
<div class="row">Fal küszöb: <input type="number" step="0.1" name="threshold_inputf" value="%THRESHOLDF%" required></div>
<div class="row">Fűtés engedélyezés: <input type="checkbox" name="enable_arm_inputf" value="true" %ENABLE_ARM_INPUTF%></div>
<div class="row">Szivattyú kézi (Manual): <input type="checkbox" name="manual_fal" value="true" %MANUAL_FAL%></div>

<h3>Paraméterek</h3>
<div class="row">Check idő (s): <input type="number" step="1" name="check" value="%CHECK%" required></div>
<div class="row">Pumpa előfutási idő (s): <input type="number" step="1" name="prerun" value="%PRERUN%" required></div>

<div class="row"><input type="submit" value="Mentés"></div>
</form>

<h3>Állapotok</h3>
<div>Padló fűtés állapota: <span id="padlo-allapot">%PADLO_ALLAPOT%</span></div>
<div>Fal fűtés állapota: <span id="fal-allapot">%FAL_ALLAPOT%</span></div>
<div>Kazán állapota: <span id="kazan-allapot">%KAZAN_ALLAPOT%</span></div>
<div class="small">WiFi hálózat: <span id="wifissid">%SSID%</span></div>
<div class="small">WiFi jel: <span id="wifijel">%WIFIJEL%</span> dBm</div>

<script>
setInterval(function(){
  var xhttp=new XMLHttpRequest();
  xhttp.onreadystatechange=function(){ if(this.readyState==4 && this.status==200){
    var r=JSON.parse(this.responseText);
    document.getElementById("temperature-padlo").innerHTML = r.padlo + " &deg;C";
    document.getElementById("temperature-fal").innerHTML = r.fal + " &deg;C";
    document.getElementById("remaining-padlo").innerHTML = r.remainingPadlo;
    document.getElementById("remaining-fal").innerHTML = r.remainingFal;
  }};
  xhttp.open("GET","/temperature",true); xhttp.send();
},2000);

setInterval(function(){
  var xhttp=new XMLHttpRequest();
  xhttp.onreadystatechange=function(){ if(this.readyState==4 && this.status==200){
    var r=JSON.parse(this.responseText);
    document.getElementById("padlo-allapot").innerHTML = r.padloState;
    document.getElementById("fal-allapot").innerHTML = r.falState;
    document.getElementById("kazan-allapot").innerHTML = r.kazan;
    document.getElementById("wifijel").innerHTML = r.wifijel;
    // Az SSID nem változik futás közben, nem kell ide frissítés, de itt van példának:
    // document.getElementById("wifissid").innerHTML = r.ssid; 
  }};
  xhttp.open("GET","/allapot",true); xhttp.send();
},1000);
</script>

</body></html>
)rawliteral";

// === Forward decls ===
void initEEPROMAndLoad();
void saveAllToEEPROM();
bool readScratchpadAndCheckCRC(DeviceAddress addr, uint8_t *outScratch);
String zoneStateToText(ZoneState s);

// === Implementation ===

void initEEPROMAndLoad() {
  EEPROM.begin(512);
  int offset = 0;
  String s1, s2, s3, s4, s5, s6;
  offset = readStringFromEEPROM(offset, &s1); // threshold padlo
  offset = readStringFromEEPROM(offset, &s2); // threshold fal
  offset = readStringFromEEPROM(offset, &s3); // check interval
  offset = readStringFromEEPROM(offset, &s4); // prerun seconds
  offset = readStringFromEEPROM(offset, &s5); // manualPadlo
  offset = readStringFromEEPROM(offset, &s6); // manualFal

  if (s1.length()) inputMessage = s1;
  if (s2.length()) inputMessagef = s2;
  if (s3.length()) inputMessagecheck = s3;
  if (s4.length()) preRunSeconds = (unsigned int) s4.toInt();
  if (s5.length()) manualPadloStr = s5;
  if (s6.length()) manualFalStr = s6;

  manualPadlo = (manualPadloStr == "true");
  manualFal  = (manualFalStr == "true");

  intervalp = inputMessagecheck.toInt() * 1000UL;
  intervalf = intervalp;
}

void saveAllToEEPROM() {
  int offset = 0;
  offset = writeStringToEEPROM(offset, inputMessage);
  offset = writeStringToEEPROM(offset, inputMessagef);
  offset = writeStringToEEPROM(offset, inputMessagecheck);
  offset = writeStringToEEPROM(offset, String(preRunSeconds));
  offset = writeStringToEEPROM(offset, manualPadlo ? "true" : "false");
  offset = writeStringToEEPROM(offset, manualFal ? "true" : "false");
}

// Read scratchpad with CRC for given address (DeviceAddress is uint8_t)
bool readScratchpadAndCheckCRC(DeviceAddress addr, uint8_t *outScratch) {
  oneWire.reset();
  oneWire.select(addr);
  oneWire.write(0xBE); // READ SCRATCHPAD
  for (int i = 0; i < 9; i++) {
    outScratch[i] = oneWire.read();
  }
  uint8_t c = OneWire::crc8(outScratch, 8);
  if (c != outScratch[8]) { // JAVÍTVA: outScratch[8] a CRC bájt
    Serial.printf("Scratchpad CRC fail (got 0x%02X expected 0x%02X)\n", outScratch[8], c);
    return false;
  }
  return true;
}

String zoneStateToText(ZoneState s) {
  switch(s) {
    case Z_IDLE: return "Idle";
    case Z_PRE_RUN: return "Pre-run";
    case Z_RUNNING: return "Running";
    default: return "Unknown";
  }
}

// find two sensors and check ROM CRC
void initTwoSensors() {
  sensors.begin();
  int count = sensors.getDeviceCount();
  Serial.printf("OneWire devices found: %d\n", count);

  // try get addresses for index 0 and 1
  if (count >= 1) {
    if (sensors.getAddress(addrPadlo, 0)) {
      uint8_t crc = OneWire::crc8(addrPadlo, 7);
      if (crc == addrPadlo[7]) { // JAVÍTVA: addrPadlo[7] a ROM CRC bájt
        sensorPadloFound = true;
        Serial.print("Padló sensor addr: ");
        for (int i=0;i<8;i++) Serial.printf("%02X", addrPadlo[i]);
        Serial.println();
        sensors.setResolution(addrPadlo, 12);
      } else {
        Serial.println("Padló sensor ROM CRC fail");
      }
    }
  }
  if (count >= 2) {
    if (sensors.getAddress(addrFal, 1)) {
      uint8_t crc = OneWire::crc8(addrFal, 7);
      if (crc == addrFal[7]) { // JAVÍTVA: addrFal[7] a ROM CRC bájt
        sensorFalFound = true;
        Serial.print("Fal sensor addr: ");
        for (int i=0;i<8;i++) Serial.printf("%02X", addrFal[i]);
        Serial.println();
        sensors.setResolution(addrFal, 12);
      } else {
        Serial.println("Fal sensor ROM CRC fail");
      }
    }
  }

  if (!sensorPadloFound) Serial.println("Padló sensor not found or CRC bad");
  if (!sensorFalFound) Serial.println("Fal sensor not found or CRC bad");
}

// === Web handlers ===
void handleRoot() {
  String html = String(index_html);
  html.replace("%TEMPERATURE%", lastTemperature);
  html.replace("%THRESHOLD%", inputMessage);
  html.replace("%TEMPERATUREF%", lastTemperaturef);
  html.replace("%THRESHOLDF%", inputMessagef);
  html.replace("%ENABLE_ARM_INPUT%", inputMessage2_enabled ? "checked" : "");
  html.replace("%ENABLE_ARM_INPUTF%", inputMessage4_enabled ? "checked" : "");
  html.replace("%CHECK%", inputMessagecheck);
  html.replace("%PRERUN%", String(preRunSeconds));
  html.replace("%PADLO_ALLAPOT%", String(digitalRead(outputp) == HIGH ? "Bekapcsolva" : "Kikapcsolva"));
  html.replace("%FAL_ALLAPOT%", String(digitalRead(outputf) == HIGH ? "Bekapcsolva" : "Kikapcsolva"));
  html.replace("%KAZAN_ALLAPOT%", String(digitalRead(outputk) == HIGH ? "Bekapcsolva" : "Kikapcsolva"));
  html.replace("%WIFIJEL%", String(WiFi.RSSI()));
  html.replace("%SSID%", String(WiFi.SSID())); // HOZZÁADVA: SSID lekérése
  html.replace("%MANUAL_PADLO%", manualPadlo ? "checked" : "");
  html.replace("%MANUAL_FAL%", manualFal ? "checked" : "");
  server.send(200, "text/html", html);
}

void handleTemperature() {
  // Provide padlo/fal temps plus remaining seconds
  String padloVal = lastTemperature;
  String falVal = lastTemperaturef;
  // remainingPadlo/remainingFal kept updated in loop
  String json = "{\"padlo\":\"" + padloVal + "\",\"fal\":\"" + falVal +
                "\",\"remainingPadlo\":\"" + String(remainingPadlo) +
                "\",\"remainingFal\":\"" + String(remainingFal) + "\"}";
  server.send(200, "application/json", json);
}

void handleAllapot() {
  String json = "{\"padloState\":\"" + zoneStateToText(statePadlo) +
                "\",\"falState\":\"" + zoneStateToText(stateFal) +
                "\",\"kazan\":\"" + String(digitalRead(outputk) == HIGH ? "Bekapcsolva" : "Kikapcsolva") +
                "\",\"wifijel\":\"" + String(WiFi.RSSI()) +
                "\",\"manualPadlo\":\"" + (manualPadlo ? "true":"false") +
                "\",\"manualFal\":\"" + (manualFal ? "true":"false") + "\"}";
  server.send(200, "application/json", json);
}

void handleGet() {
  // thresholds / enables / preRun etc.
  if (server.hasArg("threshold_input")) {
    inputMessage = server.arg("threshold_input");
  }
  inputMessage2_enabled = server.hasArg("enable_arm_input");
  if (server.hasArg("threshold_inputf")) {
    inputMessagef = server.arg("threshold_inputf");
  }
  inputMessage4_enabled = server.hasArg("enable_arm_inputf");
  if (server.hasArg("check")) {
    inputMessagecheck = server.arg("check");
    intervalp = inputMessagecheck.toInt() * 1000UL;
    intervalf = intervalp;
  }
  if (server.hasArg("prerun")) {
    preRunSeconds = (unsigned int) server.arg("prerun").toInt();
  }

  // manual pumps (checkbox presence => true)
  manualPadlo = server.hasArg("manual_padlo");
  manualFal = server.hasArg("manual_fal");
  manualPadloStr = manualPadlo ? "true" : "false";
  manualFalStr  = manualFal  ? "true" : "false";

  // mark for EEPROM save
  // we will save immediately (non-blocking short op)
  saveAllToEEPROM();

  server.send(200, "text/html", "<meta charset='UTF-8'><p>Beállítások mentve.</p><p><a href='/'>Vissza</a></p>");
}

// === setup / loop ===
void setup() {
  Serial.begin(115200);
  delay(50);

  initEEPROMAndLoad();

  WiFiManager wm;
  wm.setHostname("heatctrl");
  if(!wm.autoConnect("HeatCtrlAP")) {
    Serial.println("WiFiManager failed, restarting.");
    delay(3000);
    ESP.restart();
  }
  Serial.print("Connected. IP: "); Serial.println(WiFi.localIP());

  initTwoSensors();

  pinMode(outputp, OUTPUT);
  pinMode(outputf, OUTPUT);
  pinMode(outputk, OUTPUT);
  digitalWrite(outputp, LOW);
  digitalWrite(outputf, LOW);
  digitalWrite(outputk, LOW);

  // web handlers
  server.on("/", HTTP_GET, handleRoot);
  server.on("/temperature", HTTP_GET, handleTemperature);
  server.on("/allapot", HTTP_GET, handleAllapot);
  server.on("/get", HTTP_GET, handleGet);

  server.begin();

  if (mdns.begin("gp")) {
    mdns.addService("http", "tcp", 80);
  }

  previousMillis = millis();
  previousMillisp = millis();
  previousMillisf = millis();
}

// Helper to get temperature by DeviceAddress with scratchpad CRC validation
float getTempForAddr(DeviceAddress a, bool &ok) {
  ok = false;
  if (!a[0]) return INVALID_TEMP;
  // request conversion only once in main loop and then read by index — to be simple we read scratchpad here:
  uint8_t scratch[9];
  if (!readScratchpadAndCheckCRC(a, scratch)) {
    return INVALID_TEMP;
  }
  // raw temp (LSB,MSB)
  int16_t raw = (scratch[1] << 8) | scratch[0];
  // DS18S vs DS18B handling: Dallas lib does complicated handling; for simplicity assume DS18B/DS1822 12-bit
  float t = (float)raw / 16.0;
  ok = true;
  return t;
}

void loop() {
  server.handleClient();

  unsigned long now = millis();

  // Non-blocking periodic sampling trigger (we request conversion via Dallas lib)
  // We'll request conversion every intervalp for padló and every intervalf for fal.
  // Use shared conversion: request once then read scratchpads after small delay (Dallas lib handles).
  static unsigned long lastConvRequest = 0;
  if (now - lastConvRequest >= 1000) {
    // request temperatures via Dallas lib (it issues conversion on the bus)
    sensors.requestTemperatures();
    lastConvRequest = now;
  }

  // Update remaining countdowns (based on last trigger times)
  unsigned long nowMs = millis();
  if (previousMillisp + intervalp > nowMs)
    remainingPadlo = (previousMillisp + intervalp - nowMs) / 1000;
  else remainingPadlo = 0;
  if (previousMillisf + intervalf > nowMs)
    remainingFal = (previousMillisf + intervalf - nowMs) / 1000;
  else remainingFal = 0;

  // read temps from library (getTempCByIndex uses addresses internally)
  float tPadlo_raw = INVALID_TEMP;
  float tFal_raw = INVALID_TEMP;
  bool okPadlo = false, okFal = false;

  if (sensorPadloFound) {
    // use library getTempCByIndex(0) only if we trust device ordering; instead use getTemp for address:
    tPadlo_raw = sensors.getTempC(addrPadlo);
    // Validate typical invalid codes:
    if (tPadlo_raw == DEVICE_DISCONNECTED_C) okPadlo = false;
    else okPadlo = true;
  }
  if (sensorFalFound) {
    tFal_raw = sensors.getTempC(addrFal);
    if (tFal_raw == DEVICE_DISCONNECTED_C) okFal = false;
    else okFal = true;
  }

  // Format to strings; if sensor not ok show "--"
  lastTemperature = okPadlo ? String(tPadlo_raw, 2) : String("--");
  lastTemperaturef = okFal ? String(tFal_raw, 2) : String("--");

  // Update trigger timers: if check interval elapsed -> allow next automatic decision
  if (now - previousMillisp >= intervalp) {
    previousMillisp = now;
    triggertimer = true;
  }
  if (now - previousMillisf >= intervalf) {
    previousMillisf = now;
    triggertimerf = true;
  }

  // ---- PADLÓ state machine & logic ----
  float temperature = okPadlo ? tPadlo_raw : DEVICE_DISCONNECTED_C;

  // Transition logic (non-blocking)
  switch (statePadlo) {
    case Z_IDLE:
      // Automatic start condition: if enabled and temp below threshold and timer allows
      if (inputMessage2_enabled && triggertimer && (temperature != DEVICE_DISCONNECTED_C) && (temperature < inputMessage.toFloat())) {
        // Enter PRE_RUN: ensure pump runs for preRunSeconds before trusting measurement
        statePadlo = Z_PRE_RUN;
        stateStartPadlo = now;
        digitalWrite(outputp, HIGH); // start pump
        Serial.println("Padló -> PRE_RUN (pumpa indítva)");
      }
      break;
    case Z_PRE_RUN:
      // If manual forced off? manualPadlo == false and user turned off while pre-run active => still allow pre-run to complete? We'll allow manual force ON only; manual OFF does not interrupt pre-run.
      // When preRunSeconds elapsed -> go to RUNNING
      if (now - stateStartPadlo >= (unsigned long)preRunSeconds * 1000UL) {
        statePadlo = Z_RUNNING;
        Serial.println("Padló -> RUNNING (mérés érvényes)");
        // reset triggertimer so we don't retrigger too fast
        triggertimer = false;
      }
      break;
    case Z_RUNNING:
      // Running -> decide to stop or keep running
      if (!inputMessage2_enabled) {
        // user disabled auto heating -> stop pump if not manual
        if (!manualPadlo) digitalWrite(outputp, LOW);
        statePadlo = Z_IDLE;
        previousMillisp = now; // Frissítjük az időzítőt IDLE állapotba lépéskor
        Serial.println("Padló -> IDLE (auto disabled)");
      } else {
        // use temperature to decide turning off
        if (temperature != DEVICE_DISCONNECTED_C && (temperature > inputMessage.toFloat())) {
          // reached above threshold -> stop pump (unless manual forcing)
          if (!manualPadlo) digitalWrite(outputp, LOW);
          statePadlo = Z_IDLE;
          previousMillisp = now; // Frissítjük az időzítőt IDLE állapotba lépéskor
          Serial.println("Padló -> IDLE (temp ok)");
        } else {
          // keep running (pump already ON)
        }
      }
      break;
  }

  // Manual override: if manualPadlo==true ensure pump is ON always (force)
  if (manualPadlo) {
    digitalWrite(outputp, HIGH);
  } else {
    // if manual turned off, we must respect automatic logic
    // Edge-case handling: if manual was ON and now OFF, ensure pump is off if auto also doesn't want it
    if (lastManualPadlo && !manualPadlo) {
      // manual went from ON->OFF: if auto state is IDLE, ensure pump off; if RUNNING keep as auto requires
      if (statePadlo == Z_IDLE) {
        digitalWrite(outputp, LOW);
      }
      // else keep pump as auto decided
    }
  }

  // ---- FAL state machine & logic (mirror of padló) ----
  float temperaturef = okFal ? tFal_raw : DEVICE_DISCONNECTED_C;

  switch (stateFal) {
    case Z_IDLE:
      if (inputMessage4_enabled && triggertimerf && (temperaturef != DEVICE_DISCONNECTED_C) && (temperaturef < inputMessagef.toFloat())) {
        stateFal = Z_PRE_RUN;
        stateStartFal = now;
        digitalWrite(outputf, HIGH);
        Serial.println("Fal -> PRE_RUN (pumpa indítva)");
      }
      break;
    case Z_PRE_RUN:
      if (now - stateStartFal >= (unsigned long)preRunSeconds * 1000UL) {
        stateFal = Z_RUNNING;
        Serial.println("Fal -> RUNNING (mérés érvényes)");
        triggertimerf = false;
      }
      break;
    case Z_RUNNING:
      if (!inputMessage4_enabled) {
        if (!manualFal) digitalWrite(outputf, LOW);
        stateFal = Z_IDLE;
        previousMillisf = now; // Frissítjük az időzítőt IDLE állapotba lépéskor
        Serial.println("Fal -> IDLE (auto disabled)");
      } else {
        if (temperaturef != DEVICE_DISCONNECTED_C && (temperaturef > inputMessagef.toFloat())) {
          if (!manualFal) digitalWrite(outputf, LOW);
          stateFal = Z_IDLE;
          previousMillisf = now; // Frissítjük az időzítőt IDLE állapotba lépéskor
          Serial.println("Fal -> IDLE (temp ok)");
        }
      }
      break;
  }

  if (manualFal) digitalWrite(outputf, HIGH);
  else {
    if (lastManualFal && !manualFal) {
      if (stateFal == Z_IDLE) digitalWrite(outputf, LOW);
    }
  }

  // ---- kazán control: start kazán if either zone requests heating and their preconditions satisfied ----
  // We define kazán should be turned ON if any zone is RUNNING and auto logic requests kazán
  bool kazancall = false;
  // For padló: if state is RUNNING and temperature below threshold -> request kazán
  if (statePadlo == Z_RUNNING && temperature != DEVICE_DISCONNECTED_C && (temperature < inputMessage.toFloat())) kazancall = true;
  if (stateFal == Z_RUNNING && temperaturef != DEVICE_DISCONNECTED_C && (temperaturef < inputMessagef.toFloat())) kazancall = true;

  // if any zone requests => start kazán; else stop
  if (kazancall) {
    digitalWrite(outputk, HIGH);
  } else {
    digitalWrite(outputk, LOW);
  }

  // Update remaining display values using state timers
  if (statePadlo == Z_PRE_RUN) {
    long elapsed = (now - stateStartPadlo) / 1000;
    remainingPadlo = preRunSeconds > elapsed ? (preRunSeconds - elapsed) : 0;
  } else if (statePadlo == Z_RUNNING) { // RUNNING módban 0 a visszaszámlálás
    remainingPadlo = 0;
  }
  // Ha Z_IDLE, a korábban kiszámolt idő a következő check-ig (az intervalp timer alapján)


  if (stateFal == Z_PRE_RUN) {
    long elapsed = (now - stateStartFal) / 1000;
    remainingFal = preRunSeconds > elapsed ? (preRunSeconds - elapsed) : 0;
  } else if (stateFal == Z_RUNNING) { // RUNNING módban 0 a visszaszámlálás
     remainingFal = 0;
  }
  // Ha Z_IDLE, a korábban kiszámolt idő a következő check-ig


  // Save last manual for edge detection
  lastManualPadlo = manualPadlo;
  lastManualFal = manualFal;

  // short yield
  delay(0);
}