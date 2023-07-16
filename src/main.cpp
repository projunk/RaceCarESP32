#include <functions.h>
#include <WebServer.h>


TaskHandle_t handle_task1, handle_task2, handle_task3, handle_task4;

IPAddress ip(192,168,1,170);
IPAddress gateway(192,168,1,1);
IPAddress subnet(255,255,255,0);
String ssid = WIFI_SSID;
String password = WIFI_PASSWORD;

IPAddress AP_ip(192,168,8,1);
IPAddress AP_gateway(192,168,8,1);
IPAddress AP_subnet(255,255,255,0);
const char* AP_password = "123gps456";
const uint8_t AP_channel = 13;

WebServer server(80);

String activeTab = NAME_TAB_BUTTON_TELEMETRY;

#define REDIRECT_TO_ROOT server.sendHeader("Location", "/", true); server.send(302, "text/plain", "");



void getWebPageHandler() {
  server.send(200, "text/html", getWebPage(activeTab)); 
}


void saveHandler() {
  Serial.println("/Save");
    
  yawPID.set(server.arg("Yaw1"), server.arg("Yaw2"), server.arg("Yaw3"), server.arg("Yaw4"));
  yawPID.print();    
  yawPID.save();

  steerExpoFactor = checkExpo(server.arg(getIdFromName(NAME_STEER_EXPO)).toDouble());
  steerServoCenterOffset = checkCenterOffset(server.arg(getIdFromName(NAME_STEER_SERVO_CENTER_OFFSET)).toInt());
  speedEscCenterOffset = checkCenterOffset(server.arg(getIdFromName(NAME_SPEED_ESC_CENTER_OFFSET)).toInt());
  voltageCorrectionFactor = server.arg(getIdFromName(NAME_VOLTAGE_CORRECTION)).toDouble();
  currentCorrectionFactor = server.arg(getIdFromName(NAME_CURRENT_CORRECTION)).toDouble();
  calibrated_angle_roll_acc = server.arg(getIdFromName(NAME_CALIBRATED_ROLL_ANGLE)).toDouble();
  calibrated_angle_pitch_acc = server.arg(getIdFromName(NAME_CALIBRATED_PITCH_ANGLE)).toDouble();
  printProps();
  saveProps();

  yawOutputPID.reset();

  activeTab = NAME_TAB_BUTTON_SETTINGS;
  
  REDIRECT_TO_ROOT;
}


void cancelHandler() {
  Serial.println("/Cancel");
  activeTab = NAME_TAB_BUTTON_SETTINGS;
  REDIRECT_TO_ROOT;
}  


void calibrateAccHandler() {
  Serial.println("/CalibrateAcc");
  calibrateAcc();
  saveProps();
  activeTab = NAME_TAB_BUTTON_SETTINGS;
  REDIRECT_TO_ROOT;
}  


void wifiOffHandler() {
  Serial.println("/WifiOff");
  WiFi.mode(WIFI_OFF);
}  


void buzzerOnOffHandler() {
  Serial.println("/BuzzerOnOff");
  buzzerOff = ! buzzerOff;
  activeTab = NAME_TAB_BUTTON_SETTINGS;
  REDIRECT_TO_ROOT;
}  


void defaultsHandler() {
  Serial.println("/Defaults");
    
  yawPID.resetToDefault();
  yawPID.print();    
  yawPID.save();

  steerExpoFactor = defaultSteerExpoFactor;
  steerServoCenterOffset = defaultSteerServoCenterOffset;
  speedEscCenterOffset = defaultSpeedEscCenterOffset;
  voltageCorrectionFactor = defaultVoltageCorrectionFactor;
  currentCorrectionFactor = defaultCurrentCorrectionFactor;
  calibrated_angle_roll_acc = defaultCalibratedRollAngleAcc;
  calibrated_angle_pitch_acc = defaultCalibratedPitchAngleAcc;    
  printProps();
  saveProps();

  yawOutputPID.reset();

  activeTab = NAME_TAB_BUTTON_SETTINGS;
  REDIRECT_TO_ROOT;  
};    

void zeroHandler() {
  Serial.println("/Zero");

  maxSpeed = 0.0;
  totalDistance = 0.0;

  activeTab = NAME_TAB_BUTTON_GPS;
  REDIRECT_TO_ROOT;
};  


void getLatestDataHandler() {
  //Serial.println("getLatestDataHandler");
  server.send(200, "text/html", getLatestData()); 
}


void notFoundHandler() {
  server.send(404, "text/plain", "Not found");
}


void setup() {
  Serial.begin(115200);

  robotName = identifyRobot(); 
  Serial.println();
  UniqueIDdump(Serial);
  Serial.println();
  Serial.print("Robot: ");
  Serial.println(robotName);

  Serial.print("Core: ");
  Serial.println(xPortGetCoreID());

  if (SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)) {
    yawPID.load();
    loadProps();
  } else {
    Serial.println("SPIFFS Mount Failed");
  }

  // start tasks second core
  xTaskCreatePinnedToCore(
    task1,
    "task1",
    STACK_SIZE_CORE,
    NULL,
    1,
    &handle_task1,
    CORE1);

  xTaskCreatePinnedToCore(
    task2,
    "task2",
    STACK_SIZE_CORE,
    NULL,
    1,
    &handle_task2,
    CORE1);

  xTaskCreatePinnedToCore(
    task3,
    "task3",
    STACK_SIZE_CORE,
    NULL,
    1,
    &handle_task3,
    CORE0);

  xTaskCreatePinnedToCore(
    task4,
    "task4",
    STACK_SIZE_CORE,
    NULL,
    1,
    &handle_task4,
    CORE1);

  int32_t strongestChannel;
  uint8_t* strongestBssid;

  strongestBssid = getChannelWithStrongestSignal(ssid, &strongestChannel);
  if (strongestBssid == NULL) {
    // standalone accesspoint
    WiFi.onEvent(WiFiAPStarted, WiFiEvent_t::ARDUINO_EVENT_WIFI_AP_START);

    WiFi.mode(WIFI_AP);
    WiFi.softAP(getSSID().c_str(), AP_password, AP_channel); 
    for (;;) {
      if (isAPStarted()) {
        break;
      }
      delay(1);
    }  
    WiFi.softAPConfig(AP_ip, AP_gateway, AP_subnet);
    Serial.println();
    Serial.print("SSID: ");
    Serial.println(getSSID());
    Serial.print("IP address: ");
    Serial.println(WiFi.softAPIP());
  } else {
    // connect to local wifi network
    Serial.print("Connecting WiFi"); 
    WiFi.config(ip, gateway, subnet);

    // disable power safe for performance (low latency)
    esp_wifi_set_ps(WIFI_PS_NONE);

    WiFi.begin(ssid.c_str(), password.c_str(), strongestChannel, strongestBssid, true);
    while (WiFi.status() != WL_CONNECTED) {
      Serial.print(".");
      delay(500);
    }
    Serial.println();
    Serial.print("IP address: ");
    Serial.println(ip);
  }

  Serial.println("WebServer startup");

  server.on("/", HTTPMethod::HTTP_GET, getWebPageHandler);   

  server.on("/Save", HTTPMethod::HTTP_GET, saveHandler);

  server.on("/Cancel", HTTPMethod::HTTP_GET, cancelHandler);

  server.on("/CalibrateAcc", HTTPMethod::HTTP_GET, calibrateAccHandler);

  server.on("/WifiOff", HTTPMethod::HTTP_GET, wifiOffHandler);

  server.on("/BuzzerOnOff", HTTPMethod::HTTP_GET, buzzerOnOffHandler);

  server.on("/Defaults", HTTPMethod::HTTP_GET, defaultsHandler);

  server.on("/Zero", HTTPMethod::HTTP_GET, zeroHandler);

  server.on("/RequestLatestData", HTTPMethod::HTTP_GET, getLatestDataHandler);

  server.onNotFound(notFoundHandler);

  server.begin();
}


void loop() {
  server.handleClient();
}
