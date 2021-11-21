#include <functions.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>


TaskHandle_t core2;

IPAddress ip(192, 168, 1, 170);
IPAddress gateway(192,168,1,1);
IPAddress subnet(255,255,255,0);
String ssid = WIFI_SSID;
String password = WIFI_PASSWORD;

IPAddress AP_ip(192,168,8,1);
IPAddress AP_gateway(192,168,8,1);
IPAddress AP_subnet(255,255,255,0);
const char* AP_password = "123gps456";
const uint8_t AP_channel = 13;

AsyncWebServer server(80);

String activeTab = NAME_TAB_BUTTON_TELEMETRY;


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

  // start second core
  xTaskCreatePinnedToCore(
    runOnCore2,
    "Core2",
    STACK_SIZE_CORE2,
    NULL,
    1,
    &core2,
    0);

  int32_t strongestChannel;
  uint8_t* strongestBssid;

  strongestBssid = getChannelWithStrongestSignal(ssid, &strongestChannel);
  if (strongestBssid == NULL) {
    // standalone accesspoint
    WiFi.onEvent(WiFiAPStarted, WiFiEvent_t::SYSTEM_EVENT_AP_START);

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

  voltage = readVoltage();
  Serial.print("Voltage [V]: ");
  Serial.println(getVoltageStr());
  Serial.println();

  Serial.println("WebServer startup");

  server.on("/", WebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request) {   
    //Serial.println("/");
    request->send(200, "text/html", getWebPage(activeTab));
  }); 

  server.on("/Save", WebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request) {
    Serial.println("/Save");
    yawPID.set(request->getParam("Yaw1")->value(), request->getParam("Yaw2")->value(), request->getParam("Yaw3")->value(), request->getParam("Yaw4")->value());
    yawPID.print();    
    yawPID.save();

    steerExpoFactor = checkExpo(request->getParam(getIdFromName(NAME_STEER_EXPO))->value().toDouble());
    steerServoCenterOffset = checkCenterOffset(request->getParam(getIdFromName(NAME_STEER_SERVO_CENTER_OFFSET))->value().toInt());
    speedEscCenterOffset = checkCenterOffset(request->getParam(getIdFromName(NAME_SPEED_ESC_CENTER_OFFSET))->value().toInt());
    voltageCorrectionFactor = request->getParam(getIdFromName(NAME_VOLTAGE_CORRECTION))->value().toDouble();
    currentCorrectionFactor = request->getParam(getIdFromName(NAME_CURRENT_CORRECTION))->value().toDouble();
    calibrated_angle_roll_acc = request->getParam(getIdFromName(NAME_CALIBRATED_ROLL_ANGLE))->value().toDouble();
    calibrated_angle_pitch_acc = request->getParam(getIdFromName(NAME_CALIBRATED_PITCH_ANGLE))->value().toDouble();
    printProps();
    saveProps();

    yawOutputPID.reset();

    activeTab = NAME_TAB_BUTTON_SETTINGS;
    request->redirect("/");
  });  

  server.on("/Cancel", WebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request) {
    Serial.println("/Cancel");
    activeTab = NAME_TAB_BUTTON_SETTINGS;
    request->redirect("/");
  });  

  server.on("/CalibrateAcc", WebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request) {
    Serial.println("/CalibrateAcc");
    calibrateAcc();
    saveProps();
    activeTab = NAME_TAB_BUTTON_SETTINGS;
    request->redirect("/");
  });  

  server.on("/WifiOff", WebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request) {
    Serial.println("/WifiOff");
    WiFi.mode(WIFI_OFF);
  });  

  server.on("/Defaults", WebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request) {
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
    request->redirect("/");
  });    

  server.on("/Zero", WebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request) {
    Serial.println("/Zero");

    maxSpeed = 0.0;
    totalDistance = 0.0;

    activeTab = NAME_TAB_BUTTON_GPS;
    request->redirect("/");
  });  

  server.on("/RequestLatestData", WebRequestMethod::HTTP_GET, [](AsyncWebServerRequest *request) {
  //  Serial.println("/RequestLatestData");
    request->send(200, "application/json", getLatestData());
  });  

  server.begin();
  hs.begin(9600);
}


void loop() {
  voltage = LowPassFilter(VOLTAGE_NOICE_FILTER, readVoltage(), voltage);
  current = LowPassFilter(CURRENT_NOICE_FILTER, readCurrent(), current);  
  getGPSData();
  vTaskDelay(1);
}
