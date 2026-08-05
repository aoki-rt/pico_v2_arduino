// Copyright 2023 RT Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#define AP_MODE

AsyncWebServer server(80);

#ifdef AP_MODE
const char * ssid = "PICO2";
const char * password = "12345678";
#else
const char * ssid = "使用しているルータのSSID";
const char * password = "ルータのパスワード";
#endif

void webServerSetup(void)
{
#ifdef AP_MODE
  WiFi.softAP(ssid, password);
  IPAddress myIP = WiFi.softAPIP();
  Serial.println(myIP);
#else
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.printf("WiFi Failed!\n");
    return;
  }
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
#endif

  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
    String html = "";
    html += "<!DOCTYPE html>";
    html += "<html lang=\"en\">";
    html += "<head>";
    html += "<meta charset=\"UTF-8\">";
    html += "<title>Pi:Co V2 WebServer</title>";
    html += "<style>";
    html += "  html {";
    html += "    font-family: Helvetica;";
    html += "    display: inline-block;";
    html += "    margin: 0px auto;";
    html += "    text-align: center;";
    html += "  }";
    html += "  h1 {";
    html += "    font-size: 3.0rem;";
    html += "    color: blue;";
    html += "  }";
    html += "  </style>";
    html += "</head>";
    html += "<body>";
    html += "<h1>Pi:Co V2_Parameter</h1>";
    if(parameter_select==1){
      html += "<input type=\"checkbox\" id =\"parameter_init\" onchange=\"toggleData()\" checked >ParameterのINITの値を使用";
    }else{
      html += "<input type=\"checkbox\" id =\"parameter_init\" onchange=\"toggleData()\" >ParameterのINITの値を使用";
    }

    html += "<form action=\"/get\">";

    html += "<p><h2>battery Value</h2></p>";
    html += "<table align=\"center\">";
    html += "<tr><td id=\"voltage\">";
    html += String(g_sensor.battery_value);
    html += "</td></tr>";
    html += "</table>";
    html += "<br>";

    html += "<p><h2>Sensor Value</h2></p>";
    html += "<table align=\"center\" border=\"1\">";
    html += "<tr><th></th><th>Left</th><th>Right</th></tr>";
    html += "<tr><th>SIDE VALUE</th><td id=\"left_value\">";
    html += String(g_sensor.sen_l.value);
    html += "</td><td id=\"right_value\">";
    html += String(g_sensor.sen_r.value);
    html += "</td></tr>";
    html += "<tr><th>FRONT VALUE</th><td id=\"left_front_value\">";
    html += String(g_sensor.sen_fl.value);
    html += "</td><td id=\"right_front_value\">";
    html += String(g_sensor.sen_fr.value);
    html += "</td></tr>";
    html += "</table>";
    html += "<br>";
    html += "<br>";

    html += "<p><h2>Sensor Parameter</h2></p>";

    html += "<table align=\"center\">";
    html += "<tr><th>Gain</th><td><input name=\"wall_kp\" type=\"text\" id=\"wall_kp\" size=\"10\" value=";
    html += String(g_run.con_wall.kp, 3);
    html += "></td></tr>";
    html += "</table>";
    html += "<br>";

    html += "<table align=\"center\">";
    html += "<tr><th></th><th>Left</th><th>Right</th></tr>";
    html += "<tr><th>SIDE REF</th><th><input name=\"ref_left\" type=\"text\" id=\"ref_left\" size=\"10\" value=";
    html += String(g_sensor.sen_l.ref);
    html += "></th><th><input name=\"ref_right\" type=\"text\" id=\"ref_right\" size=\"10\" value=";
    html += String(g_sensor.sen_r.ref);
    html += "></th></tr>";
    html +=
      "<tr><th>SIDE Threshold</th><td><input name=\"th_left\" type=\"text\" id=\"th_left\" size=\"10\" value=";
    html += String(g_sensor.sen_l.th_wall);
    html += "></td><td><input name=\"th_right\" type=\"text\" id=\"th_right\" size=\"10\" value=";
    html += String(g_sensor.sen_r.th_wall);
    html += "></td></tr>";
    html +=
      "<tr><th>FRONT Threshold</th><th><input name=\"th_fl\" id=\"th_fl\" type=\"text\"size=\"10\"  value=";
    html += String(g_sensor.sen_fl.th_wall);
    html += "></th><th><input name=\"th_fr\" type=\"text\" id=\"th_fr\" size=\"10\" value=";
    html += String(g_sensor.sen_fr.th_wall);
    html += "></th></tr>";
    html += "</table>";

    html += "<br>";
    html += "<br>";

    html += "<input type=\"submit\" value=\"Save\">";

    html += "<br>";
    html += "<br>";

    html += "<p><h2>Tire Parameter</h2></p>";
    html += "<table align=\"center\">";
    html +=
      "<tr><th>TIRE_DIAMETER</th><td><input name=\"tire_dia\" id=\"tire_dia\" type=\"text\" size=\"10\" value=";
    html += String(g_run.tire_diameter, 3);
    html += ">mm</td></tr>";
    html +=
      "<tr><th>TREAD_WIDTH</th><td><input name=\"tread_width\" id=\"tread_width\" type=\"text\" size=\"10\" value=";
    html += String(g_run.tread_width, 3);
    html += ">mm</td></tr>";
    html += "</table>";
    html += "<br>";
    html += "<br>";

    html += "<p><h2>GOAL Parameter</h2></p>";
    html += "<table align=\"center\">";
    html += "<tr><th></th><th>X</th><th>Y</th></tr>";
    html += "<tr><th>AXIS</th><td><input name=\"goal_x\" type=\"text\" id=\"goal_x\" size=\"10\" value=";
    html += String(g_map.goal_mx);
    html += "></td><td><input name=\"goal_y\" type=\"text\" size=\"10\" id=\"goal_y\" value=";
    html += String(g_map.goal_my);
    html += "></td></tr>";
    html += "</table>";
    html += "<br>";
    html += "<br>";

    html += "<p><h2>Accel Parameter</h2></p>";
    html += "<table align=\"center\">";
    html +=
      "<tr><th>Search accel</th><td><input name=\"search_acc\" id=\"search_acc\" type=\"text\" size=\"10\" value=";
    html += String(g_run.search_accel, 3);
    html += ">mm</td></tr>";
    html += "<tr><th>Turn accel</th><td><input name=\"turn_acc\" id=\"turn_acc\" type=\"text\" size=\"10\" value=";
    html += String(g_run.turn_accel, 3);
    html += ">mm</td></tr>";
    html += "</table>";
    html += "<br>";
    html += "<br>";

    html += "<p><h2>Speed Parameter</h2></p>";
    html += "<table align=\"center\">";
    html +=
      "<tr><th>Search speed</th><td><input name=\"search_spd\" id=\"search_spd\" type=\"text\" size=\"10\" value=";
    html += String(g_run.search_speed);
    html += ">mm</td></tr>";
    html += "<tr><th>max speed</th><td><input name=\"max_spd\" id=\"max_spd\" type=\"text\" size=\"10\" value=";
    html += String(g_run.max_speed);
    html += ">mm</td></tr>";
    html += "</table>";
    html += "<br>";
    html += "<br>";

    html += "<input type=\"submit\" value=\"Save\">";
    html += "</form><br>";
    html += "</body>";
    html += "<script>";

    html += "var getVoltage = function () {";
    html += "var xhr = new XMLHttpRequest();";
    html += "xhr.onreadystatechange = function() {";
    html += "if (this.readyState == 4 && this.status == 200) {";
    html += "document.getElementById(\"voltage\").innerHTML = this.responseText;";
    html += "}";
    html += "};";
    html += "xhr.open(\"GET\", \"/voltage\", true);";
    html += "xhr.send(null);";
    html += "};";
    html += "setInterval(getVoltage, 1000);";

    html += "var getLeftValue = function () {";
    html += "var xhr = new XMLHttpRequest();";
    html += "xhr.onreadystatechange = function() {";
    html += "if (this.readyState == 4 && this.status == 200) {";
    html += "document.getElementById(\"left_value\").innerHTML = this.responseText;";
    html += "}";
    html += "};";
    html += "xhr.open(\"GET\", \"/left_value\", true);";
    html += "xhr.send(null);";
    html += "};";
    html += "setInterval(getLeftValue, 1000);";

    html += "var getRightValue = function () {";
    html += "var xhr = new XMLHttpRequest();";
    html += "xhr.onreadystatechange = function() {";
    html += "if (this.readyState == 4 && this.status == 200) {";
    html += "document.getElementById(\"right_value\").innerHTML = this.responseText;";
    html += "}";
    html += "};";
    html += "xhr.open(\"GET\", \"/right_value\", true);";
    html += "xhr.send(null);";
    html += "};";
    html += "setInterval(getRightValue, 1000);";

    html += "var getLeftFrontValue = function () {";
    html += "var xhr = new XMLHttpRequest();";
    html += "xhr.onreadystatechange = function() {";
    html += "if (this.readyState == 4 && this.status == 200) {";
    html += "document.getElementById(\"left_front_value\").innerHTML = this.responseText;";
    html += "}";
    html += "};";
    html += "xhr.open(\"GET\", \"/left_front_value\", true);";
    html += "xhr.send(null);";
    html += "};";
    html += "setInterval(getLeftFrontValue, 1000);";

    html += "var getRightFrontValue = function () {";
    html += "var xhr = new XMLHttpRequest();";
    html += "xhr.onreadystatechange = function() {";
    html += "if (this.readyState == 4 && this.status == 200) {";
    html += "document.getElementById(\"right_front_value\").innerHTML = this.responseText;";
    html += "}";
    html += "};";
    html += "xhr.open(\"GET\", \"/right_front_value\", true);";
    html += "xhr.send(null);";
    html += "};";
    html += "setInterval(getRightFrontValue, 1000);";

//チェックボックスの切り替え
   html += "function toggleData() {";
    html += "  const isChecked = document.getElementById(\"parameter_init\").checked;";
    html += "  const val = isChecked ? 1 : 0;";
    html += "  var xhr = new XMLHttpRequest();";
    html += "  xhr.onreadystatechange = function() {";
    html += "    if (this.readyState == 4 && this.status == 200) {";
    html += "      var data = JSON.parse(this.responseText);";
    html += "      document.getElementById(\"wall_kp\").value = data.wall_kp;";
    html += "      document.getElementById(\"ref_left\").value = data.ref_left;";
    html += "      document.getElementById(\"ref_right\").value = data.ref_right;";
    html += "      document.getElementById(\"th_left\").value = data.th_left;";
    html += "      document.getElementById(\"th_right\").value = data.th_right;";
    html += "      document.getElementById(\"th_fl\").value = data.th_fl;";
    html += "      document.getElementById(\"th_fr\").value = data.th_fr;";
    html += "      document.getElementById(\"tire_dia\").value = data.tire_dia;";
    html += "      document.getElementById(\"tread_width\").value = data.tread_width;";
    html += "      document.getElementById(\"goal_x\").value = data.goal_x;";
    html += "      document.getElementById(\"goal_y\").value = data.goal_y;";
    html += "      document.getElementById(\"search_acc\").value = data.search_acc;";
    html += "      document.getElementById(\"turn_acc\").value = data.turn_acc;";
    html += "      document.getElementById(\"search_spd\").value = data.search_spd;";
    html += "      document.getElementById(\"max_spd\").value = data.max_spd;";
    html += "    }";
    html += "  };";
    html += "  xhr.open(\"GET\", \"/toggle_param?state=\" + val, true);";
    html += "  xhr.send(null);";
    html += "}";

    html += "</script>";
    html += "</html>";
    request->send(200, "text/html", html);
  });

  server.on("/voltage", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(200, "text/plain", String(g_sensor.battery_value) + "mV");
  });

  server.on("/left_value", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(200, "text/plain", String(g_sensor.sen_l.value));
  });

  server.on("/right_value", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(200, "text/plain", String(g_sensor.sen_r.value));
  });

  server.on("/left_front_value", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(200, "text/plain", String(g_sensor.sen_fl.value));
  });

  server.on("/right_front_value", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(200, "text/plain", String(g_sensor.sen_fr.value));
  });

server.on("/toggle_param", HTTP_GET, [](AsyncWebServerRequest *request) {
    String json = "";
    if (request->hasParam("state")) {
        String stateVal = request->getParam("state")->value();
        if (stateVal == "1") {
            parameter_select = 1;
            json = "{";
            json += "\"wall_kp\":" + String(CON_WALL_KP_INIT,3) + ",";
            json += "\"ref_left\":" + String(REF_SEN_L_INIT) + ",";
            json += "\"ref_right\":" + String(REF_SEN_R_INIT) + ",";
            json += "\"th_left\":" + String(TH_SEN_L_INIT) + ",";
            json += "\"th_right\":" + String(TH_SEN_R_INIT) + ",";
            json += "\"th_fl\":" + String(TH_SEN_FL_INIT) + ",";
            json += "\"th_fr\":" + String(TH_SEN_FR_INIT) +",";
            json += "\"tire_dia\":" + String(TIRE_DIAMETER_INIT,3) + ",";
            json += "\"tread_width\":" + String(TREAD_WIDTH_INIT,3) + ",";
            json += "\"goal_x\":" + String(GOAL_X_INIT) + ",";
            json += "\"goal_y\":" + String(GOAL_Y_INIT) + ",";
            json += "\"search_acc\":" + String(SEARCH_ACCEL_INIT,3) + ",";
            json += "\"turn_acc\":" + String(TURN_ACCEL_INIT,3) + ",";
            json += "\"search_spd\":" + String(SEARCH_SPEED_INIT) + ",";
            json += "\"max_spd\":" + String(MAX_SPEED_INIT);
            json += "}";
        } else {
            parameter_select = 0;
            json = "{";
            json += "\"wall_kp\":" + String(g_run.con_wall.kp,3) + ",";
            json += "\"ref_left\":" + String(g_sensor.sen_l.ref ) + ",";
            json += "\"ref_right\":" + String(g_sensor.sen_r.ref ) + ",";
            json += "\"th_left\":" + String(g_sensor.sen_l.th_wall) + ",";
            json += "\"th_right\":" + String(g_sensor.sen_r.th_wall) + ",";
            json += "\"th_fl\":" + String(g_sensor.sen_fl.th_wall) + ",";
            json += "\"th_fr\":" + String(g_sensor.sen_fr.th_wall) +",";
            json += "\"tire_dia\":" + String(g_run.tire_diameter,3) + ",";
            json += "\"tread_width\":" + String(g_run.tread_width,3) + ",";
            json += "\"goal_x\":" + String(g_map.goal_mx) + ",";
            json += "\"goal_y\":" + String(g_map.goal_my) + ",";
            json += "\"search_acc\":" + String(g_run.search_accel,3) + ",";
            json += "\"turn_acc\":" + String(g_run.turn_accel,3) + ",";
            json += "\"search_spd\":" + String(g_run.search_speed) + ",";
            json += "\"max_spd\":" + String(g_run.max_speed);
            json += "}";
        }
    }
    // JSON形式としてブラウザにレスポンスを返す
    request->send(200, "application/json", json);    
  });
  
  server.on("/get", HTTP_GET, [](AsyncWebServerRequest * request) {
    String inputMessage;

    inputMessage = request->getParam("tire_dia")->value();
    g_run.tire_diameter = inputMessage.toFloat();
    inputMessage = request->getParam("tread_width")->value();
    g_run.tread_width = inputMessage.toFloat();

    inputMessage = request->getParam("wall_kp")->value();
    g_run.con_wall.kp = inputMessage.toFloat();

    inputMessage = request->getParam("ref_left")->value();
    g_sensor.sen_l.ref = inputMessage.toInt();
    inputMessage = request->getParam("ref_right")->value();
    g_sensor.sen_r.ref = inputMessage.toInt();

    inputMessage = request->getParam("th_left")->value();
    g_sensor.sen_l.th_wall = inputMessage.toInt();
    inputMessage = request->getParam("th_right")->value();
    g_sensor.sen_r.th_wall = inputMessage.toInt();

    inputMessage = request->getParam("th_fl")->value();
    g_sensor.sen_fl.th_wall = inputMessage.toInt();
    inputMessage = request->getParam("th_fr")->value();
    g_sensor.sen_fr.th_wall = inputMessage.toInt();

    inputMessage = request->getParam("goal_x")->value();
    g_map.goal_mx = inputMessage.toInt();
    inputMessage = request->getParam("goal_y")->value();
    g_map.goal_my = inputMessage.toInt();

    inputMessage = request->getParam("search_acc")->value();
    g_run.search_accel = inputMessage.toFloat();
    inputMessage = request->getParam("turn_acc")->value();
    g_run.turn_accel = inputMessage.toFloat();

    inputMessage = request->getParam("search_spd")->value();
    g_run.search_speed = inputMessage.toInt();
    inputMessage = request->getParam("max_spd")->value();
    g_run.max_speed = inputMessage.toInt();

    Serial.println("saved");
    paramWrite();
    g_run.pulse = g_run.tire_diameter * PI / (35.0 / 10.0 * 20.0 * 8.0);

    buzzerEnable(INC_FREQ);
    delay(30);
    buzzerDisable();
    request->redirect("/");
  });

  // ESP32_WebServer start
  server.begin();  //APモードを起動
  Serial.println("ESP32_WebServer start!");
}