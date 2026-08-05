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

typedef struct{
  double gain;
  short left_ref;
  short right_ref;
  short left_side_th;
  short right_side_th;
  short left_front_th;
  short right_front_th;
  double tire;
  double tread;
  char goal_x;
  char goal_y;
  double search_accel;
  double turn_accel;
  short search_speed;
  short max_speed;
}t_flash_value;

t_flash_value g_flash_value;


void flashInit(void)
{
  String cmd_tmp;
  String file_tmp;
  file_tmp = "/parameters.txt";

  Serial.printf("\n\r parameter init ");
  Serial.println(file_tmp);

  controlInterruptStop();
  sensorInterruptStop();
  PWMInterruptStop();
  delay(100);

  cmd_tmp = "ref_sen_r " + String(REF_SEN_R_INIT) + '\n';
  writeFile(SPIFFS, file_tmp, cmd_tmp);
  cmd_tmp = "ref_sen_l " + String(REF_SEN_L_INIT) + '\n';
  appendFile(SPIFFS, file_tmp, cmd_tmp);
  cmd_tmp = "th_sen_r " + String(TH_SEN_R_INIT) + '\n';
  appendFile(SPIFFS, file_tmp, cmd_tmp);
  cmd_tmp = "th_sen_l " + String(TH_SEN_L_INIT) + '\n';
  appendFile(SPIFFS, file_tmp, cmd_tmp);
  cmd_tmp = "th_sen_fr " + String(TH_SEN_FR_INIT) + '\n';
  appendFile(SPIFFS, file_tmp, cmd_tmp);
  cmd_tmp = "th_sen_fl " + String(TH_SEN_FL_INIT) + '\n';
  appendFile(SPIFFS, file_tmp, cmd_tmp);
  cmd_tmp = "goal_x " + String(GOAL_X_INIT) + '\n';
  appendFile(SPIFFS, file_tmp, cmd_tmp);
  cmd_tmp = "goal_y " + String(GOAL_Y_INIT) + '\n';
  appendFile(SPIFFS, file_tmp, cmd_tmp);
  cmd_tmp = "tire_d " + String(TIRE_DIAMETER_INIT) + '\n';
  appendFile(SPIFFS, file_tmp, cmd_tmp);
  cmd_tmp = "tread_w " + String(TREAD_WIDTH_INIT) + '\n';
  appendFile(SPIFFS, file_tmp, cmd_tmp);
  cmd_tmp = "con_kp " + String(CON_WALL_KP_INIT) + '\n';
  appendFile(SPIFFS, file_tmp, cmd_tmp);

  cmd_tmp = "search_accel " + String(SEARCH_ACCEL_INIT) + '\n';
  appendFile(SPIFFS, file_tmp, cmd_tmp);
  cmd_tmp = "turn_accel " + String(TURN_ACCEL_INIT) + '\n';
  appendFile(SPIFFS, file_tmp, cmd_tmp);
  cmd_tmp = "search_speed " + String(SEARCH_SPEED_INIT) + '\n';
  appendFile(SPIFFS, file_tmp, cmd_tmp);
  cmd_tmp = "max_speed " + String(MAX_SPEED_INIT) + '\n';
  appendFile(SPIFFS, file_tmp, cmd_tmp);

  parameter_select = 1;
  cmd_tmp = "parameter_init 1\n";
  writeFile(SPIFFS, "/parameters_init.txt", cmd_tmp);

  PWMInterruptStart();
  controlInterruptStart();
  sensorInterruptStart();
}

void flashBegin(void)
{
  controlInterruptStop();
  sensorInterruptStop();
  PWMInterruptStop();
  delay(100);

  Serial.println("begin SPIFFS");
  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS Mount Failed");
    delay(100);
  }
  Serial.println("end SPIFFS");

  if (switchGet() == SW_LM) {
    delay(10);
    flashInit();
  }

  PWMInterruptStart();
  controlInterruptStart();
  sensorInterruptStart();
}

void mapWrite(void)
{
  String file_tmp;
  unsigned char data_temp;
  file_tmp = "/map.txt";

  controlInterruptStop();
  sensorInterruptStop();
  PWMInterruptStop();

  Serial.printf("\n\r map_data write ");
  Serial.println(file_tmp);

  File file = SPIFFS.open(file_tmp, FILE_WRITE);
  if (!file) {
    Serial.println("- failed to open file for writing");
    return;
  }
  for (int i = 0; i < MAZESIZE_X; i++) {
    for (int j = 0; j < MAZESIZE_Y; j++) {
      data_temp = g_map.wallDataGet(i, j, north) + (g_map.wallDataGet(i, j, east) << 2) +
                  (g_map.wallDataGet(i, j, south) << 4) + (g_map.wallDataGet(i, j, west) << 6);
      if (file.write(data_temp)) {  //バイナリ書き込み
      } else {
        Serial.println("- write failed");
      }
    }
  }

  file.close();
  controlInterruptStart();
  sensorInterruptStart();
  PWMInterruptStart();
}

void mapCopy(void)
{
  String file_tmp;
  unsigned char read_data;

  controlInterruptStop();
  sensorInterruptStop();
  PWMInterruptStop();

  File file = SPIFFS.open("/map.txt", FILE_READ);
  if (!file || file.isDirectory()) {
    Serial.println("- failed to open file for reading");
    return;
  }
  for (int i = 0; i < MAZESIZE_X; i++) {
    for (int j = 0; j < MAZESIZE_Y; j++) {
      if (file.available()) {
        read_data = file.read();
        g_map.wallDataSet(i, j, north, read_data & 0x03);
        g_map.wallDataSet(i, j, east, (read_data >> 2) & 0x03);
        g_map.wallDataSet(i, j, south, (read_data >> 4) & 0x03);
        g_map.wallDataSet(i, j, west, (read_data >> 6) & 0x03);
      } else {
        Serial.println("Read Error");
      }
    }
  }
  file.close();
  controlInterruptStart();
  sensorInterruptStart();
  PWMInterruptStart();
}

void paramWrite(void)
{
  String cmd_tmp;
  String file_tmp;
  char * temp_char;

  //タイマーによる割り込みは禁止にする必要がある。
  controlInterruptStop();
  sensorInterruptStop();
  PWMInterruptStop();
  delay(10);

  int temp;
  file_tmp = "/parameters.txt";

  Serial.printf("\n\r parameter write ");
  Serial.println(file_tmp);
  cmd_tmp = "ref_sen_r " + String(g_sensor.sen_r.ref) + '\n';
  writeFile(SPIFFS, file_tmp, cmd_tmp);
  cmd_tmp = "ref_sen_l " + String(g_sensor.sen_l.ref) + '\n';
  appendFile(SPIFFS, file_tmp, cmd_tmp);
  cmd_tmp = "th_sen_r " + String(g_sensor.sen_r.th_wall) + '\n';
  appendFile(SPIFFS, file_tmp, cmd_tmp);
  cmd_tmp = "th_sen_l " + String(g_sensor.sen_l.th_wall) + '\n';
  appendFile(SPIFFS, file_tmp, cmd_tmp);
  cmd_tmp = "th_sen_fr " + String(g_sensor.sen_fr.th_wall) + '\n';
  appendFile(SPIFFS, file_tmp, cmd_tmp);
  cmd_tmp = "th_sen_fl " + String(g_sensor.sen_fl.th_wall) + '\n';
  appendFile(SPIFFS, file_tmp, cmd_tmp);
  cmd_tmp = "goal_x " + String((short)g_map.goal_mx) + '\n';
  appendFile(SPIFFS, file_tmp, cmd_tmp);
  cmd_tmp = "goal_y " + String((short)g_map.goal_my) + '\n';
  appendFile(SPIFFS, file_tmp, cmd_tmp);
  cmd_tmp = "tire_d " + String(g_run.tire_diameter) + '\n';
  appendFile(SPIFFS, file_tmp, cmd_tmp);
  cmd_tmp = "tread_w " + String(g_run.tread_width) + '\n';
  appendFile(SPIFFS, file_tmp, cmd_tmp);
  cmd_tmp = "con_kp " + String(g_run.con_wall.kp) + '\n';
  appendFile(SPIFFS, file_tmp, cmd_tmp);

  cmd_tmp = "search_accel " + String(g_run.search_accel) + '\n';
  appendFile(SPIFFS, file_tmp, cmd_tmp);
  cmd_tmp = "turn_accel " + String(g_run.turn_accel) + '\n';
  appendFile(SPIFFS, file_tmp, cmd_tmp);
  cmd_tmp = "search_speed " + String(g_run.search_speed) + '\n';
  appendFile(SPIFFS, file_tmp, cmd_tmp);
  cmd_tmp = "max_speed " + String(g_run.max_speed) + '\n';
  appendFile(SPIFFS, file_tmp, cmd_tmp);

  cmd_tmp = "parameter_init " + String(parameter_select) + '\n';
  writeFile(SPIFFS, "/parameters_init.txt", cmd_tmp);
  Serial.printf("parameter_select %d\n\r", parameter_select);

  controlInterruptStart();
  sensorInterruptStart();
  PWMInterruptStart();
}

void paramRead(void)
{
  String cmd_tmp;
  String file_tmp;
  int temp;

  String cmds[2] = {"\0"};  // 分割された文字列を格納する配列

  //タイマーによる割り込みは禁止にする必要がある。
  controlInterruptStop();
  sensorInterruptStop();
  PWMInterruptStop();
  delay(10);

  file_tmp = "/parameters_init.txt";
  File file = SPIFFS.open(file_tmp);

  if (!file || file.isDirectory()) {
    Serial.println("- parameter_init failed to open file for reading");
    parameter_select = 1;
    cmd_tmp = "parameter_init 1\n";
    writeFile(SPIFFS, "/parameters_init.txt", cmd_tmp);
  } else {
    while (file.available()) {
      int index = split(file.readStringUntil('\n'), ' ', cmds);
      if (cmds[0].equals("parameter_init")) {
        Serial.println("hit parameter_init");
        parameter_select = cmds[1].toInt();
      }
      cmds[0] = { "\0" };
      cmds[1] = { "\0" };
    }
  }
  Serial.printf("parameter_select %d\n\r", parameter_select);
  file.close();  

  file = SPIFFS.open("/parameters.txt");
  if (!file || file.isDirectory()) {
    Serial.println("- failed to open file for reading");
    flashInit();
  }

  Serial.println("- read from file:");
  while (file.available()) {
    int index = split(file.readStringUntil('\n'), ' ', cmds);
    if (cmds[0].equals("ref_sen_r")) {
      g_flash_value.right_ref = cmds[1].toInt();
    } else if (cmds[0].equals("ref_sen_l")) {
      g_flash_value.left_ref = cmds[1].toInt();
    } else if (cmds[0].equals("th_sen_r")) {
      g_flash_value.right_side_th = cmds[1].toInt();
    } else if (cmds[0].equals("th_sen_l")) {
      g_flash_value.left_side_th = cmds[1].toInt();
    } else if (cmds[0].equals("th_sen_fr")) {
      g_flash_value.right_front_th = cmds[1].toInt();
    } else if (cmds[0].equals("th_sen_fl")) {
      g_flash_value.left_front_th = cmds[1].toInt();
    } else if (cmds[0].equals("goal_x")) {
      g_flash_value.goal_x = cmds[1].toInt();
    } else if (cmds[0].equals("goal_y")) {
      g_flash_value.goal_y = cmds[1].toInt();
    } else if (cmds[0].equals("tire_d")) {
      g_flash_value.tire = cmds[1].toFloat();
    } else if (cmds[0].equals("tread_w")) {
      g_flash_value.tread = cmds[1].toFloat();
    } else if (cmds[0].equals("con_kp")) {
      g_flash_value.gain = cmds[1].toFloat();
    } else if (cmds[0].equals("search_accel")) {
      g_flash_value.search_accel = cmds[1].toFloat();
    } else if (cmds[0].equals("turn_accel")) {
      g_flash_value.turn_accel = cmds[1].toFloat();
    } else if (cmds[0].equals("search_speed")) {
      g_flash_value.search_speed = cmds[1].toInt();
    } else if (cmds[0].equals("max_speed")) {
      g_flash_value.max_speed = cmds[1].toInt();
    } else {
      Serial.print("parameter cmds Error ");
      Serial.println(cmds[0]);
      delay(10);
      flashInit();
      break;
    }
    cmds[0] = { "\0" };
    cmds[1] = { "\0" };
  }
  file.close();

  if (parameter_select == 1) {
    g_sensor.sen_r.ref = REF_SEN_R_INIT;
    g_sensor.sen_l.ref = REF_SEN_L_INIT;
    g_sensor.sen_r.th_control = g_sensor.sen_r.th_wall = TH_SEN_R_INIT;
    g_sensor.sen_l.th_control = g_sensor.sen_l.th_wall = TH_SEN_L_INIT;
    g_sensor.sen_fr.th_wall = TH_SEN_FR_INIT;
    g_sensor.sen_fl.th_wall = TH_SEN_FL_INIT;
    g_map.goal_mx = GOAL_X_INIT;
    g_map.goal_my = GOAL_Y_INIT;
    g_run.tire_diameter = TIRE_DIAMETER_INIT;
    g_run.tread_width = TREAD_WIDTH_INIT;
    g_run.con_wall.kp = CON_WALL_KP_INIT;
    g_run.search_accel = SEARCH_ACCEL_INIT;
    g_run.turn_accel = TURN_ACCEL_INIT;
    g_run.search_speed = SEARCH_SPEED_INIT;
    g_run.max_speed = MAX_SPEED_INIT;
  } else {
    g_sensor.sen_r.ref = g_flash_value.right_ref;
    g_sensor.sen_l.ref = g_flash_value.left_ref;
    g_sensor.sen_r.th_control = g_sensor.sen_r.th_wall = g_flash_value.right_side_th;
    g_sensor.sen_l.th_control = g_sensor.sen_l.th_wall = g_flash_value.left_side_th;
    g_sensor.sen_fr.th_wall = g_flash_value.right_front_th;
    g_sensor.sen_fl.th_wall = g_flash_value.left_front_th;
    g_map.goal_mx = g_flash_value.goal_x;
    g_map.goal_my = g_flash_value.goal_y;
    g_run.tire_diameter = g_flash_value.tire;
    g_run.tread_width = g_flash_value.tread;
    g_run.con_wall.kp = g_flash_value.gain;
    g_run.search_accel = g_flash_value.search_accel;
    g_run.turn_accel = g_flash_value.turn_accel;
    g_run.search_speed = g_flash_value.search_speed;
    g_run.max_speed = g_flash_value.max_speed;
  }
  g_run.pulse = g_run.tire_diameter * PI / (35.0 / 10.0 * 20.0 * 8.0);

  controlInterruptStart();
  sensorInterruptStart();
  PWMInterruptStart();
}

int split(String data, char delimiter, String * dst)
{
  int index = 0;
  int arraySize = (sizeof(data) / sizeof((data)[0]));
  int datalength = data.length();
  for (int i = 0; i < datalength; i++) {
    char tmp = data.charAt(i);
    if (tmp == delimiter) {
      index++;
      if (index > (arraySize - 1)) return -1;
    } else
      dst[index] += tmp;
  }
  return (index + 1);
}

void readFile(fs::FS & fs, String path)
{
  Serial.printf("Reading file: ");
  Serial.println(path);

  File file = fs.open(path);
  if (!file || file.isDirectory()) {
    Serial.println("- failed to open file for reading");
    return;
  }

  Serial.println("- read from file:");
  while (file.available()) {
    Serial.write(file.read());
  }
  file.close();
}

void writeFile(fs::FS & fs, String path, String message)
{
  Serial.printf("Writing file: ");
  Serial.println(path);

  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("- failed to open file for writing");
    return;
  }
  if (file.print(message)) {
  } else {
    Serial.println("- frite failed");
  }
  file.close();
}

void appendFile(fs::FS & fs, String path, String message)
{
  Serial.printf("Appending to file: ");
  Serial.println(path);

  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("- failed to open file for appending");
    return;
  }
  if (file.print(message)) {
  } else {
    Serial.println("- append failed");
  }
  file.close();
}
