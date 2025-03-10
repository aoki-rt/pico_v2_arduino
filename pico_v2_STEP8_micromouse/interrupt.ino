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

void sensorInterrupt(void)
{
  static char cnt = 0;
  static char bled_cnt = 0;

  switch (cnt) {
    case 0:
      g_sen_r.p_value = g_sen_r.value;
      g_sen_l.p_value = g_sen_l.value;
      getSensorS(&g_sen_r.value, &g_sen_l.value);
      if (g_sen_r.value > g_sen_r.th_wall) {
        g_sen_r.is_wall = true;
      } else {
        g_sen_r.is_wall = false;
      }
      if (g_sen_r.value > g_sen_r.th_control) {
        g_sen_r.error = g_sen_r.value - g_sen_r.ref;
        g_sen_r.is_control = true;
      } else {
        g_sen_r.error = 0;
        g_sen_r.is_control = false;
      }
      if (g_sen_l.value > g_sen_l.th_wall) {
        g_sen_l.is_wall = true;
      } else {
        g_sen_l.is_wall = false;
      }
      if (g_sen_l.value > g_sen_l.th_control) {
        g_sen_l.error = g_sen_l.value - g_sen_l.ref;
        g_sen_l.is_control = true;
      } else {
        g_sen_l.error = 0;
        g_sen_l.is_control = false;
      }
      break;
    case 1:
      g_sen_fr.p_value = g_sen_fr.value;
      g_sen_fl.p_value = g_sen_fl.value;
      getSensorF(&g_sen_fr.value, &g_sen_fl.value);
      if (g_sen_fr.value > g_sen_fr.th_wall) {
        g_sen_fr.is_wall = true;
      } else {
        g_sen_fr.is_wall = false;
      }
      if (g_sen_fl.value > g_sen_fl.th_wall) {
        g_sen_fl.is_wall = true;
      } else {
        g_sen_fl.is_wall = false;
      }

      bled_cnt++;
      if (bled_cnt > 10) {
        bled_cnt = 0;
      }
      g_battery_value = getBatteryVolt();
      if (((g_battery_value - BATT_MIN) * 10 / (BATT_MAX - BATT_MIN)) > bled_cnt) {
        setBLED(1);
      } else {
        setBLED(2);
      }
      break;
    default:
      Serial.printf("sensor state error\n\r");
      break;
  }
  cnt++;
  if (cnt == 2) cnt = 0;
}