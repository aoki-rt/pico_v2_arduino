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

#include "fast.h"
#include "misc.h"
#include "sensor.h"

MISC g_misc;

short MISC::buttonInc(short data, short limit, short limit_data)
{
  data++;
  if (data > limit) {
    data = limit_data;
  }
  buzzerEnable(INC_FREQ);
  delay(30);
  buzzerDisable();

  return data;
}

void MISC::buttonOk(void)
{
  while (1) {
    ledSet(0x0f);
    delay(500);
    if ((g_sensor.sen_fr.value + g_sensor.sen_fl.value) > 2000) {
      break;
    }
    ledSet(0x00);
    delay(500);
    if ((g_sensor.sen_fr.value + g_sensor.sen_fl.value) > 2000) {
      break;
    }
  }
  ledSet(0x00);

  buzzerEnable(DEC_FREQ);
  delay(80);
  buzzerEnable(INC_FREQ);
  delay(80);
  buzzerDisable();
}

void MISC::goalAppeal(void)  //ゴールしたことをアピールする
{
  unsigned char led_data;

  int wtime = 100;

  motorDisable();
  mapWrite();

  for (int j = 0; j < 10; j++) {
    led_data = 1;
    for (int i = 0; i < 4; i++) {
      ledSet(led_data);
      led_data <<= 1;
      delay(wtime);
    }
    led_data = 8;
    for (int i = 0; i < 4; i++) {
      ledSet(led_data);
      led_data >>= 1;
      delay(wtime);
    }
    wtime -= 5;
  }

  delay(500);
  motorEnable();
}

void MISC::modeExec(int mode)
{
  short max_speed_tmp;
  motorEnable();
  delay(1000);

  switch (mode) {
    case 1:
      g_search.lefthand();
      break;
    case 2:  //足立法-low
      max_speed_tmp = g_run.max_speed;
      g_run.max_speed = g_run.search_speed;
      g_map.positionInit();
      g_search.adachi(g_map.goal_mx, g_map.goal_my, g_run.search_accel_low, g_run.search_speed_low);
      g_run.rotate(right, 2);
      g_map.nextDir(right);
      g_map.nextDir(right);
      g_misc.goalAppeal();
      g_search.adachi(0, 0, g_run.search_accel_low, g_run.search_speed_low);
      g_run.rotate(right, 2);
      g_map.nextDir(right);
      g_map.nextDir(right);
      mapWrite();
      g_run.max_speed = max_speed_tmp;
      break;
    case 3:  //足立法-normal
      max_speed_tmp = g_run.max_speed;
      g_run.max_speed = (g_run.search_speed + g_run.max_speed) / 2;
      g_map.positionInit();
      g_search.adachi(g_map.goal_mx, g_map.goal_my, g_run.search_accel, g_run.search_speed);
      g_run.rotate(right, 2);
      g_map.nextDir(right);
      g_map.nextDir(right);
      g_misc.goalAppeal();
      g_search.adachi(0, 0, g_run.search_accel, g_run.search_speed);
      g_run.rotate(right, 2);
      g_map.nextDir(right);
      g_map.nextDir(right);
      mapWrite();
      g_run.max_speed = max_speed_tmp;
      break;
    case 4:  //最短走行-normal
      mapCopy();
      g_map.positionInit();
      g_fast.run(g_map.goal_mx, g_map.goal_my, g_run.search_accel, g_run.search_speed);
      g_run.rotate(right, 2);
      g_map.nextDir(right);
      g_map.nextDir(right);
      g_misc.goalAppeal();
      g_fast.run(0, 0, g_run.search_accel, g_run.search_speed);
      g_run.rotate(right, 2);
      g_map.nextDir(right);
      g_map.nextDir(right);
      break;
    case 5:  //最短走行-high
      mapCopy();
      g_map.positionInit();
      g_fast.run(g_map.goal_mx, g_map.goal_my, g_run.search_accel_high, g_run.search_speed_high);
      g_run.rotate(right, 2);
      g_map.nextDir(right);
      g_map.nextDir(right);
      g_misc.goalAppeal();
      g_fast.run(0, 0, g_run.search_accel, g_run.search_speed);
      g_run.rotate(right, 2);
      g_map.nextDir(right);
      g_map.nextDir(right);
    case 6:  //足立法 sura-low
      max_speed_tmp = g_run.max_speed;
      g_run.max_speed = g_run.search_speed;
      g_map.positionInit();
      g_search.adachi_sura(
        g_map.goal_mx, g_map.goal_my, g_run.search_accel_low, g_run.search_speed_low - 25);
      g_run.rotate(right, 2);
      g_map.nextDir(right);
      g_map.nextDir(right);
      g_misc.goalAppeal();
      g_search.adachi_sura(0, 0, g_run.search_accel_low, g_run.search_speed_low - 25);
      g_run.rotate(right, 2);
      g_map.nextDir(right);
      g_map.nextDir(right);
      mapWrite();
      g_run.max_speed = max_speed_tmp;
      break;
    case 7:  //足立法 sura-normal
      max_speed_tmp = g_run.max_speed;
      g_run.max_speed = (g_run.search_speed + g_run.max_speed) / 2;
      g_map.positionInit();
      g_search.adachi_sura(
        g_map.goal_mx, g_map.goal_my, g_run.search_accel, g_run.search_speed - 25);
      g_run.rotate(right, 2);
      g_map.nextDir(right);
      g_map.nextDir(right);
      g_misc.goalAppeal();
      g_search.adachi_sura(0, 0, g_run.search_accel, g_run.search_speed - 25);
      g_run.rotate(right, 2);
      g_map.nextDir(right);
      g_map.nextDir(right);
      mapWrite();
      g_run.max_speed = max_speed_tmp;
      break;
    case 8:
      mapCopy();
      g_map.positionInit();
      g_fast.runSura(g_map.goal_mx, g_map.goal_my, g_run.search_accel, g_run.search_speed - 25);
      g_run.rotate(right, 2);
      g_map.nextDir(right);
      g_map.nextDir(right);
      g_misc.goalAppeal();
      g_fast.runSura(0, 0, g_run.search_accel, g_run.search_speed - 25);
      g_run.rotate(right, 2);
      g_map.nextDir(right);
      g_map.nextDir(right);
      break;
    case 9:
      mapCopy();
      g_map.positionInit();
      g_fast.runSura(
        g_map.goal_mx, g_map.goal_my, g_run.search_accel_high, g_run.search_speed_high - 25);
      g_run.rotate(right, 2);
      g_map.nextDir(right);
      g_map.nextDir(right);
      g_misc.goalAppeal();
      g_fast.runSura(0, 0, g_run.search_accel, g_run.search_speed - 25);
      g_run.rotate(right, 2);
      g_map.nextDir(right);
      g_map.nextDir(right);
      break;
    case 10:
      break;
    case 11:
      break;
    case 12:
      break;
    case 13:
      break;
    case 14:
      g_fast.patternMake(g_map.goal_mx, g_map.goal_my);
      for (int i = 0; i < 256; i++) {
        Serial.print(g_fast.second_pattern[i++]);
        Serial.print(" ");
        if (g_fast.second_pattern[i] == 127) {
          Serial.println(g_fast.second_pattern[i]);
          break;
        } else {
          Serial.println(g_fast.second_pattern[i++]);
        }
      }

      break;
    case 15:
      motorDisable();
      g_adjust.menu();  //調整メニューに行く
      break;
    default:
      break;
  }
  motorDisable();
}
