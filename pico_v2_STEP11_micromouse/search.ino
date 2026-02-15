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

#include "search.h"

SEARCH g_search;

void SEARCH::lefthand(void)
{
  g_run.accelerate(HALF_SECTION, g_run.search_speed, g_run.search_accel);

  while (1) {
    if (g_sensor.sen_l.is_wall == false) {
      g_run.decelerate(HALF_SECTION, g_run.search_speed, g_run.search_accel);
      g_run.rotate(left, 1);
      g_run.accelerate(HALF_SECTION, g_run.search_speed, g_run.search_accel);
    } else if ((g_sensor.sen_fl.is_wall == false) && (g_sensor.sen_fr.is_wall == false)) {
      g_run.straight(
        SECTION, g_run.search_speed, g_run.search_speed, g_run.search_speed, g_run.search_accel);
    } else if (g_sensor.sen_r.is_wall == false) {
      g_run.decelerate(HALF_SECTION, g_run.search_speed, g_run.search_accel);
      g_run.rotate(right, 1);
      g_run.accelerate(HALF_SECTION, g_run.search_speed, g_run.search_accel);
    } else {
      g_run.decelerate(HALF_SECTION, g_run.search_speed, g_run.search_accel);
      g_run.rotate(right, 2);
      g_run.accelerate(HALF_SECTION, g_run.search_speed, g_run.search_accel);
    }
  }
}

void SEARCH::adachi(char gx, char gy, float l_accel, float l_speed)
{
  t_global_direction glob_nextdir;
  t_local_direction temp_next_dir;
  int straight_count = 0;

  switch (g_map.nextDirGet(gx, gy, &glob_nextdir)) {
    case shortcut_front:
    case front:
      break;
    case shortcut_right:
    case right:
      g_run.rotate(right, 1);
      break;
    case shortcut_left:
    case left:
      g_run.rotate(left, 1);
      break;
    case shortcut_rear:
    case rear:
      g_run.rotate(right, 2);
      break;
  }

  g_run.accelerate(HALF_SECTION, l_speed, l_accel);

  g_map.mypos.dir = glob_nextdir;
  g_map.axisUpdate();

  while ((g_map.mypos.x != gx) || (g_map.mypos.y != gy)) {
    if (straight_count == 0) {
      g_map.wallSet(g_sensor.sen_fr.is_wall, g_sensor.sen_r.is_wall, g_sensor.sen_l.is_wall);
    }

    switch (g_map.nextDirGet(gx, gy, &glob_nextdir)) {
      case front:
        if (straight_count > 0) {
          g_run.straight(straight_count * SECTION, l_speed, g_run.max_speed, l_speed, l_accel);
          straight_count = 0;
        }
        g_run.oneStep(SECTION, l_speed);
        break;
      case shortcut_right:
      case right:
        if (straight_count > 0) {
          g_run.straight(straight_count * SECTION, l_speed, g_run.max_speed, l_speed, l_accel);
          straight_count = 0;
        }
        g_run.decelerate(HALF_SECTION, l_speed, l_accel);
        g_run.rotate(right, 1);
        g_run.accelerate(HALF_SECTION, l_speed, l_accel);
        break;
      case shortcut_left:
      case left:
        if (straight_count > 0) {
          g_run.straight(straight_count * SECTION, l_accel, g_run.max_speed, l_speed, l_accel);
          straight_count = 0;
        }
        g_run.decelerate(HALF_SECTION, l_speed, l_accel);
        g_run.rotate(left, 1);
        g_run.accelerate(HALF_SECTION, l_speed, l_accel);
        break;
      case shortcut_rear:
      case rear:
        if (straight_count > 0) {
          g_run.straight(straight_count * SECTION, l_speed, g_run.max_speed, l_speed, l_accel);
          straight_count = 0;
        }
        g_run.decelerate(HALF_SECTION, l_speed, l_accel);
        g_run.rotate(right, 2);
        g_run.accelerate(HALF_SECTION, l_speed, l_accel);
        break;
      case shortcut_front:
        straight_count++;
        break;
    }

    g_map.mypos.dir = glob_nextdir;  //方向を更新
    g_map.axisUpdate();
  }

  if (straight_count > 0) {
    g_run.straight(straight_count * SECTION, l_speed, g_run.max_speed, l_speed, l_accel);
    straight_count = 0;
  }

  g_map.wallSet(g_sensor.sen_fr.is_wall, g_sensor.sen_r.is_wall, g_sensor.sen_l.is_wall);
  g_run.decelerate(HALF_SECTION, l_speed, l_accel);
}