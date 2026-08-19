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

FAST g_fast;

void FAST::run(short gx, short gy, float l_accel, float l_speed) {
  t_global_direction glob_nextdir;
  int straight_count = 0;

  switch (g_map.nextDir2Get(gx, gy, &glob_nextdir)) {
    case right:
      g_run.rotate(right, 1);  //右に曲がって
      break;
    case left:
      g_run.rotate(left, 1);  //左に曲がって
      break;
    case rear:
      g_run.rotate(right, 2);  //180度に旋回して
      break;
    default:
      break;
  }

  g_run.accelerate(HALF_SECTION, l_speed, l_accel);
  straight_count = 0;
  g_map.mypos.dir = glob_nextdir;
  g_map.axisUpdate();

  while ((g_map.mypos.x != gx) || (g_map.mypos.y != gy)) {
    switch (g_map.nextDir2Get(gx, gy, &glob_nextdir)) {
      case front:
        straight_count++;
        break;
      case right:
        if (straight_count > 0) {
          g_run.straight(
            straight_count * SECTION, l_speed, g_run.max_speed, l_speed, l_accel);
          straight_count = 0;
        }
        g_run.decelerate(HALF_SECTION, l_speed, l_accel);
        g_run.rotate(right, 1);
        g_run.accelerate(HALF_SECTION, l_speed, l_accel);
        break;
      case left:
        if (straight_count > 0) {
          g_run.straight(
            straight_count * SECTION, l_speed, g_run.max_speed, l_speed, l_accel);
          straight_count = 0;
        }
        g_run.decelerate(HALF_SECTION, l_speed, l_accel);
        g_run.rotate(left, 1);
        g_run.accelerate(HALF_SECTION, l_speed, l_accel);
        break;
      default:
        break;
    }
    g_map.mypos.dir = glob_nextdir;
    g_map.axisUpdate();
  }
  if (straight_count > 0) {
    g_run.straight(
      straight_count * SECTION, l_speed, g_run.max_speed, l_speed, l_accel);
  }
  g_run.decelerate(HALF_SECTION, l_speed, l_accel);
}


void FAST::runSura(short gx, short gy, float l_accel, float l_speed) {
  t_global_direction glob_nextdir;
  int straight_count = 0;

  switch (g_map.nextDir2Get(gx, gy, &glob_nextdir)) {
    case right:
      g_run.rotate(right, 1);  //右に曲がって
      break;
    case left:
      g_run.rotate(left, 1);  //左に曲がって
      break;
    case rear:
      g_run.rotate(right, 2);  //180度に旋回して
      break;
    default:
      break;
  }

  g_run.accelerate(HALF_SECTION, l_speed, l_accel);
  straight_count = 0;
  g_map.mypos.dir = glob_nextdir;
  g_map.axisUpdate();

  while ((g_map.mypos.x != gx) || (g_map.mypos.y != gy)) {
    switch (g_map.nextDir2Get(gx, gy, &glob_nextdir)) {
      case front:
        straight_count++;
        break;
      case right:
        if (straight_count > 0) {
          g_run.straight(
            straight_count * SECTION, l_speed, g_run.max_speed, l_speed, l_accel);
          straight_count = 0;
        }
        g_run.sura(R90, l_speed);
        break;
      case left:
        if (straight_count > 0) {
          g_run.straight(
            straight_count * SECTION, l_speed, g_run.max_speed, l_speed, l_accel);
          straight_count = 0;
        }
        g_run.sura(L90, l_speed);
        break;
      default:
        break;
    }
    g_map.mypos.dir = glob_nextdir;
    g_map.axisUpdate();
  }
  if (straight_count > 0) {
    g_run.straight(
      straight_count * SECTION, l_speed, g_run.max_speed, l_speed, l_accel);
  }
  g_run.decelerate(HALF_SECTION, l_speed, l_accel);
}


void FAST::patternMake(short gx, short gy) {
  t_global_direction glob_nextdir;
  int straight_count = 0;
  int i = 0;

  g_map.nextDir2Get(gx, gy, &glob_nextdir);
  g_map.mypos.dir = glob_nextdir;
  g_map.axisUpdate();

  while ((g_map.mypos.x != gx) || (g_map.mypos.y != gy)) {
    switch (g_map.nextDir2Get(gx, gy, &glob_nextdir)) {
      case front:
        straight_count++;
        break;
      case right:
        second_pattern[i++] = straight_count;
        second_pattern[i++] = R90;
        straight_count = 0;
        break;
      case left:
        second_pattern[i++] = straight_count;
        second_pattern[i++] = L90;
        straight_count = 0;
        break;
      default:
        break;
    }
    g_map.mypos.dir = glob_nextdir;
    g_map.axisUpdate();
  }
  second_pattern[i++] = straight_count;
  second_pattern[i++] = 127;  //pattern end
}

void FAST::changeHighSura(void) {
  unsigned char j = 0;
  unsigned char i = 0;

  while (1) {
    if ((second_pattern[i] != 0) && (second_pattern[i + 1] == R90) && (second_pattern[i + 2] == 0) && (second_pattern[i + 3] == R90) && (second_pattern[i + 4] != 0)) {
      second_pattern[j++] = second_pattern[i];
      second_pattern[j++] = R180;
      i = i + 4;
    } else if ((second_pattern[i] != 0) && (second_pattern[i + 1] == L90) && (second_pattern[i + 2] == 0) && (second_pattern[i + 3] == L90) && (second_pattern[i + 4] != 0)) {
      second_pattern[j++] = second_pattern[i];
      second_pattern[j++] = L180;
      i = i + 4;
    } else if ((second_pattern[i] != 0) && (second_pattern[i + 1] == R90) && (second_pattern[i + 2] != 0)) {
      second_pattern[j++] = second_pattern[i];
      second_pattern[j++] = R90H;
      i = i + 2;
    } else if ((second_pattern[i] != 0) && (second_pattern[i + 1] == L90) && (second_pattern[i + 2] != 0)) {
      second_pattern[j++] = second_pattern[i];
      second_pattern[j++] = L90H;
      i = i + 2;

    } else if (second_pattern[i + 1] == 127) {  //データの終わり
      second_pattern[j++] = second_pattern[i];
      second_pattern[j++] = 127;
      break;
    } else {
      second_pattern[j++] = second_pattern[i];
      second_pattern[j++] = second_pattern[i + 1];
      i = i + 2;
    }
  }
}

void FAST::changeNanameSura(void) {
  unsigned char j = 0;
  unsigned char i = 0;
  unsigned char n = 0;
  unsigned char n_length = 0;
  unsigned char naname_f = 0;

  while (1) {
    if ((second_pattern[i + 1] == R90) && (second_pattern[i + 2] == 0) && (second_pattern[i + 3] == L90) && (second_pattern[i] != 0)) {
      second_pattern[j++] = second_pattern[i];  //直線
      second_pattern[j++] = R45;                //斜め進入
      n_length = 1;                             //斜めの距離
      n = 0;                                    //R90V,L90Vを行うと斜めの距離の奇数、偶数で判断してR45、L45が逆になる
      i = i + 4;
      naname_f = 1;
    } else if ((second_pattern[i + 1] == L90) && (second_pattern[i + 2] == 0) && (second_pattern[i + 3] == R90) && (second_pattern[i] != 0)) {
      second_pattern[j++] = second_pattern[i];  //直線
      second_pattern[j++] = L45;                //斜め進入
      n_length = 1;                             //斜めの距離
      n = 1;                                    //R90V,L90Vを行うと斜めの距離の奇数、偶数で判断してR45、L45が逆になる
      i = i + 4;
      naname_f = 1;
    } else if ((second_pattern[i + 1] == R90) && (second_pattern[i + 2] == 0) && (second_pattern[i + 3] == R90) 
            && (second_pattern[i + 4] == 0) && (second_pattern[i + 5] == L90) && (second_pattern[i] != 0)) {  //R135
      second_pattern[j++] = second_pattern[i];  //直線
      second_pattern[j++] = R135;
      n_length = 1;
      n = 0;  //R90V,L90Vを行うと斜めの距離の奇数、偶数で判断してR45、L45が逆になる
      i = i + 6;
      naname_f = 1;
    } else if ((second_pattern[i + 1] == L90) && (second_pattern[i + 2] == 0) && (second_pattern[i + 3] == L90) 
            && (second_pattern[i + 4] == 0) && (second_pattern[i + 5] == R90) && (second_pattern[i] != 0)) {  //L135
      second_pattern[j++] = second_pattern[i];  //直線
      second_pattern[j++] = L135;
      n_length = 1;
      n = 1;  //R90V,L90Vを行うと斜めの距離の奇数、偶数で判断してR45、L45が逆になる
      i = i + 6;
      naname_f = 1;
    } else if (second_pattern[i + 1] == 127) {  //データの終わり
      second_pattern[j++] = second_pattern[i];
      second_pattern[j++] = 127;
      break;
    } else {                                        //何もいっちしない つまりふつうのスラローム
      second_pattern[j++] = second_pattern[i];      //直線
      second_pattern[j++] = second_pattern[i + 1];  //曲がりR90 or L90
      i = i + 2;
      naname_f = 0;
    }
    while (naname_f) {
      //斜め終了条件
      if (second_pattern[i] > 0) {  //次の直線が0より大きい場合 斜めを終了
        second_pattern[j++] = n_length;
        if (second_pattern[i - 1] == R90) {
          second_pattern[j++] = R45;
        } else {
          second_pattern[j++] = L45;
        }
        break;
      } else if (second_pattern[i + 1] == 127) {  //data_end
        second_pattern[j++] = n_length;
        if (second_pattern[i - 1] == R90) {
          second_pattern[j++] = R45;
        } else {
          second_pattern[j++] = L45;
        }
        break;
      } else if (second_pattern[i - 1] == second_pattern[i + 1]) {  //second[i]は0よりおおきい　[i-1]と[i+1]が同じ方向に曲がる=R135、L135、R90V、L90Vのいずれか
        if (second_pattern[i + 2] == 0) {                           //second_[i+2]が0より大きい場合、R135 or L135
          second_pattern[j++] = n_length;
          if (second_pattern[i + 3] == 127) {
            if (second_pattern[i + 1] == L90) second_pattern[j++] = L135;
            if (second_pattern[i + 1] == R90) second_pattern[j++] = R135;
            i = i + 2;
            break;
          } else {
            if (second_pattern[i + 1] == L90) second_pattern[j++] = L90V;
            if (second_pattern[i + 1] == R90) second_pattern[j++] = R90V;
          }
          n_length = 0;
          n = (n + 1) & 0x01;
          i = i + 2;
        } else {
          second_pattern[j++] = n_length;
          if (second_pattern[i + 1] == L90) second_pattern[j++] = L135;
          if (second_pattern[i + 1] == R90) second_pattern[j++] = R135;
          i = i + 2;
          break;
        }
      } else {
        n_length++;
        i = i + 2;
      }
    }  //naname_f while end
    naname_f = 0;
  }
}


void FAST::runPatternSura(short gx, short gy, float l_accel, float l_speed) {
  int i = 0;
  t_global_direction dir;
  char old_turn = 0;
  int length_ago, length_next;
  float speed_ago, speed_next;
  naname_f = 0;

  patternMake(gx, gy);

  for (int i = 0; i < 20; i++) {
    Serial.printf("%d ", second_pattern[i]);
  }
  Serial.println();


  if ((g_misc.mode_select >= 8) && (g_misc.mode_select <= 9)) {
    changeHighSura();
  }
  if ((g_misc.mode_select >= 10) && (g_misc.mode_select <= 11)) {
    changeNanameSura();
  }

  for (int i = 0; i < 20; i++) {
    Serial.printf("%d ", second_pattern[i]);
  }

  g_run.accelerate(HALF_SECTION, l_speed, l_accel);

  while (1) {
    if (second_pattern[i] > 0) {  //直進
      if ((second_pattern[i + 1] == R90H) || (second_pattern[i + 1] == L90H)) {
        length_next = -15;
        speed_next = 75;
      } else if ((second_pattern[i + 1] == R180) || (second_pattern[i + 1] == L180)) {
        length_next = -15;
        speed_next = 50;
      } else if ((second_pattern[i + 1] == R45) || (second_pattern[i + 1] == L45)) {
        if (naname_f == 0) {
          length_next = -18;
          speed_next = 0;
        } else {
          length_next = 13;
          speed_next = 0;
        }
      } else if ((second_pattern[i + 1] == R135) || (second_pattern[i + 1] == L135)) {
        if (naname_f == 0) {
          length_next = -18;
          speed_next = 0;
        } else {
          length_next = -13;
          speed_next = 0;
        }
      } else {
        length_next = 0;
        speed_next = 0;
      }

      if ((i > 1) && ((second_pattern[i - 1] == R90H) || (second_pattern[i - 1] == L90H))) {
        length_ago = -15;
        speed_ago = 75;
      } else if ((i > 1) && ((second_pattern[i - 1] == R180) || (second_pattern[i - 1] == L180))) {
        length_ago = -15;
        speed_ago = 50;
      } else if ((i > 1) && ((second_pattern[i - 1] == R45) || (second_pattern[i - 1] == L45))) {
        if (naname_f == 0) {
          length_ago = -18;
          speed_ago = 0;
        } else {
          length_ago = 13;
          speed_ago = 0;
        }
      } else if ((i > 1) && ((second_pattern[i - 1] == R135) || (second_pattern[i - 1] == L135))) {
        if (naname_f == 0) {
          length_ago = -18;
          speed_ago = 0;
        } else {
          length_ago = -13;
          speed_ago = 0;
        }

      } else {
        length_ago = 0;
        speed_ago = 0;
      }

      if (naname_f == 0) {  //直行系
        g_run.straight(SECTION * second_pattern[i] + length_ago + length_next, l_speed + speed_ago, g_run.max_speed, l_speed + speed_next, l_accel);
      } else {
        g_run.straight(NANAME_SECTION * second_pattern[i] + length_ago + length_next, l_speed + speed_ago, g_run.max_speed, l_speed + speed_next, l_accel);
      }
    }
    i++;
    if (second_pattern[i] == 127) {
      break;
    } else {
      switch (second_pattern[i]) {
        case R90:
          g_run.sura(R90, l_speed);
          break;
        case L90:
          g_run.sura(L90, l_speed);
          break;
        case R90H:
          g_run.sura(R90H, l_speed + 75);
          break;
        case L90H:
          g_run.sura(L90H, l_speed + 75);
          break;
        case R180:
          g_run.sura(R180, l_speed + 50);
          break;
        case L180:
          g_run.sura(L180, l_speed + 50);
          break;
        case R45:
          g_run.sura(R45, l_speed);
          if (naname_f == 0) {
            naname_f = 1;
          } else {
            naname_f = 0;
          }
          break;
        case L45:
          g_run.sura(L45, l_speed);
          if (naname_f == 0) {
            naname_f = 1;
          } else {
            naname_f = 0;
          }
          break;
        case R135:
          g_run.sura(R135, l_speed);
          if (naname_f == 0) {
            naname_f = 1;
          } else {
            naname_f = 0;
          }
          break;
        case L135:
          g_run.sura(L135, l_speed);
          if (naname_f == 0) {
            naname_f = 1;
          } else {
            naname_f = 0;
          }
          break;
      }
    }
    i++;
  }

  g_run.decelerate(HALF_SECTION, l_speed, l_accel);
}
