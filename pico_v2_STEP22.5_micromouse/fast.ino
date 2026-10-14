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

void FAST::run(short gx, short gy,float l_accel, float l_speed)
{
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

  g_run.accelerate(HALF_SECTION, l_speed,l_accel);
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
            straight_count * SECTION, l_speed, g_run.max_speed, l_speed,l_accel);
          straight_count = 0;
        }
        g_run.decelerate(HALF_SECTION, l_speed,l_accel);
        g_run.rotate(right, 1);
        g_run.accelerate(HALF_SECTION, l_speed,l_accel);
        break;
      case left:
        if (straight_count > 0) {
          g_run.straight(
            straight_count * SECTION,l_speed, g_run.max_speed, l_speed,l_accel);
          straight_count = 0;
        }
        g_run.decelerate(HALF_SECTION, l_speed,l_accel);
        g_run.rotate(left, 1);
        g_run.accelerate(HALF_SECTION, l_speed,l_accel);
        break;
      default:
        break;
    }
    g_map.mypos.dir = glob_nextdir;
    g_map.axisUpdate();
  }
  if (straight_count > 0) {
    g_run.straight(
      straight_count * SECTION, l_speed, g_run.max_speed,l_speed,l_accel);
  }
  g_run.decelerate(HALF_SECTION, l_speed,l_accel);
}


void FAST::runSura(short gx, short gy,float l_accel, float l_speed)
{
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

  g_run.accelerate(HALF_SECTION, l_speed,l_accel);
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
            straight_count * SECTION, l_speed, g_run.max_speed, l_speed,l_accel);
          straight_count = 0;
        }
        g_run.sura(R90, l_speed);
        break;
      case left:
        if (straight_count > 0) {
          g_run.straight(
            straight_count * SECTION,l_speed, g_run.max_speed, l_speed,l_accel);
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
      straight_count * SECTION, l_speed, g_run.max_speed,l_speed,l_accel);
  }
  g_run.decelerate(HALF_SECTION, l_speed,l_accel);
}


void FAST::patternMake(short gx, short gy)
{
  t_global_direction glob_nextdir;
  int straight_count = 0;
  int i=0;

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

void FAST::changeHighSura(void){
  unsigned char j=0;
  unsigned char i=0;

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

void FAST::runPatternSura(short gx, short gy,float l_accel, float l_speed){
  int i =0;
  t_global_direction dir;
  char old_turn=0;
  int length_ago,length_next;
  float speed_ago,speed_next;


  patternMake(gx,gy);
  if((g_misc.mode_select>=8)&&(g_misc.mode_select<=11)){
    changeHighSura();
  }
  g_run.accelerate(HALF_SECTION, l_speed,l_accel);

  while(1){
    if (second_pattern[i] > 0) {//直進
      if((second_pattern[i+1]==R90H)||(second_pattern[i+1]==L90H)){
        length_next=-15;
        speed_next=75;
      }else if((second_pattern[i+1]==R180)||(second_pattern[i+1]==L180)){
        length_next=-15;
        speed_next=50;
      }else{
        length_next=0;
        speed_next=0;
      }
      if((i>1)&&((second_pattern[i-1]==R90H)||(second_pattern[i-1]==L90H))){
        length_ago=-15;
        speed_ago=75;
      }else if((i>1)&&((second_pattern[i-1]==R180)||(second_pattern[i-1]==L180))){
        length_ago=-15;
        speed_ago=50;
      }else{
        length_ago=0;
        speed_ago=0;
      }
      
      g_run.straight(SECTION * second_pattern[i]+length_ago+length_next , l_speed+speed_ago, g_run.max_speed, l_speed+speed_next,l_accel);
    }
    i++;
    if (second_pattern[i] == 127) {
      break;
    } else {
      switch(second_pattern[i]){
        case R90:
        g_run.sura(R90, l_speed);
        break;
        case L90:
        g_run.sura(L90, l_speed);
        break;
        case R90H:
        g_run.sura(R90H, l_speed+75);
        break;
        case L90H:
        g_run.sura(L90H, l_speed+75);
        break;
        case R180:
        g_run.sura(R180, l_speed+50);
        break;
        case L180:
        g_run.sura(L180, l_speed+50);
        break;
      }
    }
    i++;
  } 

  g_run.decelerate(HALF_SECTION, l_speed,l_accel);
}
