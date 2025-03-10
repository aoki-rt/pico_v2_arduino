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

#include "parameter.h"
#include "run.h"

RUN g_run;

RUN::RUN()
{
  // TODO Auto-generated constructor stub
  speed = 0.0;
  accel = 0.0;
}

RUN::~RUN()
{
  // TODO Auto-generated destructor stub
}

//割り込み
void controlInterrupt(void) { g_run.interrupt(); }

void RUN::interrupt(void)
{  //割り込み内からコール

  speed += accel;

  if (speed > upper_speed_limit) {
    speed = upper_speed_limit;
  }
  if (speed < lower_speed_limit) {
    speed = lower_speed_limit;
  }

  if (con_wall.enable == true) {
    con_wall.p_error = con_wall.error;
    if ((g_sensor.sen_r.is_control == true) && (g_sensor.sen_l.is_control == true)) {
      con_wall.error = g_sensor.sen_r.error - g_sensor.sen_l.error;
    } else {
      con_wall.error = 2.0 * (g_sensor.sen_r.error - g_sensor.sen_l.error);
    }
    con_wall.diff = con_wall.error - con_wall.p_error;
    con_wall.sum += con_wall.error;
    if (con_wall.sum > con_wall.sum_max) {
      con_wall.sum = con_wall.sum_max;
    } else if (con_wall.sum < (-con_wall.sum_max)) {
      con_wall.sum = -con_wall.sum_max;
    }
    con_wall.control = 0.001 * speed * con_wall.kp * con_wall.error;
    speed_target_r = speed + con_wall.control;
    speed_target_l = speed - con_wall.control;
  } else {
    speed_target_r = speed;
    speed_target_l = speed;
  }
  if (speed_target_r < MIN_SPEED) speed_target_r = MIN_SPEED;
  if (speed_target_l < MIN_SPEED) speed_target_l = MIN_SPEED;

  speedSet(speed_target_l, speed_target_r);
}

void RUN::dirSet(t_CW_CCW dir_left, t_CW_CCW dir_right) { motorDirectionSet(dir_left, dir_right); }

void RUN::counterClear(void)
{
  stepClearR();
  stepClearL();
}

void RUN::speedSet(double l_speed, double r_speed)
{
  stepHzSetR(r_speed / pulse);
  stepHzSetL(l_speed / pulse);
}

void RUN::stay(float l_speed)
{
  controlInterruptStop();
  upper_speed_limit = lower_speed_limit = speed = (double)l_speed;
  accel = 0.0;
  counterClear();
  speedSet(l_speed, l_speed);
  controlInterruptStart();
}

void RUN::stepGet(void)
{
  step_lr = stepGetR() + stepGetL();
  step_lr_len = (int)((float)step_lr / 2.0 * pulse);
}

void RUN::stop(void)
{
  motorMoveSet(0);
  delay(300);
}

void straight(int len, int init_speed, int max_sp, int finish_speed)
{
  int obj_step;

  controlInterruptStop();
  upper_speed_limit = (double)max_sp;
  accel = (double)search_accel;

  if (init_speed < MIN_SPEED) {
    speed = (double)MIN_SPEED;
  } else {
    speed = (double)init_speed;
  }
  if (finish_speed < MIN_SPEED) {
    finish_speed = MIN_SPEED;
  }
  lower_speed_limit = (double)finish_speed;

  con_wall.enable = true;
  speedSet(speed, speed);
  dirSet(MOT_FORWARD, MOT_FORWARD);
  obj_step = (int)((float)len * 2.0 / PULSE);
  controlInterruptStart();
  motorMoveSet(1);

  while (1) {
    stepGet();
    if (
      (len - step_lr_len) <
      (int)(((speed * speed) - (lower_speed_limit * lower_speed_limit)) / (2.0 * 1000.0 * accel))) {
      break;
    }
  }

  accel = -1.0 * search_accel;

  while (1) {
    stepGet();
    if (step_lr > obj_step) {
      break;
    }
  }

  if (finish_speed == MIN_SPEED) {
    stop();
  } else {
    stay(finish_speed);
  }
}

void accelerate(int len, int finish_speed)
{
  int obj_step;

  controlInterruptStop();
  g_max_speed = finish_speed;
  g_accel = SEARCH_ACCEL;
  g_speed = g_min_speed = MIN_SPEED;
  setRStepHz((unsigned short)(g_speed / PULSE));
  setLStepHz((unsigned short)(g_speed / PULSE));
  clearStepR();
  clearStepL();
  g_con_wall.enable = true;
  obj_step = (int)((float)len * 2.0 / PULSE);
  moveDir(MOT_FORWARD, MOT_FORWARD);
  controlInterruptStart();

  g_motor_move = 1;

  while ((getStepR() + getStepL()) < obj_step) {
    continue;
  }

  controlInterruptStop();
  g_max_speed = g_min_speed = g_speed = finish_speed;
  g_accel = 0.0;
  clearStepR();
  clearStepL();
  controlInterruptStart();
}

void oneStep(int len, int tar_speed)
{
  int obj_step;
  controlInterruptStop();
  g_speed = g_min_speed = g_max_speed = tar_speed;
  g_accel = 0.0;
  setRStepHz((unsigned short)(g_speed / PULSE));
  setLStepHz((unsigned short)(g_speed / PULSE));
  g_con_wall.enable = true;
  obj_step = (int)((float)len * 2.0 / PULSE);
  moveDir(MOT_FORWARD, MOT_FORWARD);
  controlInterruptStart();

  while ((getStepR() + getStepL()) < obj_step) {
    continue;
  }
  controlInterruptStop();
  g_max_speed = g_min_speed = g_speed = tar_speed;
  g_accel = 0.0;
  clearStepR();
  clearStepL();
  controlInterruptStart();
}

void decelerate(int len, int tar_speed)
{
  int obj_step;
  controlInterruptStop();
  g_max_speed = tar_speed;
  g_accel = 0.0;
  g_speed = g_min_speed = tar_speed;

  setRStepHz((unsigned short)(g_speed / PULSE));
  setLStepHz((unsigned short)(g_speed / PULSE));
  g_con_wall.enable = true;
  obj_step = (int)((float)len * 2.0 / PULSE);
  moveDir(MOT_FORWARD, MOT_FORWARD);
  controlInterruptStart();

  while ((len - (getStepR() + getStepL()) / 2.0 * PULSE) >
         (((g_speed * g_speed) - (MIN_SPEED * MIN_SPEED)) / (2.0 * 1000.0 * SEARCH_ACCEL))) {
    continue;
  }
  g_accel = -1.0 * SEARCH_ACCEL;
  g_min_speed = MIN_SPEED;

  while ((getStepR() + getStepL()) < obj_step) {
    continue;
  }

  g_motor_move = 0;

  delay(300);
}

void rotate(t_direction dir, int times)
{
  int obj_step;
  controlInterruptStop();
  g_max_speed = SEARCH_SPEED;
  g_accel = TURN_ACCEL;
  g_speed = g_min_speed = MIN_SPEED;
  setRStepHz((unsigned short)(g_speed / PULSE));
  setLStepHz((unsigned short)(g_speed / PULSE));
  clearStepR();
  clearStepL();
  g_con_wall.enable = false;
  obj_step = (int)(TREAD_WIDTH * PI / 4.0 * (float)times * 2.0 / PULSE);

  switch (dir) {
    case right:
      moveDir(MOT_FORWARD, MOT_BACK);
      g_motor_move = 1;
      break;
    case left:
      moveDir(MOT_BACK, MOT_FORWARD);
      g_motor_move = 1;
      break;
    default:
      g_motor_move = 0;
      break;
  }
  controlInterruptStart();

  while (((obj_step - (getStepR() + getStepL())) / 2.0 * PULSE) >
         (((g_speed * g_speed) - (MIN_SPEED * MIN_SPEED)) / (2.0 * 1000.0 * TURN_ACCEL))) {
    continue;
  }
  g_accel = -1.0 * TURN_ACCEL;

  while ((getStepR() + getStepL()) < obj_step) {
    continue;
  }

  g_motor_move = 0;
  delay(300);
}
