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

#ifndef SRC_FAST_H_
#define SRC_FAST_H_

typedef enum {
  R90,
  L90,
  R180,
  L180,
  R45,
  L45,
  R135,
  L135,
  R90V,
  L90V,
  R90H,
  L90H,
  EOF127=127
} t_sura_mode;


class FAST
{
public:
  void run(short gx, short gy,float l_accel, float l_speed);
  void runSura(short gx, short gy,float l_accel, float l_speed);
  void patternMake(short gx, short gy);
  void runPatternSura(short gx, short gy,float l_accel, float l_speed);
  unsigned char second_pattern[256];
private:
  void changeHighSura(void);
};

extern FAST g_fast;

#endif /* SRC_FAST_H_ */
