/***********************************************************************************
 *    (c) Tuxcnc 2025
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program; if not, write to the Free Software
 *    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 ***********************************************************************************/


// Hardware : ESP32C3 Dev Module 

#define forward_pin 5
#define reverse_pin 6
#define dir_pin     3
#define step_pin    4
#define encA_pin    0
#define encB_pin    1
#define encZ_pin    2
#define steplen     8    //increase for lower rpm
#define microstep   16   //as at stepper driver
#define cpr         200*microstep
uint8_t dir = 0;
uint8_t speed = 0;
uint8_t cmd = 0;
uint8_t enc = 0;
int16_t enc_count = 0;
bool enc_A[4] = {0,0,1,1};
bool enc_B[4] = {0,1,1,0};


void setup() {
  pinMode(forward_pin, INPUT_PULLDOWN);
  pinMode(reverse_pin, INPUT_PULLDOWN);
  pinMode(dir_pin, OUTPUT);
  pinMode(step_pin, OUTPUT);
  pinMode(encA_pin, OUTPUT);
  pinMode(encB_pin, OUTPUT);
  pinMode(encZ_pin, OUTPUT);
}

void loop() {
  while (speed == 0){
    if (digitalRead(forward_pin)){
      dir = 1;
      speed = 1;
      digitalWrite(dir_pin, 1);
    }
    if (digitalRead(reverse_pin)){
      dir = 2;
      speed = 1;
      digitalWrite(dir_pin, 0);
    }
  }
  digitalWrite(step_pin, 1);
  delayMicroseconds(steplen);
  digitalWrite(step_pin, 0);
  delayMicroseconds((256*steplen) - (steplen*speed));
  if (dir == 1) {
    enc++;
    enc_count++;
    if (enc_count > (cpr - 1)) {enc_count = 0;}
    digitalWrite(encA_pin, enc_A[(enc & 3)]);
    digitalWrite(encB_pin, enc_B[(enc & 3)]);
  }
  if (dir == 2) {
    enc--;
    enc_count--;
    if (enc_count < 0) {enc_count = (cpr - 1);}
    digitalWrite(encA_pin, enc_A[(enc & 3)]);
    digitalWrite(encB_pin, enc_B[(enc & 3)]);
  }
  if (enc_count < 2) { digitalWrite(encZ_pin, 1);}
  else { digitalWrite(encZ_pin, 0);}  
  if (((digitalRead(forward_pin)) && (dir == 1)) || ((digitalRead(reverse_pin)) && (dir == 2))) {
    if (speed < 255){ speed++;}
  }
  else {
    if (speed > 0){ speed--;}
  }
}
