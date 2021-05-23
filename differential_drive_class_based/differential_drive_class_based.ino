#include "DifferentialDrive.h"

//DifferentialDrive robot(0.135, 0.035);

int left_index = 0;
int idx = 0;

//float velocity_signal[2];
float pose[3];

void setup() {
  // put your setup code here, to run once:
  Serial.begin(2400);
  robot.init_ISR();
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available() > 0) {
    left_index = 0;
    idx = 0;
    String data = Serial.readStringUntil('\n');
    for(int i = 0; i < data.length(); i++){
      if (data[i] == ','){
        float val = data.substring(left_index, i).toFloat();
        Serial.println(val);
        pose[idx] = val;
        idx ++;
        left_index = i+1;
      }
    }
    robot.move_it(pose[0], pose[1], pose[2]);
  }
}
