// Author: Bruce Noble
// Team: Articulated Removable Manipulator (ARM), 2025-2026
// Part of the ARM project is the integration of a force sensor and serial communications for each of the dynamixel servos
// This script starts by initializing the servos to default positions and calibrating the force sensor
// The script then waits to recieve a command over serial, and parses the command for position and force commands as well as servo velocities
// After writing and executing the commands it will send back a confirmation of the current servo positions and velocities.

// ----- SETUP -----
// include servo packages
#include <Dynamixel2Arduino.h>
#include <string.h>

#define DXL_SERIAL Serial1  // OpenCM 9.04 dxl serial
// #define DEBUG_SERIAL Serial2

using namespace ControlTableItem;

const int DXL_DIR_PIN = 28, FS_PIN = A0;  // DXL and force sensor pins
const float DXL_PROTOCOL_VERSION1 = 1.0, DXL_PROTOCOL_VERSION2 = 2.0;

const int open_factor = 4, grip_trig = 200, grip_safe_speed = 1297, e_fs = 102;  // 102 = 0.1N, grip_safe_speed = 273

// vectors to store position and velocity data
// int goalPos[6] = {154, 343, 165, 188, 157, 0}, maxVel[6] = {30, 30, 30, 30, 30, 30}; // convert to ticks!
// int goalPos_last[6], maxVel_last[6];
int goalPos[6] = {2048, 2048, 2048, 614, 614, 0}, maxVel[6] = {32, 23, 23, 82, 82, 1297};
int goalPos_last[6] = {2048, 2048, 2048, 614, 614, 0}, maxVel_last[6] = {32, 23, 23, 82, 82, 1297};

int grip_rot, fs, send_freq = 20;  // t_prev, t_now;
bool grip_present = false, grip_state = false;

bool new_command = false;

Dynamixel2Arduino dxl1(DXL_SERIAL, DXL_DIR_PIN);
Dynamixel2Arduino dxl2(DXL_SERIAL, DXL_DIR_PIN);

// ----- INITIALIZATION -----
void setup() {
  // put your setup code here, to run once:
  // Start COMM Serial
  Serial.begin(115200);
  while (!Serial)
    ;
  // Configure Force Sensor
  pinMode(FS_PIN, INPUT_ANALOG);
  // Initialize Dynamixels
  dxl_init();
  // t_prev = millis();
}

// ----- MAIN -----
void loop() {
  // read full command
  if (Serial.available()) {
    receive();  // sets new_command = true when a full 12‑value packet arrives
  }

  // only act when a new command was received
  if (new_command) {

    if (vector_compare(goalPos, goalPos_last) || vector_compare(maxVel, maxVel_last)) {


      joints_write();

      grip_present = dxl1.ping(6);
      if (grip_present) {
        grip_write();
      }

      for (int i = 0; i < 6; i++) {
        goalPos_last[i] = goalPos[i];
        maxVel_last[i] = maxVel[i];
      }
    }

    send();  // feedback only when a command was processed
    new_command = false;
  }
}

// ----- FUNCTIONS -----

void dxl_init() {
  // Set Port Baudrates to 1Mbps
  DXL_SERIAL.begin(1000000);
  dxl1.begin(1000000);
  dxl2.begin(1000000);

  // Configure Protocol 2 Servos
  dxl2.setPortProtocolVersion(DXL_PROTOCOL_VERSION2);
  for (int i = 1; i < 4; i++) {
    dxl2.torqueOn(i);
    dxl2.ping(i);
  }
  // Configure Protocol 1 Servos
  dxl1.setPortProtocolVersion(DXL_PROTOCOL_VERSION1);
  for (int i = 4; i < 6; i++) {  // all except gripper (ID 6)
    dxl1.torqueOff(i);
    dxl1.ping(i);
    // for joint mode (limited rotation),
    dxl1.writeControlTableItem(CW_ANGLE_LIMIT, i, 0);      // CW angle limit = 0 (min)
    dxl1.writeControlTableItem(CCW_ANGLE_LIMIT, i, 1023);  // CCW angle limit = 300 (max)
    dxl1.torqueOn(i);
  }
  // Configure Gripper Servo
  // gripper has to be set in wheel mode since it spins more than 360 deg
  dxl1.torqueOff(6);
  grip_present = dxl1.ping(6);
  dxl1.writeControlTableItem(CW_ANGLE_LIMIT, 6, 0);
  dxl1.writeControlTableItem(CCW_ANGLE_LIMIT, 6, 0);
  dxl1.torqueOn(6);

  // write configuration values
  joints_write();
  if (grip_present) {
    grip_init();
  }
}

bool vector_compare(const int* a, const int* b) {
  for (int i = 0; i < 6; i++) {
    if (a[i] != b[i]) {
      return true;
    }
  }
  return false;
}

void joints_write() {
  // Write Protocol 2 Servos
  for (int i = 0; i < 3; i++) {
    int id = i + 1;
    dxl2.writeControlTableItem(PROFILE_VELOCITY, id, maxVel[i]);
    dxl2.writeControlTableItem(GOAL_POSITION, id, goalPos[i]);
  }

  // Write Protocol 1 Servos
  for (int i = 3; i < 5; i++) {  // all except the gripper
    int id = i + 1;
    dxl1.writeControlTableItem(MOVING_SPEED, id, maxVel[i]);
    dxl1.writeControlTableItem(GOAL_POSITION, id, goalPos[i]);
  }
}

void grip_init() {
  int grip_command = 600;  // 0 - 1023, 0N - 1N
  dxl1.writeControlTableItem(MOVING_SPEED, 6, grip_safe_speed);
  while (!grip_state) {
    // close gripper
    fs = analogRead(FS_PIN);  // 0 - 1023, 0V - 3.3V, 0N - 1N
    if (fs > (grip_command - e_fs)) {
      grip_state = true;
    }
  }
  dxl1.writeControlTableItem(MOVING_SPEED, 6, 0);
  // when gripper closed, mark grip_rot as 0
  grip_rot = 0;
  // after gripper has closed, open it, and keep track of rotations
  grip_open();
}

void grip_write() {
  int grip_command = goalPos[5];  // 0 - 1023, 0N - 1N
  fs = analogRead(FS_PIN);        // 0 - 1023, 0V - 3.3V, 0N - 1N
  // check if gripper commanded open
  if (grip_command == 0) {
    // open gripper
    grip_open();
  }
  // if gripper is not closed, close
  else if (!grip_state) {  // check what we want cutoff to be, fs = 0 - 1023
    // close gripper until force sensor reads within range
    int pos_last = dxl1.getPresentPosition(6);
    dxl1.writeControlTableItem(MOVING_SPEED, 6, grip_safe_speed);
    while (!grip_state) {
      int pos_current = dxl1.getPresentPosition(6);
      int diff = abs(pos_current - pos_last);
      if (diff > grip_trig) {
        grip_rot -= 1;
      }
      pos_last = pos_current;
      fs = analogRead(FS_PIN);  // 0 - 1023, 0V - 3.3V, 0N - 1N
      if (fs >= (grip_command - e_fs)) {
        grip_state = true;
      }
    }
    dxl1.writeControlTableItem(MOVING_SPEED, 6, 0);
    grip_state = true;
  }
  // if gripper is closed too much, open slowely a bit
  else if (grip_state && (fs >= (grip_command + e_fs))) {
    // not very likely to enter this state
    // open gripper very slowely
    int pos_last = dxl1.getPresentPosition(6);
    dxl1.writeControlTableItem(MOVING_SPEED, 6, (grip_safe_speed - 1024));  // 1024 - 2047 represents ccw rotation
    while (grip_rot < open_factor) {
      int pos_current = dxl1.getPresentPosition(6);
      int diff = abs(pos_last - pos_current);
      if (diff > grip_trig) {
        grip_rot += 1;
      }
      pos_last = pos_current;
    }
    dxl1.writeControlTableItem(MOVING_SPEED, 6, 1025);
  } else {
    // wtf? how did you even get here?
  }
}

void grip_open() {
  // open the gripper to at least the maximum
  int pos_last = (int)dxl1.getPresentPosition(6);
  dxl1.writeControlTableItem(MOVING_SPEED, 6, (grip_safe_speed - 1024));  // 1024 - 2047 represents ccw rotation
  while (grip_rot < 8) {
    int pos_current = (int)dxl1.getPresentPosition(6);
    int diff = abs(pos_current - pos_last);
    if (diff > grip_trig) {
      grip_rot += 1;
    }
    pos_last = pos_current;
    // Serial.print("Count: ");
    // Serial.print(grip_rot);
    // Serial.print("  Last: ");
    // Serial.print(pos_last);
    // Serial.print("  Current: ");
    // Serial.print(pos_current);
    // Serial.print("  Diff: ");
    // Serial.println(diff);
  }
  dxl1.writeControlTableItem(MOVING_SPEED, 6, (1024));
  grip_state = false;
}

void receive() {
  static char buffer[80];
  static int index = 0;

  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n') {
      buffer[index] = '\0';

      char* token = strtok(buffer, ",");
      int count = 0;

      while (token != NULL && count < 12) {
        if (count < 6) {
          goalPos[count] = atoi(token);
        } else {
          maxVel[count - 6] = atoi(token);
        }

        token = strtok(NULL, ",");
        count++;
      }

      if (count == 12) {
        new_command = true;  // full valid packet received
      }

      index = 0;
      return;
    }

    if (index < 79) {
      buffer[index++] = c;
    }
  }
}

void send() {
  // send back periodically to confirm the dynamixels are actually moving as intended
  String posMessage = "", message, velMessage = "";
  int currentPos[6];                // , currentVel[6];
  for (int i = 0; i < 3; i++) {
    posMessage += String((int)dxl2.getPresentPosition((i + 1))) + ",";
    velMessage += String(dxl2.getPresentVelocity((i + 1))) + ",";
  }
  for (int i = 3; i < 5; i++) {
    posMessage += String((int)dxl1.getPresentPosition((i + 1))) + ",";
    velMessage += String(dxl1.getPresentVelocity((i + 1))) + ",";
  }
  posMessage += String(analogRead(A0)) + ",";  // force sensor value for gripper
  velMessage += String(dxl1.getPresentVelocity((6)));
  // P1,P2,P3,P4,P5,fs\n // P1,P2,P3,P4,P5,P6,V1,V2,V3,V4,V5,V6\n
  // velMessage.remove(velMessage.length() - 1);
  message = posMessage + velMessage + "\n";  // + velMessage + "\n";
  Serial.write((const uint8_t*)message.c_str(), message.length());
  Serial.flush();
}