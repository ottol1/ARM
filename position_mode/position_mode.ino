/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#include <Dynamixel2Arduino.h>

#define DXL_SERIAL   Serial1 //OpenCM9.04 EXP Board's DXL port Serial. (Serial1 for the DXL port on the OpenCM 9.04 board)
#define DEBUG_SERIAL Serial
const int DXL_DIR_PIN = 28; //OpenCM9.04 EXP Board's DIR PIN. (28 for the DXL port on the OpenCM 9.04 board)
 

const float DXL_PROTOCOL_VERSION1 = 1.0;
const float DXL_PROTOCOL_VERSION2 = 2.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

//This namespace is required to use Control table item names
using namespace ControlTableItem;

  // Set goal positions for all six joints (in degress)
  float goalPos[] = {180,180,180,180,180,180};

  // Set max velocities for all six Joints (masx = 0)
  int maxVel[] = {0,0,0,0,0,0};

void setup() {
  // put your setup code here, to run once:
  
  // Use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(115200);
  while(!DEBUG_SERIAL);

  // Set Port baudrate to 1000000bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(1000000);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  // Get DYNAMIXEL information
  for(int i = 1; i < 7; i++){
    if (i < 4) {
      dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION2);
    } else { 
      dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION1);
    }
    dxl.torqueOn(i);
    dxl.ping(i);
  }

  for (int i = 1; i < 7; i++) {
    if (i < 4) {
      dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION2);
      dxl.writeControlTableItem(PROFILE_VELOCITY, i, maxVel[i-1]);
    } else { 
      dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION1);
      dxl.writeControlTableItem(MOVING_SPEED, i, maxVel[i-1]);
    }

    // Set Goal Position in DEGREE value
    dxl.setGoalPosition(i, goalPos[i-1], UNIT_DEGREE);
  }
}

void loop() {
  
}
