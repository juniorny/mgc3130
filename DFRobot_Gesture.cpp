/*
* @file DFRobot_Gesture.cpp
* @brief DFRobot_Gesture.cpp   3D Gesture Sensor-mini Library file
*
*  DFRobot_Gesture.cpp 
*  Read gesture recognition and sensor data processing
* 
* @author leiwu(lei.wu@dfrobot.com)
* @version  V1.0
* @date  2015-12-11
*/

 #include <Arduino.h>
 #include "DFRobot_Gesture.h"
 #include <Wire.h>

typedef unsigned char uint8_t;
typedef unsigned long int uint32_t;
typedef unsigned int uint16_t;
static uint8_t AirWheelActivePrevious = 0;   // AirWheel status in the previous run
static int16_t AirWheelValuePrevious=0;                 // AirWheel counter value each time that the flag in the SystemInfo goes from 0 to 1
static int16_t AirWheelDiff = 0; 

unsigned char *cRecData=md.buf;
union data md;

void DFRobot_Gesture::I2C1_MasterRead( uint8_t *pdata,uint8_t reclength,uint16_t address)
{
	unsigned char i=0;
	Wire.requestFrom(address,reclength);    // request 6 bytes from slave device #2

	md.buf[0] = 4; // read at least header, but update after first read anyway

	while (Wire.available() && (i < md.buf[0]))   // slave may send less than requested
	{
		*pdata = Wire.read(); // receive a byte as character
//		Serial.print(*pdata, HEX);
//		Serial.print(" ");
		pdata++;
		i++;
	}
//	Serial.println("");
	delay(1);
}

uint8_t DFRobot_Gesture::mgcProcMsg(void)
{
     // bits 0 to 4 must be set in the StreamOutputMask to have valid data to read
//   uint16_t streamOutRequired = (STREAMOUT_DSPINFO | STREAMOUT_GESTUREINFO | STREAMOUT_TOUCHINFO | STREAMOUT_AIRWHEELINFO | STREAMOUT_XYZPOSITION);
   uint16_t streamOutRequired = (STREAMOUT_GESTUREINFO | STREAMOUT_AIRWHEELINFO);

    uint8_t retVal = GI_NOGESTURE;


    if(md.header.id != MSGID_SENSORDATAOUT)
        return GI_NOGESTURE;

//    if(md.header.size < 22 )  // message too short for our sensor data configuration
//        return GI_NOGESTURE;              //DSP(2)+Gesture(4)+Touch(4)+AirWheel(2)+XYZPosition(6)+NoisePower(4)= 22bytes

    // ok, we got a sensor data message. Check that the 5 first bits of the StreamOutputMask are set to indicate valid data are present
    if((md.sensorData.streamingOutputMask & streamOutRequired) == streamOutRequired) {

//        uint32_t TouchInfo = md.sensorData.touchInfo;
        uint8_t AirWheelActive = (md.sensorData.systemInfo & SI_AIRWHEELVALID) != 0; // AirWheel is active and valid if bit1 of SystemInfo is set
        int16_t AirWheelValueNew = md.sensorData.airWheelCounter;

        /* FLICK DETECTION */
	retVal = md.sensorData.gestureInfo & 0xFF; // keep only the first byte (b0..b7) that gives the gesture information (in DECIMAL)
/*
        // TAP DETECTION 
        if(TouchInfo & 1<<BITSHIFT_TAP_SOUTH ) {
            retVal = GI_TAP_SOUTH;
        }
        else if(TouchInfo & 1<<BITSHIFT_TAP_WEST) {
            retVal = GI_TAP_WEST;
        }
        else if(TouchInfo & 1<<BITSHIFT_TAP_NORTH) {
            retVal = GI_TAP_NORTH;
        }
        else if(TouchInfo & 1<<BITSHIFT_TAP_EAST) {
            retVal = GI_TAP_EAST;
        }
        else if(TouchInfo & 1<<BITSHIFT_TAP_CENTER) {
            retVal = GI_TAP_CENTER;
        }
        else {}
*/
        /* AIRWHEEL DETECTION */
        //store the airwheel counter when the airwheel is started
        if (AirWheelActive && !AirWheelActivePrevious)
        {
            AirWheelValuePrevious = md.sensorData.airWheelCounter;
        }
        else if (AirWheelActive)
        {
            if( AirWheelValuePrevious < 64 && AirWheelValueNew > 192 ){
                    // "crossing zero point" happened
                    AirWheelDiff += ( (AirWheelValueNew-256) - AirWheelValuePrevious);
            }
            else if( AirWheelValuePrevious > 192 && AirWheelValueNew < 64 ){
                    // "crossing zero point" happened
                    AirWheelDiff += ( (AirWheelValueNew+256) - AirWheelValuePrevious);
            }
            else{
                    AirWheelDiff += AirWheelValueNew - AirWheelValuePrevious ;
            }

            if (AirWheelDiff >= 32)
            {
                AirWheelDiff=0;
                retVal = GI_AIRWHEEL_CW;
            }
            else if (AirWheelDiff <= -32)
            {
                AirWheelDiff=0;
                retVal = GI_AIRWHEEL_CCW;
            }
            else{}
        }
        AirWheelActivePrevious = AirWheelActive;    // save the status for the next run
        AirWheelValuePrevious = AirWheelValueNew;
    }
//    Serial.println(retVal, HEX);
    return retVal;
}
	

/******************************************************************************
  Copyright (C) <2015>  <Wulei>
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
  Contact: lei.wu@dfrobot.com
 ******************************************************************************/
uint8_t getFW_Version_Info[] = {0x0C, 0x00, 0x00, 0x06, 0x83, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
// only bit 4:0
uint8_t gesture_processing_hmm[] = {0x10, 0x00, 0x00, 0xA2, 0x85, 0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF};
// arg0 set to 0 is disable, set to 0x08 is enable
uint8_t disable_touch_detection[] = {0x10, 0x00, 0x00, 0xA2, 0x97, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00};

// arg0 set to 0 is disable, set to 0x01 is enable
uint8_t enable_approach_detection[] = {0x10, 0x00, 0x00, 0xA2, 0x97, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00};

// arg0 set to 0 is disable, set to 0x20 is enable
uint8_t enable_airwheel[] = {0x10, 0x00, 0x00, 0xA2, 0x90, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00};
 
// gesture and airwheel
uint8_t data_output_enable_mask[] = {0x10, 0x00, 0x00, 0xA2, 0xA0, 0x00, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF};


void DFRobot_Gesture::MGC3130_sendMessage(uint8_t data[], uint8_t length)
{
  	Wire.beginTransmission(MGC3130_I2C_ADDR);
    Wire.write(data, length);
  	Wire.endTransmission();
    delay(2);
    MGC3130_receiveMessage();
}


bool DFRobot_Gesture::MGC3130_receiveMessage()
{
	I2C1_MasterRead(md.buf, 32, MGC3130_I2C_ADDR);

	switch(md.header.id)
	{
		case MGC3130_SYSTEM_STATUS:
	  		if (md.SystemStatus.ErrorCode != NoError)
	  		{
	  			Serial.print("systemStatus ErrorCode = ");
	  			Serial.println(md.SystemStatus.ErrorCode, HEX);
	  			return false;
	  		}
	  		break;	

        case MGC3130_FW_VERSION:
			if (md.fwInfo.valid == 0xaa)
			{
				Serial.print("fwVersion = ");
				Serial.write(md.fwInfo.fwVersion, 120);
				Serial.write("\n");
			}
			else
			{
				Serial.println("fwInfo wrong!");
				return false;
			}
	      	break;	
	}
  	return true;
}

