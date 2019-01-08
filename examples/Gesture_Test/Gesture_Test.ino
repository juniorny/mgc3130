/**************************************************
*!
 * @file 3D_GestureTest.ino
 * @brief 
 *         this procedure to achieve 3D gesture recognition can be achieved in 3-dimensional 
 *         space,up, down, left,right,Counterclockwise circle recognition and Clockwise circle recognition
 * @n [Get the module here](http://wiki.dfrobot.com.cn/index.php?title=(SKU:SEN0202)3D%E6%89%8B%E5%8A%BF%E8%AF%86%E5%88%AB%E8%BF%B7%E4%BD%A0%E4%BC%A0%E6%84%9F%E5%99%A8%E6%A8%A1%E5%9D%97)
 * @copyright [DFRobot](http://www.dfrobot.com), 2016
 * @copyright GNU Lesser General Public License
 *
 * @author [Carl](lei.wu@dfrobot.com)
 * @version  V1.0
 * @date  2016-09-9

Hardware Connections:
Arduino Pin  3D Gesture Sensor-mini  Board  Function
 GND          GND              Ground
 3.3V-5V      VCC              Power
 A5           SCL              I2C Clock
 A4           SDA              I2C Data
 D7           D                Digital port
 
***************************************************/
#include <DFRobot_Gesture.h>
#include <Wire.h>

int testPin= 7;
unsigned char cmd;

DFRobot_Gesture myGesture;

void setup()
{
  Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(9600);  // start serial for output
 pinMode(testPin, INPUT);  
 Serial.write("3D Gesture sensor is now running....\r\n");  

// myGesture.MGC3130_sendMessage(disable_touch_detection, disable_touch_detection[0]);
// myGesture.MGC3130_sendMessage(enable_approach_detection, enable_approach_detection[0]);
 myGesture.MGC3130_sendMessage(enable_airwheel, enable_airwheel[0]);
 myGesture.MGC3130_sendMessage(gesture_processing_hmm, gesture_processing_hmm[0]);
 myGesture.MGC3130_sendMessage(data_output_enable_mask, data_output_enable_mask[0]);
 myGesture.MGC3130_sendMessage(getFW_Version_Info, getFW_Version_Info[0]);

}


void loop()
{

  if(digitalRead(testPin)==0)
  {
     
   myGesture.I2C1_MasterRead(md.buf, 32, MGC3130_I2C_ADDR); //The address is:0x42
   cmd = myGesture.mgcProcMsg();     //process the message
          if(cmd != GI_NOGESTURE )
            {
                switch(cmd)
                {
                 
                    case GI_FLICK_R:
                      Serial.println("RIGHT");
                        break;

                    case GI_FLICK_L:
                        Serial.println("LEFT");
                        break;

                    case GI_FLICK_D:
                     Serial.println("DOWN");
                        break;

                    case GI_FLICK_U:
                       Serial.println("UP");
                        break;
                    case GI_AIRWHEEL_CW://Clockwise in circles
                     Serial.println("CW");
                        break;

                    case GI_AIRWHEEL_CCW://Counterclockwise circles
                       Serial.println("CCW");
                        break;
/*
                   case GI_TAP_SOUTH:
                       Serial.println("GI_TAP_SOUTH");
                       break;

                   case GI_TAP_WEST:
                       Serial.println("GI_TAP_WEST");
                       break;

                   case GI_TAP_NORTH:
                       Serial.println("GI_TAP_NORTH");
                       break;

                   case GI_TAP_EAST:
                       Serial.println("GI_TAP_EAST");
                       break;

                  case GI_TAP_CENTER:
                       Serial.println("GI_TAP_CENTER");
                       break;
*/
                    default: break;
                }
            }
  }
  
 else  {};

}
