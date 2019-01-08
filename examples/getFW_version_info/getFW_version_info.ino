/**************************************************
*!
 * @file 3D_GestureTest.ino
 * @brief 
 *         The program 3D gesture recognition at a distance of 3-dimensional space induction
 *        
 * @n [Get the module here](http://wiki.dfrobot.com.cn/index.php?title=(SKU:SEN0202)3D%E6%89%8B%E5%8A%BF%E8%AF%86%E5%88%AB%E8%BF%B7%E4%BD%A0%E4%BC%A0%E6%84%9F%E5%99%A8%E6%A8%A1%E5%9D%97)
 *
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
int testPin = 7;
DFRobot_Gesture myGesture;
void setup()
{
    Wire.begin();        // join i2c bus (address optional for master)
    Serial.begin(9600);  // start serial for output
    pinMode(testPin, INPUT);
    Serial.write("3D Proximity sensor is now running....\r\n");  
}

void loop()
{
    myGesture.getFW_Version_Info();
    delay(1000);
}

