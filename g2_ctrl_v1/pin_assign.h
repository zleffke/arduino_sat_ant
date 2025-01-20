/*
File for Winegard G2 Antenna Controller
Pin Assignments
Licensed under MIT License
*/

//Elevation
#define elDirPin  10
#define elStepPin 9
#define elnSleep  6 //Active Low
#define elEnable  21
#define elnFault  20 //Active Low
#define elCSPin   5

Motor_Pins elPin = {.dir = elDirPin, 
                    .step = elStepPin, 
                    .nSleep = elnSleep, 
                    .enable = elEnable, 
                    .nFault = elnFault, 
                    .chipSelect = elCSPin};

//Azimuth
#define azDirPin  11
#define azStepPin 12
#define aznSleep  13 //Active Low
#define azEnable  A1
#define aznFault  A2 //Active Low
#define azCSPin   A0

Motor_Pins azPin = {.dir = azDirPin, 
                    .step = azStepPin, 
                    .nSleep = aznSleep, 
                    .enable = azEnable, 
                    .nFault = aznFault, 
                    .chipSelect = azCSPin};

//Other Pins
//This development is on an Adafruit Feather M0 Adalogger, 
//pin assignments might be different for other devices
#define LEDGreen 8 //Green, Driver Enabled, Motion possible
#define LEDRed 13 //Green, Driver Enabled Motion Happening

void initialize_pins(){
  //Azimuth Pins
  pinMode(azPin.dir, OUTPUT);
  digitalWrite(azPin.dir, LOW);
  pinMode(azPin.step, OUTPUT);
  digitalWrite(azPin.step, LOW);
  pinMode(azPin.nSleep, OUTPUT);
  digitalWrite(azPin.nSleep, LOW);
  pinMode(azPin.enable, OUTPUT);
  digitalWrite(azPin.enable, LOW);
  delay(100);
  digitalWrite(azPin.nSleep, HIGH);
  digitalWrite(azPin.enable, HIGH);
  // pinMode(azPin.chipSelect, OUTPUT);
  pinMode(azPin.nFault, INPUT);
  digitalWrite(azPin.nFault, HIGH); //enable pullup
  //Elevation Pins
  pinMode(elPin.dir, OUTPUT);
  digitalWrite(elPin.dir, LOW);
  pinMode(elPin.step, OUTPUT);
  digitalWrite(elPin.step, LOW);
  pinMode(elPin.nSleep, OUTPUT);
  digitalWrite(elPin.nSleep, LOW);
  pinMode(elPin.enable, OUTPUT);
  digitalWrite(elPin.enable, LOW);
  delay(100);
  digitalWrite(elPin.nSleep, HIGH);
  digitalWrite(elPin.enable, HIGH);
  // pinMode(elPin.chipSelect, OUTPUT);
  pinMode(elPin.nFault, INPUT);
  digitalWrite(elPin.nFault, HIGH); //enable pullup

}