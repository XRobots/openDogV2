//ODrive
#include <ODriveArduino.h>
//IMU stuff
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

// Radio
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// Pixel display - using Teensy 2812 lib
#include <WS2812Serial.h>
const int numled = 8;
const int pin = 29;
byte drawingMemory[numled*3];         //  3 bytes per LED
DMAMEM byte displayMemory[numled*12]; // 12 bytes per LED
WS2812Serial leds(numled, displayMemory, drawingMemory, pin, WS2812_GRB);
#define RED    0x160000
#define GREEN  0x001600
#define BLUE   0x000016
#define YELLOW 0x101400
#define PINK   0x120009
#define ORANGE 0x100400
#define WHITE  0x101010
#define BLACK  0x000000

RF24 radio(9, 10); // CE, CSN
const byte addresses[][6] = {"00001", "00002"};

// Printing with stream operator
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

int IMUdataReady = 0;
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

float pitch;
float roll;

//ODrive Objects
ODriveArduino odrive1(Serial1);
ODriveArduino odrive2(Serial2);
ODriveArduino odrive3(Serial3);
ODriveArduino odrive4(Serial4);
ODriveArduino odrive5(Serial5);
ODriveArduino odrive6(Serial6);

//**************remote control****************
struct RECEIVE_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to send
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
    int16_t menuDown;  
    int16_t Select;    
    int16_t menuUp;  
    int16_t toggleBottom;  
    int16_t toggleTop; 
    int16_t toggle1;
    int16_t toggle2;
    int16_t mode;  
    int16_t RLR;
    int16_t RFB;
    int16_t RT;
    int16_t LLR;
    int16_t LFB;
    int16_t LT;
};

RECEIVE_DATA_STRUCTURE mydata_remote;

int RLR = 0;
int RFB = 0;
int RT = 0;
int LLR = 0;
int LFB = 0;
int LT = 0;

float RLRFiltered = 0;
float RFBFiltered = 0;
float RTFiltered = 0;
float LLRFiltered = 0;
float LFBFiltered = 0;
float LTFiltered = 0;
int filterFlag1 = 0;

unsigned long currentMillis;
long previousMillis = 0;    // set up timers
long interval = 10;        // time constant for timer
unsigned long count;

unsigned long remoteMillis;


int stepFlag = 0;
long previousStepMillis = 0;
int stepStartFlag = 0;

int requested_state;
int mode;
int modeFlag;
int menuFlag;
int modeConfirm;
int modeConfirmFlag = 0;
int runMode = 0;

int longLeg;
int shortLeg;

int actPos1;
int actPos2;
int actPos3;
int actPos4;

// ODrive offsets from power up once index pulse is found

int offSet10 = -420;      //ODrive 1, axis 0
int offSet11 = -280;      //ODrive 1, axis 1
int offSet20 = -600;      //ODrive 2, axis 0
int offSet21 = -36s00;      //ODrive 2, axis 1 
int offSet30 = -2700;      //ODrive 3, axis 0
int offSet31 = 1500;      //ODrive 3, axis 1
int offSet40 = -450;      //ODrive 4, axis 0
int offSet41 = -420;      //ODrive 4, axis 1
int offSet50 = 1200;      //ODrive 5, axis 0
int offSet51 = -2300;      //ODrive 5, axis 1
int offSet60 = -2350;      //ODrive 6, axis 0
int offSet61 = 3300;      //ODrive 6, axis 1

// ****************** SETUP ******************************

void setup() {

   
    // initialize serial communication
    Serial.begin(115200);

    Serial1.begin(115200);
    Serial2.begin(115200);
    Serial3.begin(115200);
    Serial4.begin(115200);
    Serial5.begin(115200);
    Serial6.begin(115200);

    radio.begin();
    radio.openWritingPipe(addresses[0]); // 00002
    radio.openReadingPipe(1, addresses[1]); // 00001
    radio.setPALevel(RF24_PA_MIN);

    radio.startListening();
    

    // IMU Setup
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
  
    // initialize device
    mpu.initialize();
    devStatus = mpu.dmpInitialize();
  
    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(95);
    mpu.setYGyroOffset(5);
    mpu.setZGyroOffset(59);
    mpu.setXAccelOffset(-1172);
    mpu.setYAccelOffset(-358);
    mpu.setZAccelOffset(1652);  
  
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        mpu.setDMPEnabled(true);
  
        // enable Arduino interrupt detection
        attachInterrupt(33, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
  
        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")")); 
     
    }

    leds.begin();  // init for pixels
    
}   // end of setup

// ********************* MAIN LOOP *******************************

void loop() {  
      
        currentMillis = millis();
        if (currentMillis - previousMillis >= 10) {  // start timed event
          
            previousMillis = currentMillis;

            // check for IMU inpterrupt, read the data if it's ready
            if (IMUdataReady == 1) {
              readAngles();
            }

            // convert angles to degrees for pitch and roll
            roll = (ypr[1] * 180/M_PI);
            pitch = (ypr[2] * 180/M_PI);

            // check for radio data
            if (radio.available()) {
                    radio.read(&mydata_remote, sizeof(RECEIVE_DATA_STRUCTURE));   
                    remoteMillis = currentMillis;  
            }  

            // is the remote disconnected for too long ?
            if (currentMillis - remoteMillis > 500) {
                Serial.println("remote disconnected");
                leds.setPixel(0, GREEN);
                leds.show();
            }

            // threshold remote data
            // some are reversed based on stick wiring in remote
            RFB = (thresholdStick(mydata_remote.RFB))*-1;   
            RLR = thresholdStick(mydata_remote.RLR);
            RT = thresholdStick(mydata_remote.RT);   
            LFB = (thresholdStick(mydata_remote.LFB))*-1;   
            LLR = thresholdStick(mydata_remote.LLR);
            LT = thresholdStick(mydata_remote.LT);                  

            /*  
            // *** read encoder values **           
            Serial1 << "r axis" << 0 << ".encoder.pos_estimate\n";    
            actPos1 = odrive1.readInt();
            Serial1 << "r axis" << 1 << ".encoder.pos_estimate\n";    
            actPos2 = odrive1.readInt();
            Serial4 << "r axis" << 0 << ".encoder.pos_estimate\n";    
            actPos3 = odrive4.readInt();
            Serial4 << "r axis" << 1 << ".encoder.pos_estimate\n";    
            actPos4 = odrive4.readInt();
            */            
            /*
            Serial.print(pitch);
            Serial.print(" , ");
            Serial.print(roll);
            Serial.print(" , ");
            Serial.print(mode); 
            Serial.print(" , ");
            Serial.print(RFB);  
            Serial.print(" , ");
            Serial.print(RLR);
            Serial.print(" , ");
            Serial.print(RT);   
            Serial.print(" , ");
            Serial.print(LFB);  
            Serial.print(" , ");
            Serial.print(LLR);
            Serial.print(" , ");
            Serial.print(LT);  
            Serial.println(" , ");
            Serial.println(mode);
            */


            

            // mode select

            if (mydata_remote.menuUp == 1 && menuFlag == 0) {           
              menuFlag = 1;
              mode = mode+1;
              mode = constrain(mode,0,10);
            }            
            else if (mydata_remote.menuDown == 1 && menuFlag == 0) {
              menuFlag = 1;
              mode = mode-1;
              mode = constrain(mode,0,10);
            }
            else if (mydata_remote.menuDown == 0 && mydata_remote.menuUp == 0){
            menuFlag = 0;    
            }

            binPixel(mode);     // use the function to display potential mode on the pixels

            if (mydata_remote.Select == 1){
              modeConfirm = mode;             // make the actual mode be the potential mode when the select button is pressed
              modeSelect(modeConfirm);        // display actual mode on pixels.        
            } 

            // init ODrives (with serial)

            if (Serial.available()) {
                char c = Serial.read();
   
                if (c == 'a') {
                  OdriveInit1();
                  OdriveInit4();
                }
                else if (c == 'b') {
                  OdriveInit2();
                }
                else if (c == 'c') {
                  OdriveInit3();
                }
                else if (c == 'd') {
                  applyOffsets();
                }
                else if (c == 'e') {
                  modifyGains();
                }
            }

            // remote Menu handling with debounce

            if (mydata_remote.Select == 0) {
              modeConfirmFlag = 0;
            }

            if (modeConfirm == 1 && modeConfirmFlag == 0 && mydata_remote.Select == 1) {
              // **init hips**
              OdriveInit1();    // back hips
              OdriveInit4();    // front hips
              // move to zero position away from stand
              odrive1.SetPosition(0, 0);         // back left leg
              odrive1.SetPosition(1, 0);         // back right leg
              odrive4.SetPosition(0, 0);         // front left leg
              odrive4.SetPosition(1, 0);         // front right leg
              modeConfirmFlag = 1;
            }

            else if (modeConfirm == 2 && modeConfirmFlag == 0 && mydata_remote.Select == 1) {
              // ** init shoulders **
              OdriveInit2();                     
              modeConfirmFlag = 1;
            }

            else if (modeConfirm == 3 && modeConfirmFlag == 0 && mydata_remote.Select == 1) {
              // ** init knees **
              OdriveInit3();                         
              modeConfirmFlag = 1;
            }

            else if (modeConfirm == 4 && modeConfirmFlag == 0 && mydata_remote.Select == 1) {
              // ** apply offsets to get joints to home position **
              applyOffsets();                
              modeConfirmFlag = 1;
            }

            else if (modeConfirm == 5 && modeConfirmFlag == 0 && mydata_remote.Select == 1) {
              // ** apply ODrive parameters to make joints stiffer **
              modifyGains(); 
              modeConfirmFlag = 1;
            }

            else if (modeConfirm == 6 && modeConfirmFlag == 0 && mydata_remote.Select == 1) {
              // ** test mode **
              runMode = 1; 
              modeConfirmFlag = 1;
            }

            else if (modeConfirm == 7 && modeConfirmFlag == 0 && mydata_remote.Select == 1) {
              // ** kinemtaics mode **
              runMode = 2; 
              modeConfirmFlag = 1;
            }

            else if (modeConfirm == 8 && modeConfirmFlag == 0 && mydata_remote.Select == 1) {
              // ** kinemtaics mode **
              runMode = 3; 
              modeConfirmFlag = 1;
            }

            // ***** start of runModes ******

            if (runMode == 1) {
                // ** simple joint motion test mode **
  
                // scale sticks to drive joints directly for this test mode
                RT = RT * 10;
                RFB = RFB * 10;
                RLR = RLR * 10;
  
                // filter sticks a reasonable amount for this test mode
                RFBFiltered = filter(RFB, RFBFiltered, 20);
                RLRFiltered = filter(RLR, RLRFiltered, 20);
                RTFiltered = filter(RT, RTFiltered, 20);
                
                // drive joints all in the same direction as a test
                // hips
                driveJoints (10, RTFiltered);     // back left
                driveJoints (11, RTFiltered);     // back rght
                driveJoints (40, RTFiltered);     // front left
                driveJoints (41, RTFiltered);     // front right
                // shoulders
                driveJoints (20, RFBFiltered);    // back right
                driveJoints (30, RFBFiltered);    // back left
                driveJoints (50, RFBFiltered);    // front right
                driveJoints (60, RFBFiltered);    // front left
                // knees
                driveJoints (21, RLRFiltered);    // back right
                driveJoints (31, RLRFiltered);    // back left
                driveJoints (51, RLRFiltered);    // front right
                driveJoints (61, RLRFiltered);    // back left
            }

            else if (runMode == 2) {
                // ** inverse kinematics demo **  

                stepStartFlag = 0;   // reset flag for stepping setup

                if (filterFlag1 == 0) {   // make sure Z height filtered value is already set to mid position
                  RTFiltered = 282;
                  filterFlag1 = 1;
                }
                
                RFB = map(RFB,-462,462,-100,100);
                RFBFiltered = filter(RFB, RFBFiltered, 30);
                RLR = map(RLR,-462,462,-100,100);
                RLRFiltered = filter(RLR, RLRFiltered, 30);
                RT = map(RT,-462,462,182,382);
                RTFiltered = filter(RT, RTFiltered, 30);
                
                LFB = map(LFB,-462,462,-10,10);
                LFBFiltered = filter(LFB, LFBFiltered, 30);
                LLR = map(LLR,-462,462,-15,15);
                LLRFiltered = filter(LLR, LLRFiltered, 30);
                LT = map(LT,-462,462,-10,10);
                LTFiltered = filter(LT, LTFiltered, 30);
                            
                kinematics (1, RFBFiltered, RLRFiltered-50, RTFiltered, LLRFiltered, LFBFiltered, LTFiltered);   // front right
                kinematics (2, RFBFiltered, RLRFiltered+50, RTFiltered, LLRFiltered, LFBFiltered, LTFiltered);   // front left
                kinematics (3, RFBFiltered, RLRFiltered+50, RTFiltered, LLRFiltered, LFBFiltered, LTFiltered);   // back left
                kinematics (4, RFBFiltered, RLRFiltered-50, RTFiltered, LLRFiltered, LFBFiltered, LTFiltered);   // back right
                  
             }

             else if (runMode == 3) {
                // simple walking

                // position legs a bit straighter, only do it once
                if (stepStartFlag == 0) {
                    modifyPos2();
                stepStartFlag = 1;
                }

                Serial.println(RFB);

                if (mydata_remote.toggle1 == 1) {

                    RFB = map(RFB,-462,462,-50,50);
                    RLR = map(RLR,-462,462,-25,25);
                    LT = map(LT,-462,462,-5,5);
                    longLeg = 350;
                    shortLeg = 280;  
                
                    int timer1 = 100;   // feet ON the ground
                    int timer2 = 100;   // feet OFF the ground
        
                    if (stepFlag == 0 && currentMillis - previousStepMillis > timer1) {
                        modifyPos1();
                        stepFlag = 1;              
                        previousStepMillis = currentMillis;
                    }
        
                    else if (stepFlag == 1 && currentMillis - previousStepMillis > timer2) {
                        modifyPos2();
                        stepFlag = 2;              
                        previousStepMillis = currentMillis;
                    }
        
                    else if (stepFlag == 2 && currentMillis - previousStepMillis > timer1) {
                        modifyPos3();
                        stepFlag = 3;              
                        previousStepMillis = currentMillis;
                    }
        
                    else if (stepFlag == 3 && currentMillis - previousStepMillis > timer2) {
                        modifyPos4();
                        stepFlag = 0;              
                        previousStepMillis = currentMillis;
                    }              
                }
                else {
                    modifyPos2();
                }
                  
             }
             
 
      
        }     // end of timed loop         
   
}       // end  of main loop


void modifyPos1() {
    // the other diag legs bent 
    kinematics (1, 0-RFB, -50-RLR, shortLeg, 0, 0, 0-LT);   // front right
    kinematics (2, RFB, 50+RLR, longLeg, 0, 0, LT);         // front left
    kinematics (3, 0-RFB, 50-RLR, shortLeg, 0, 0, 0-LT);   // back left
    kinematics (4, RFB, -50+RLR, longLeg, 0, 0, LT);   // back right
}

void modifyPos2() {
    // all legs long
    kinematics (1, 0-RFB, -50-RLR, longLeg, 0, 0, 0-LT);   // front right
    kinematics (2, RFB, 50+RFB, longLeg, 0, 0, LT);   // front left
    kinematics (3, 0-RFB, 50-RLR, longLeg, 0, 0, 0-LT);   // back left
    kinematics (4, RFB, -50+RFB, longLeg, 0, 0, LT);   // back right
}

void modifyPos3() {
    // diag legs bent
    kinematics (1, RFB, -50+RLR, longLeg, 0, 0, LT);   // front right
    kinematics (2, 0-RFB, 50-RLR, shortLeg, 0, 0, 0-LT);   // front left
    kinematics (3, RFB, 50+RLR, longLeg, 0, 0, LT);   // back left
    kinematics (4, 0-RFB, -50-RLR, shortLeg, 0, 0, -LT);   // back right
}

void modifyPos4() {
    // all legs long
    kinematics (1, RFB, -50+RLR, longLeg, 0, 0, LT);   // front right
    kinematics (2, 0-RFB, 50-RLR, longLeg, 0, 0, 0-LT);   // front left
    kinematics (3, RFB, 50+RLR, longLeg, 0, 0, LT);   // back left
    kinematics (4, 0-RFB, -50-RLR, longLeg, 0, 0, 0-LT);   // back right
}








