
#include "I2Cdev.h"
#include "MPU6050.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 accelgyro;

int16_t ax, ay, az;
double axActual, ayActual, azActual;
int16_t tmp; // in cellisious.
int16_t steps=0;
double threshold = 2.05;
double oldAccSum=0,newAccSum=0,valueCompare=0;
bool isStep=false;

#define G 2048  // 16384 for 2g 8192 for 4g , 4096 8g , 2048 16g , 1024  32g

int32_t offX=0;
int32_t offY=0;
int32_t offZ=0;
void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(38400);

    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
    
    // use the code below to change accel/gyro offset values
    
    Serial.println("Updating internal sensor offsets...");
    // offset:  -12149  32767 6232    
    Serial.println("After Setting offsets");
    accelgyro.setXAccelOffset(0);
    accelgyro.setYAccelOffset(0);
    accelgyro.setZAccelOffset(0);
    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print("\n");

    //Calibrate();
    
    offX=-12149;
    offY=32767;
    offZ=6232;
    
}

void loop() {
    // read raw accel/tmp measurements from device
    oldAccSum=0,newAccSum=0,valueCompare=0;

    for(int i=0;i<100;i++)
    {
      accelgyro.getAcceleration(&ax, &ay, &az);
      tmp = accelgyro.getTemperature();
  
      axActual=double(ax-offX)/double(G);
      ayActual=double(ay-offY)/double(G);
      azActual=double(az+(G-offZ))/double(G);
      // display tab-separated accel x/y/z values
      Serial.print("Acclerations :\t");
      Serial.print(axActual); Serial.print("\t");
      Serial.print(ayActual); Serial.print("\t");
      Serial.println(azActual); 
      Serial.print("Temperature :\t");
      Serial.println(tmp/340.00+36.53);
      delay(200);
      calculateSteps();
      
    }

    delay(1000);

}

void calculateSteps(){
 oldAccSum=newAccSum;
 newAccSum = sqrt((axActual* axActual)+ (azActual*azActual));  // + (ayActual*ayActual) neglect y
 valueCompare = (newAccSum + oldAccSum) / 2 ;
 Serial.print("valueCompare = ");
 Serial.println(valueCompare);
 if(valueCompare > threshold && isStep==false){
  steps+=1;
  isStep= true;
 }
 if(valueCompare < threshold && isStep==true){
  isStep= false;
 }
  Serial.print("Steps = ");
  Serial.println(steps);
}
void Calibrate()
{
  Serial.println("Inside saturation loop..");
  // This initial loop until the mpu saturates.
  for(int32_t i=0;i<100000;i++)
  {
    accelgyro.getAcceleration(&ax, &ay, &az);
    tmp = accelgyro.getTemperature();
    if(i%10000==0)
     Serial.println("Still inside..");
  }
  Serial.println("Start offsets calculation..");
  // Now start offsets calculation.
  offX=0;
  offY=0;
  offZ=0;
  int noOfValues = 1000;
  for(int i=0;i<noOfValues;i++)
  {
    accelgyro.getAcceleration(&ax, &ay, &az);
    offX+=ax;
    offY+=ay;
    offZ+=az;
  }
  offX/=noOfValues;
  offY/=noOfValues;
  offZ/=noOfValues;
  Serial.print("offset:\t");
  Serial.print(offX); Serial.print("\t");
  Serial.print(offY); Serial.print("\t");
  Serial.println(offZ);
}

//    Serial.print("Acclerations :\t");
//    Serial.print(ax-offX); Serial.print("\t");
//    Serial.print(ay-offY); Serial.print("\t");
//    Serial.println(az+(G-offZ)); 
