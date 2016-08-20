//motordriver.cpp
//g++ motordriver.cpp -I ../include -std=c++11
// motor driver code class to control all 4 motors 

#include "motordriver.h"

#define MOTOR_PATH "/dev/i2c-10"

// Set to true to print some debug messages, or false to disable them.
#define ENABLE_DEBUG_OUTPUT false

// initialize motor driver with i2c address 
motordriver::motordriver(uint8_t addr) {
  _i2cAddr = addr;
}

//open communication to i2c device 
void motordriver::open_i2c(void){
  _i2cHandle = open(MOTOR_PATH,O_RDWR); 
  if(_i2cHandle < 0) {cout << "Failed to open port" << endl;}
}
//close communication to i2c device 
void motordriver::close_i2c(void){
  close(_i2cHandle);
}

// start i2c communication -- call this in main to start  
int motordriver::begin(void) {
  open_i2c(); // open device 
  ioctl(_i2cHandle, I2C_SLAVE,_i2cAddr); // set slave address
  reset();
  return 0;

}


void motordriver::reset(void) {
 write8(PCA9685_MODE1, 0x0);
}

void motordriver::setPWMFreq(float freq) {
  //Serial.print("Attempting to set freq ");
  //Serial.println(freq);
  freq *= 0.9;  // Correct for overshoot in the frequency setting (see issue #11).
  float prescaleval = 25000000;
  prescaleval /= 4096;
  prescaleval /= freq;
  prescaleval -= 1;
  if (ENABLE_DEBUG_OUTPUT) {
    //Serial.print("Estimated pre-scale: "); Serial.println(prescaleval);
  }
  uint8_t prescale = floor(prescaleval + 0.5);
  if (ENABLE_DEBUG_OUTPUT) {
    //Serial.print("Final pre-scale: "); Serial.println(prescale);
  }
  
  uint8_t oldmode = read8(PCA9685_MODE1);
  uint8_t newmode = (oldmode&0x7F) | 0x10; // sleep
  write8(PCA9685_MODE1, newmode); // go to sleep
  write8(PCA9685_PRESCALE, prescale); // set the prescaler
  write8(PCA9685_MODE1, oldmode);
  usleep(5000);
  write8(PCA9685_MODE1, oldmode | 0xa1);  //  This sets the MODE1 register to turn on auto increment.
                                          // This is why the beginTransmission below was not working.
  //  Serial.print("Mode now 0x"); Serial.println(read8(PCA9685_MODE1), HEX);
}

void motordriver::setPWM(uint8_t channel, uint16_t on, uint16_t off) {
  //Serial.print("Setting PWM "); Serial.print(num); Serial.print(": "); Serial.print(on); Serial.print("->"); Serial.println(off);

  //WIRE.beginTransmission(_i2caddr);
  //WIRE.write(LED0_ON_L+4*num);
  //WIRE.write(on);
  //WIRE.write(on>>8);
  //WIRE.write(off);
  //WIRE.write(off>>8);
  //WIRE.endTransmission();

  // begin the transmission


  //write
  uint8_t buf[5];
  buf[0] = LED0_ON_L + 4*channel;
  buf[1] = on;
  buf[2] = on >> 8;
  buf[3] = off;
  buf[4] = off >> 8;
  write(_i2cHandle,buf,5);
}

// Sets pin without having to deal with on/off tick placement and properly handles
// a zero value as completely off.  Optional invert parameter supports inverting
// the pulse for sinking to ground.  Val should be a value from 0 to 4095 inclusive.
void motordriver::setPin(uint8_t num, uint16_t val, bool invert)
{
  // Clamp value between 0 and 4095 inclusive.
  //val = min(val, 4095);
  if (invert) {
    if (val == 0) {
      // Special value for signal fully on.
      setPWM(num, 4096, 0);
    }
    else if (val == 4095) {
      // Special value for signal fully off.
      setPWM(num, 0, 4096);
    }
    else {
      setPWM(num, 0, 4095-val);
    }
  }
  else {
    if (val == 4095) {
      // Special value for signal fully on.
      setPWM(num, 4096, 0);
    }
    else if (val == 0) {
      // Special value for signal fully off.
      setPWM(num, 0, 4096);
    }
    else {
      setPWM(num, 0, val);
    }
  }
}

uint8_t motordriver::read8(uint8_t addr) {
  // WIRE.beginTransmission(_i2caddr);
  // WIRE.write(addr);
  // WIRE.endTransmission();

  // WIRE.requestFrom((uint8_t)_i2caddr, (uint8_t)1);
  // return WIRE.read();
  
  uint8_t buf[2];
  buf[1] = addr;
  if(write(_i2cHandle,buf,1) != 1){
    cout << "error when trying to set read register" << endl;
    
  }
  
  if(read(_i2cHandle,buf,1) != 1){
    cout << "failed to read from i2c bus" << endl;
  }

  return buf[1];
  
  
}

// write a byte to specified register addr, d is data 
void motordriver::write8(uint8_t addr, uint8_t d) {
  //  WIRE.beginTransmission(_i2caddr);
  // WIRE.write(addr);
  // WIRE.write(d);
  // WIRE.endTransmission();
  uint8_t data[2];
  data[0] = addr;
  data[1] = d;
  if(write(_i2cHandle,data,2)!= 2){
    cout << "error using write8" << endl;
  } else {
    cout << "success using write8" << endl;
  }
}

int main(void)
{
  float freq = 200.0;
  float period = 1/freq;
  float timePerTick = period / 4096;
  int motorsOff = (int)round(0.001080/timePerTick);
  int motorsFull = (int)round(0.001950/timePerTick);
  
  

  motordriver mdriver = motordriver(0x40);
  mdriver.begin();

  mdriver.setPWMFreq(200.0);
  mdriver.setPWM(4,0,motorsOff);
  mdriver.setPWM(1,0,motorsOff);
  mdriver.setPWM(2,0,motorsOff);
  mdriver.setPWM(3,0,motorsOff);
  usleep(1000000);
  int speed0;
  int speed1;
  int speed2;
  int speed3;
  cout << "enter speed for motor0" << endl;
  cin >> speed0;
  cout << "enter speed for motor1" << endl;
  cin >> speed1;
  cout << "enter speed for motor2" << endl;
  cin >> speed2;
  cout << "enter speed for motor3" << endl;
  cin >> speed3;
  mdriver.setPWM(1,0,speed1);
  mdriver.setPWM(4,0,speed0);
  mdriver.setPWM(2,0,speed2);
  mdriver.setPWM(3,0,speed3);
  cout << "Setting motor0 to  " << speed0 << "  " << endl;
  cout << "Setting motor1 to  " << speed1 << "  " << endl;
  cout << "Setting motor2 to  " << speed2 << "  " << endl;
  cout << "Setting motor3 to  " << speed3 << "  " << endl;
  while(1){
    cout << "enter speed for motor0" << endl;
    cin >> speed0;
    cout << "enter speed for motor1" << endl;
    cin >> speed1;
    cout << "enter speed for motor2" << endl;
    cin >> speed2;
    cout << "enter speed for motor3" << endl;
    cin >> speed3;
    mdriver.setPWM(1,0,speed1);
    mdriver.setPWM(4,0,speed0);
    mdriver.setPWM(2,0,speed2);
    mdriver.setPWM(3,0,speed3);
    cout << "Setting motor0 to  " << speed0 << "  " << endl;
    cout << "Setting motor1 to  " << speed1 << "  " << endl;
    cout << "Setting motor2 to  " << speed2 << "  " << endl;
    cout << "Setting motor3 to  " << speed3 << "  " << endl;
  }


}
