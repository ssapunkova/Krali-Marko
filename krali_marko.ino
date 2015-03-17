#include <ZumoMotors.h>
#include <Pushbutton.h>
#include <QTRSensors.h>
#include <ZumoReflectanceSensorArray.h>
#include <avr/pgmspace.h>
#include <Wire.h>
#include <LSM303.h>
#include <NewPing.h>

/* 
 * Copyright (c) 2013 Pololu Corporation.  For more information, see
 * 
 * http://www.pololu.com/
 * http://forum.pololu.com/
 * 
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 * 
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

#define LOG_SERIAL true // write log output to serial port

// ---  --- //
#define START_LED 13
Pushbutton start_button(ZUMO_BUTTON); // pushbutton on pin 12
// ---  --- //

// --- ACCELEROMETER --- //
#define ACCELEROMETER_RA_SIZE 3  // number of readings to include in running average of accelerometer readings
#define ACCELEROMETER_XY_ACCELERATION_THRESHOLD 2400  // for detection of contact (~16000 = magnitude of acceleration due to gravity)
// --- ACCELEROMETER --- //

// --- REFLECTANCE--- //
#define REFLECTANCE_NUM_SENSORS 6
unsigned int reflectance_sensor_values[REFLECTANCE_NUM_SENSORS];
// this might need to be tuned for different lighting conditions, surfaces, etc.
#define REFLECTANCE_QTR_THRESHOLD  1500 // microseconds
ZumoReflectanceSensorArray reflectance_sensors(QTR_NO_EMITTER_PIN); 
// --- REFLECTANCE--- //

// --- MOTOR SETTINGS --- //
ZumoMotors motors;

#define SPEED_REVERSE     200 
#define SPEED_TURN       200
#define SPEED_SEARCH      100
#define SPEED_FULL        400
#define SPEED_ATTACK      400

#define DURATION_STOP     100 // ms
#define DURATION_REVERSE  200 // ms
#define DURATION_TURN     300 // ms

#define DIRECTION_RIGHT 1
#define DIRECTION_LEFT -1
// --- MOTOR SETTINGS --- //

enum ForwardSpeed { 
  SearchSpeed, FullSpeed, AttackSpeed };
ForwardSpeed _forwardSpeed;
// --- STATE --- //
enum State { 
  Search, Attack, Defense, RunFromCorners };
State _state;
// --- STATE --- //

// --- TIMING --- //
unsigned long time_loop_start;
unsigned long time_last_turn;
unsigned long time_contact_made;

#define MIN_DELAY_AFTER_TURN          400  // ms = min delay before detecting contact event
#define MIN_DELAY_BETWEEN_CONTACTS   1000  // ms = min delay between detecting new contact event
// --- TIMING --- //

// --- SONAR --- //
#define SONAR_TRIGGER_PIN    2  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define SONAR_ECHO_PIN       11  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define SONAR_MAX_DISTANCE   64 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonar(SONAR_TRIGGER_PIN, SONAR_ECHO_PIN, SONAR_MAX_DISTANCE); // NewPing setup of pins and maximum distance.
unsigned int sonar_ping_speed = 50; // How frequently are we going to send out a ping (in milliseconds). 50ms would be 20 times a second.
unsigned long sonar_ping_timer;     // Holds the next ping time.
// --- SONAR --- //

// RunningAverage class 
// based on RunningAverage library for Arduino
// source:  http://playground.arduino.cc/Main/RunningAverage
template <typename T> 
class RunningAverage
{  
public:
  RunningAverage(void);
  RunningAverage(int);
  ~RunningAverage();
  void clear();
  void addValue(T);
  T getAverage() const;
  void fillValue(T, int);
protected:
  int _size;
  int _cnt;
  int _idx;
  T _sum;
  T * _ar;
  static T zero;
};

// Accelerometer Class -- extends the LSM303 Library to support reading and averaging the x-y acceleration 
//   vectors from the onboard LSM303DLHC accelerometer/magnetometer
class Accelerometer : 
public LSM303
{
  typedef struct acc_data_xy
  {
    unsigned long timestamp;
    int x;
    int y;
    float dir;
  } 
  acc_data_xy;

public: 
  Accelerometer() : 
  ra_x(ACCELEROMETER_RA_SIZE), ra_y(ACCELEROMETER_RA_SIZE) {
  };
  ~Accelerometer() {
  };
  void enable(void);
  void getLogHeader(void);
  void readAcceleration(unsigned long timestamp);
  float len_xy() const;
  float dir_xy() const;
  int x_avg(void) const;
  int y_avg(void) const;
  long ss_xy_avg(void) const;
  float dir_xy_avg(void) const;
private:
  acc_data_xy last;
  RunningAverage<int> ra_x;
  RunningAverage<int> ra_y;   
};

Accelerometer lsm303;
boolean in_contact;  // set when accelerometer detects contact with opposing robot

// forward declaration
void setForwardSpeed(ForwardSpeed speed);
void set_state(State state);

void setup()
{  
  // Initiate the Wire library and join the I2C bus as a master
  Wire.begin();

  // Initiate LSM303
  lsm303.init();
  lsm303.enable();

#ifdef LOG_SERIAL
  Serial.begin(9600);
  lsm303.getLogHeader();
#endif

  randomSeed((unsigned int) millis());

  pinMode(START_LED, HIGH);
  waitForButtonAndCountDown(false);
}

void waitForButtonAndCountDown(bool restarting)
{ 
#ifdef LOG_SERIAL
  Serial.print(restarting ? "Restarting Countdown" : "Starting Countdown");
  Serial.println();
#endif

  digitalWrite(START_LED, HIGH);
  start_button.waitForButton();
  digitalWrite(START_LED, LOW);

  delay(1000);

  // reset loop variables
  in_contact = false;  // 1 if contact made; 0 if no contact or contact lost
  time_contact_made = 0;
  time_last_turn = millis();  // prevents false contact detection on initial acceleration
  _forwardSpeed = SearchSpeed;
  // SONAR
  sonar_ping_timer = millis();
}

void loop()
{
  if (start_button.isPressed())
  {
    // if button is pressed, stop and wait for another press to go again
    motors.setSpeeds(0, 0);
    start_button.waitForRelease();
    waitForButtonAndCountDown(true);
  }

  time_loop_start = millis();
  lsm303.readAcceleration(time_loop_start); 
  reflectance_sensors.read(reflectance_sensor_values);

  if (reflectance_sensor_values[0] < REFLECTANCE_QTR_THRESHOLD) // RUN AWAY FROM THE BORDERS!
  {
    turn(DIRECTION_RIGHT, true);
    set_state(RunFromCorners);
  }
  else if (reflectance_sensor_values[5] < REFLECTANCE_QTR_THRESHOLD) // RUN AWAY FROM THE BORDERS!
  {
    turn(DIRECTION_LEFT, true);
    set_state(RunFromCorners);
  }
  else if (check_for_contact()) // CHECK IF BEING HIT OR HITTING
  { 
    on_contact_made(); 
    set_state(Defense);
  }
  else if (scan_for_enemy()) // CHECK FOR ENEMY IN FRONT
  { 
    on_attack(); // ATTACK!
    set_state(Attack);
  } 
  else
  {
    look_around(); // LOOK AROUND
    set_state(Search);
  }  
}

void on_attack()
{
#ifdef LOG_SERIAL
  Serial.print("attack");
  Serial.println();
#endif

  setForwardSpeed(AttackSpeed);
  int speed = getForwardSpeed();
  motors.setSpeeds(speed, speed);
  set_state(Attack);
}

boolean scan_for_enemy(){
  if (millis() >= sonar_ping_timer) {   // pingSpeed milliseconds since last ping, do another ping.
    sonar_ping_timer += sonar_ping_speed;      // Set the next ping time.
    sonar.ping();
    if (sonar.check_timer()) { // This is how you check to see if the ping was received.
      unsigned int distance = sonar.ping_result / US_ROUNDTRIP_CM;
#ifdef LOG_SERIAL
      Serial.print("Ping: ");
      Serial.print(distance); // Ping returned, uS result in ping_result, convert to cm with US_ROUNDTRIP_CM.
      Serial.println("cm");
#endif
      // decide if robot is detected
      return _is_object_detected(distance);
    }
  }
}

boolean _is_object_detected(unsigned int distance){
#ifdef LOG_SERIAL
  Serial.print("Object detected: ");
  Serial.print(SONAR_MAX_DISTANCE >= distance); // Ping returned, uS result in ping_result, convert to cm with US_ROUNDTRIP_CM.
  Serial.println();
#endif

  return (SONAR_MAX_DISTANCE >= distance);
} 
// SONAR

void look_around()
{
#ifdef LOG_SERIAL
  Serial.print("looking around ...");
  Serial.println();
#endif
  setForwardSpeed(SearchSpeed);
  motors.setSpeeds(SPEED_SEARCH * -1, -SPEED_SEARCH * -1);
  delay(50);
}

// execute turn 
// direction:  RIGHT or LEFT
// randomize: to improve searching
void turn(char direction, bool randomize)
{
#ifdef LOG_SERIAL
  Serial.print("turning ...");
  Serial.println();
#endif

  // assume contact lost
  on_contact_lost();

  static unsigned int duration_increment = DURATION_TURN / 4;
  motors.setSpeeds(-SPEED_REVERSE, -SPEED_REVERSE);
  delay(DURATION_REVERSE);
  motors.setSpeeds(SPEED_TURN * direction, -SPEED_TURN * direction);
  delay(randomize ? DURATION_TURN + (random(8) - 2) * duration_increment : DURATION_TURN);
  int speed = getForwardSpeed();
  motors.setSpeeds(speed, speed);
  time_last_turn = millis();
}

void setForwardSpeed(ForwardSpeed speed)
{
  _forwardSpeed = speed;
}

int getForwardSpeed()
{
  int speed;
  switch (_forwardSpeed)
  {
  case FullSpeed:
    speed = SPEED_FULL;
    break;
  case AttackSpeed:
    speed = SPEED_ATTACK;
    break;
  default:
    speed = SPEED_SEARCH;
    break;
  }
  return speed;
}

void set_state(State state){

}

// check for contact, but ignore readings immediately after turning or losing contact
boolean check_for_contact()
{
  static long threshold_squared = (long) ACCELEROMETER_XY_ACCELERATION_THRESHOLD * (long) ACCELEROMETER_XY_ACCELERATION_THRESHOLD;
  return (lsm303.ss_xy_avg() >  threshold_squared) && \
    (time_loop_start - time_last_turn > MIN_DELAY_AFTER_TURN) && \
    (time_loop_start - time_contact_made > MIN_DELAY_BETWEEN_CONTACTS);
}

// accelerate on contact -- fight or flight
void on_contact_made()
{
#ifdef LOG_SERIAL
  Serial.print("contact made");
  Serial.println();
#endif
  in_contact = true;
  time_contact_made = time_loop_start;
  setForwardSpeed(FullSpeed);
}

// reset forward speed
void on_contact_lost()
{
#ifdef LOG_SERIAL
  Serial.print("contact lost");
  Serial.println();
#endif
  in_contact = false;
  setForwardSpeed(SearchSpeed);
}

// class Accelerometer -- member function definitions

// enable accelerometer only
// to enable both accelerometer and magnetometer, call enableDefault() instead
void Accelerometer::enable(void)
{
  // Enable Accelerometer
  // 0x27 = 0b00100111
  // Normal power mode, all axes enabled
  writeAccReg(LSM303::CTRL_REG1_A, 0x27);

  if (getDeviceType() == LSM303::device_DLHC)
    writeAccReg(LSM303::CTRL_REG4_A, 0x08); // DLHC: enable high resolution mode
}

void Accelerometer::getLogHeader(void)
{
  Serial.print("millis    x      y     len     dir  | len_avg  dir_avg  |  avg_len");
  Serial.println();
}

void Accelerometer::readAcceleration(unsigned long timestamp)
{
  readAcc();
  if (a.x == last.x && a.y == last.y) return;

  last.timestamp = timestamp;
  last.x = a.x;
  last.y = a.y;

  ra_x.addValue(last.x);
  ra_y.addValue(last.y);

#ifdef LOG_SERIAL
  Serial.print(last.timestamp);
  Serial.print("  ");
  Serial.print(last.x);
  Serial.print("  ");
  Serial.print(last.y);
  Serial.print("  ");
  Serial.print(len_xy());
  Serial.print("  ");
  Serial.print(dir_xy());
  Serial.print("  |  ");
  Serial.print(sqrt(static_cast<float>(ss_xy_avg())));
  Serial.print("  ");
  Serial.print(dir_xy_avg());
  Serial.println();
#endif
}

float Accelerometer::len_xy() const
{
  return sqrt(last.x*a.x + last.y*a.y);
}

float Accelerometer::dir_xy() const
{
  return atan2(last.x, last.y) * 180.0 / M_PI;
}

int Accelerometer::x_avg(void) const
{
  return ra_x.getAverage();
}

int Accelerometer::y_avg(void) const
{
  return ra_y.getAverage();
}

long Accelerometer::ss_xy_avg(void) const
{
  long x_avg_long = static_cast<long>(x_avg());
  long y_avg_long = static_cast<long>(y_avg()); 
  return x_avg_long*x_avg_long + y_avg_long*y_avg_long;
}

float Accelerometer::dir_xy_avg(void) const
{
  return atan2(static_cast<float>(x_avg()), static_cast<float>(y_avg())) * 180.0 / M_PI;
}



// RunningAverage class 
// based on RunningAverage library for Arduino
// source:  http://playground.arduino.cc/Main/RunningAverage
// author:  Rob.Tillart@gmail.com
// Released to the public domain

template <typename T>
T RunningAverage<T>::zero = static_cast<T>(0);

template <typename T>
RunningAverage<T>::RunningAverage(int n)
{
  _size = n;
  _ar = (T*) malloc(_size * sizeof(T));
  clear();
}

template <typename T>
RunningAverage<T>::~RunningAverage()
{
  free(_ar);
}

// resets all counters
template <typename T>
void RunningAverage<T>::clear() 
{ 
  _cnt = 0;
  _idx = 0;
  _sum = zero;
  for (int i = 0; i< _size; i++) _ar[i] = zero;  // needed to keep addValue simple
}

// adds a new value to the data-set
template <typename T>
void RunningAverage<T>::addValue(T f)
{
  _sum -= _ar[_idx];
  _ar[_idx] = f;
  _sum += _ar[_idx];
  _idx++;
  if (_idx == _size) _idx = 0;  // faster than %
  if (_cnt < _size) _cnt++;
}

// returns the average of the data-set added so far
template <typename T>
T RunningAverage<T>::getAverage() const
{
  if (_cnt == 0) return zero; // NaN ?  math.h
  return _sum / _cnt;
}

// fill the average with a value
// the param number determines how often value is added (weight)
// number should preferably be between 1 and size
template <typename T>
void RunningAverage<T>::fillValue(T value, int number)
{
  clear();
  for (int i = 0; i < number; i++) 
  {
    addValue(value);
  }
}




