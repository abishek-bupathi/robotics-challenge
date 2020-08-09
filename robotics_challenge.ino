// importing required packages
#include <OrangutanLCD.h>
#include <OrangutanMotors.h>
#include <Pololu3pi.h>
#include <PololuQTRSensors.h>
#include <OrangutanLEDs.h>
#include <OrangutanPushbuttons.h>
#include <OrangutanAnalog.h>
#include <OrangutanBuzzer.h>
#include <avr/pgmspace.h>

//initializing buffer size for filters
#define BUF_SIZE 10
#define filter_size 10

//initializing components
OrangutanLCD lcd;
OrangutanMotors motors;
Pololu3pi robot;
OrangutanBuzzer buzzer;



int count = 0;

//assigning pins
const int buttonA = 9,
          buttonB = 12,
          buttonC = 13,
          buzzer_pin = 10;
          
int LDR_LEFT_PORT = 6,
    LDR_RIGHT_PORT = 7;
    
unsigned int sensors[5];
    
// initializing variables for Light following taks
int leftMax = 0, rightMax = 0, 
    leftLDR = 0, rightLDR = 0;
int sensorValueL = 0, sensorValueR = 0;

//initializing motor speeds
int m1Speed = 100, m2Speed = 100,
    m3Speed = 30, m4Speed = -30,
     speed1 = 0;

// initializing variables for filters   
int buffer_smooth[BUF_SIZE];
int buffer[filter_size] = {};
int thisPoint;

// initializing variables for seesaw task
int smoothData = 0;
int flatDataSmooth = 0, flatDataMedian = 0;

/* 
   the command in the setup funtion are executed only once
   when the robot is powered on
*/
void setup() {
  
  //initializing pins for input and output
  pinMode(buttonA, INPUT);
  pinMode(buttonB, INPUT);
  pinMode(buttonC, INPUT);
  pinMode(buzzer_pin, OUTPUT);

  pololu_3pi_init_disable_emitter_pin(2000);

}

/* 
   commands in the loop function is executed infinite number
   of times and the code for the robot to perform the task goes here
*/
void loop() {
 // getting input through the 3 buttons
 
  while (digitalRead(buttonA) == HIGH && digitalRead(buttonB) == HIGH && digitalRead(buttonC) == HIGH) {
    lcd.clear();
    lcd.gotoXY(0, 0);
    lcd.print("-TEAM 5-");
    delay(100);
  }

  if (digitalRead(buttonA) == LOW) {
    // when button A is pressed only the ldr funtion is executed
    ldr();
    
  } else if (digitalRead(buttonB) == LOW) {
    // when button B is pressed line following task and seesaw is executed
    linecalibrate(); // calling the funtion to calibrate line
    delay(250);
    flatDataMedian = accSetupMedian(); //storing the flat surface value from accSetupMedian funciton
    flatDataSmooth = accSetupSmooth(); //storing the flat surface value from accSetupSmooth funciton
    linefollow(flatDataMedian); //calling linefollow funtion and passing flatDataMedian value
    delay(500);
    seesaw(flatDataSmooth); //executing seesaw balance task and passing flatDataSmooth value

  } else {
    // when button C is pressed all the task is executed with automated transitions
    ldr(); //executing light following task
    
    delay(500);
    OrangutanMotors::setSpeeds(-50, 50); //to rotate the robot to align with the line for better calibration
    delay(400);
    OrangutanMotors::setSpeeds(0, 0);
    delay(500);
    buzzersound(); //the buzzer beeps on transition

    linecalibrate(); //calibration of line

    delay(250);

    flatDataMedian = accSetupMedian(); //storing the flat surface value from accSetupMedian funciton
    flatDataSmooth = accSetupSmooth(); //storing the flat surface value from accSetupSmooth funciton

    delay(100);

    linefollow(flatDataMedian); //executing line follow task and passing flatDataMedian value
    buzzersound(); //the buzzer beeps on transition

    delay(500);
    seesaw(flatDataSmooth); //executing seesaw balance task and passing flatDataSmooth value

  }

  lcd.clear();
  lcd.gotoXY(0, 0);
  lcd.print("I'M DONE !"); //prints message when all the task is completed
  delay(5000);

}
/* 
 ldr funtion perfroms the light following task
*/
void ldr() {
  //printing message to indicate it is calibrating light
  lcd.clear();
  lcd.gotoXY(0, 0);
  lcd.print("FINDING");
  lcd.gotoXY(0, 1);
  lcd.print("LIGHT");

  for (int i = 0; i < 8000; i++) {

    motors.setSpeeds(30, -30); // robot rotates while ldr reads the values
    leftLDR = getLeftLDRCalibrated();
    rightLDR = getRightLDRCalibrated();

    if (leftLDR > leftMax) {  // if statements to store the maximum value read by the ldr
      leftMax = leftLDR;
    }

    if (rightLDR > rightMax) {
      rightMax = rightLDR;
    }

  }
  moveRobot(leftMax, rightMax); //moves the robot towards light

}
/* 
  getLeftLDRCalibrated funtion returns mapped value from the raw values got from left ldr
*/
int getLeftLDRCalibrated() {
  while (true) {
    leftLDR = analogRead(LDR_LEFT_PORT);
    sensorValueL = map(leftLDR, 3, 1023, 1000, 0); //mapping of values || 1023 - darkest, 3 - brightest
    return sensorValueL; //returns mapped value
  }
}

/* getRightLDRCalibrated funtion returns mapped value from the raw values got from right ldr
*/

int getRightLDRCalibrated() {
  while (true) {
    rightLDR = analogRead(LDR_RIGHT_PORT);
    sensorValueR = map(rightLDR, 1, 1020, 1000, 0); //mapping of values || 1020 - darkest, 1 - brightest
    return sensorValueR; //returns mapped value
  }
}

/*
 moveRobot functio moves the robot towards light while getting the reading from the ldr and comparing
*/

void moveRobot(int leftLDRMax, int rightLDRMax) {
  //printing message 
  lcd.clear();
  lcd.gotoXY(0, 0);
  lcd.print("TOWARDS");
  lcd.gotoXY(0, 1);
  lcd.print("LIGHT");
  
  //780 is the max  brightness when it is 5 cm away from the light source
  //the while loop executes till it is 5cm near the light
  
  while (leftLDR < 780 && rightLDR < 780) { 
    
    leftLDR = getLeftLDRCalibrated(); //getting left ldr values at each run of the loop
    rightLDR = getRightLDRCalibrated(); //getting right ldr values at each run of the loop
    
    // condition for the robot to move forward or turn
    if (leftLDR <= (leftLDRMax - leftLDRMax * 0.04) && rightLDR <= (rightLDRMax - rightLDRMax * 0.04)) 
    {  
      if (leftLDR > (rightLDR + rightLDR * 0.05)) { 
        //if left side is brighter than right side the robot rotates towards left
        motors.setSpeeds(m4Speed, m3Speed);
        delay(390);
      } else if (leftLDR < (rightLDR + rightLDR * 0.05)) { 
        //if right side is brighter than left side the robot rotates towards right
        motors.setSpeeds(m3Speed, m4Speed);
        delay(390);
      }

    } else {
      // the robot moves forward
      motors.setSpeeds(m1Speed, m2Speed);
      delay(100);
    }

  }
  //the robot stops after reaching the light
  motors.setSpeeds(0, 0);

}

// linecalibrate funtion calibrates the line

void linecalibrate() {

  unsigned int counter;

  lcd.clear();
  lcd.gotoXY(0, 0);
  lcd.print("FINDING");
  lcd.gotoXY(0, 1);
  lcd.print("LINE");

  for (counter = 0; counter < 80; counter++) {
    //robot rotates in both directions
    if (counter < 20 || counter >= 60)
      OrangutanMotors::setSpeeds(40, -40);
    else
      OrangutanMotors::setSpeeds(-40, 40);

    robot.calibrateLineSensors(IR_EMITTERS_ON); // enables IR emmited and calibrates

    delay(20);
  }

  OrangutanMotors::setSpeeds(0, 0); //robot stops calibrates 

}
/* 
   linefollow funtion track the line and moves the robot on the line
   and stops when it reaches the seesaw
*/
void linefollow(int flat_filtered_value) {
  //prints message
  lcd.clear();
  lcd.gotoXY(0, 0);
  lcd.print("FOLLOW");
  lcd.gotoXY(0, 1);
  lcd.print("LINE");

  int counter = 0, current_filtered_value[100] = {}, i; //initializing local variables
  //current_filtered_value array to store 100 consecutive accelerometer values
  /* the robot continues follow line till it reaches the seesaw by detecting
     change in accelerometer values for 10 consecutive runs of the loop  */
  while (counter < 10) 
  {

    unsigned int position = robot.readLine(sensors, IR_EMITTERS_ON);

    if (position < 1000) {
      //the line is in the left so the robot rotates left
      OrangutanMotors::setSpeeds(0, 40);
      OrangutanLEDs::left(HIGH);
      OrangutanLEDs::right(LOW);
    } else if (position < 3000) {
      //the line is in the right so the robot rotates right
      OrangutanMotors::setSpeeds(40, 40);
      OrangutanLEDs::left(HIGH);
      OrangutanLEDs::right(HIGH);
    } else {
      // the line is in the middle so the robot moves forward
      OrangutanMotors::setSpeeds(40, 0);
      OrangutanLEDs::left(LOW);
      OrangutanLEDs::right(HIGH);
    }

    current_filtered_value[i] = median_filter(analogRead(5)); // getting current accelerometer values and applying median filter to store in a array

    i++; // array indexing variables increase for each run of the loop

    if (i <= 100) //array size is 100 so if statement makes sure the value of indexing variable doesnt exceed 100
    {
      if (current_filtered_value[i] > flat_filtered_value + 1) //checks whether current accelerometer value is higher than flat surface value
      {
        counter++;
        //increases the counter by 1 which stores  consecutive number of times the current accelerometer value is higher than flat surface value
         
      } else 
      { //if the current accelerometer value is same as flat surface value counter is set to 0
        counter = 0;
      }

    } else {
      // once array indexing variable reaches 100 it is reset to 0
      i = 0;
    }

    delay(10);

  }
  OrangutanMotors::setSpeeds(0, 0); // robot stops after finishing the task
}

// seesaw function performs the balancing task by operating the motor based on accelerometer value

void seesaw(int flatData) {
  //prints message
  lcd.clear();
  lcd.gotoXY(0, 0);
  lcd.print("BALANCE!");
  lcd.gotoXY(0, 1);
  lcd.print("---------");
  //initailizng local variables
  int data = 0;
  int time = 0;

  while (time <= 8) // the loop is executed the robot is on balance for 8 consecutive runs
  {
    data = analogRead(5); //getting current accelerometer value
    smoothData = smooth(data); //applying smooth filter
    delay(20);

    if (smoothData > flatData) //condition to check upward tilt
    {
      speed1 = 25 - (smoothData - flatData) * 0.85;
      // the robot moves forward
      OrangutanMotors::setSpeeds(speed1, speed1);
      time = 0; //time counter is set to 0
      delay(15);
    } else if (smoothData < flatData) //checking for downward tilt
    {
      speed1 = 25 - (flatData - smoothData) * 0.85;
      // the robot moves backward
      OrangutanMotors::setSpeeds(-speed1, -speed1);
      time = 0; //time counter is set to 0
      delay(15);
    } else {
      //when robot is on balance motors stops
      OrangutanMotors::setSpeeds(0, 0);
      delay(200);
      time++; //time counter increases by 1
    }
  }

  OrangutanMotors::setSpeeds(0, 0); //robot stops after finishing the task

}

// accSetupMedian funtion returna accelerometer value on flat surface using median filter

int accSetupMedian() {
  int flat_value_median; //initializing local variable

  for (thisPoint = 0; thisPoint < 10; thisPoint++)
    buffer[thisPoint] = 0; // setting buffer values to 0

  int data_median = analogRead(5); //getting accelerometer reading

  flat_value_median = median_filter(data_median); //filtering the accelerometer reading using median filter

  return flat_value_median; //returns filtered value on flat surface

}

// accSetupSmooth funtion return accelerometer value on flat surface using smoothing filter

int accSetupSmooth() {
  int flat_value_smooth;

  for (thisPoint = 0; thisPoint < 10; thisPoint++)
    buffer[thisPoint] = 0; // setting buffer values to 0

  int data_smooth = analogRead(5); //getting accelerometer reading

  flat_value_smooth = smooth(data_smooth); //filtering the accelerometer reading using smoothing

  return flat_value_smooth; //returns filtered value on flat surface

}

//smooth function returns filtered value using smoothing technique
int smooth(int data) {
  int i, total;

  for (i = BUF_SIZE; i >= 0; i--) {
    buffer_smooth[i] = buffer_smooth[i - 1];
    buffer_smooth[0] = data;

    for (i = 0; i < BUF_SIZE; i++)
      total += buffer_smooth[i];

    return total / BUF_SIZE;
  }
}

//median_filter function returns filtered value using median filter algorithm

int median_filter(int value) {

  boolean swapped = true; // setting true
  int temp, i, j = 0;
  int median_buffer[filter_size] = {}; //temperory buffer for rearranging

  //shift buffer value left
  for (i = 0; i <= filter_size - 2; i++) //(filter_size-2)ensures that iteration will not exceed array size when moving value
  { 
    buffer[i] = buffer[i + 1]; // move value one place left 
  }

  buffer[filter_size - 1] = value; // save new value to rightmost position

  if (count < filter_size) // do nothing until buffer is filled
  {
    count++;
    return value;  //return the value inputted for now
  } else //once buffer is filled , rearrange buffer from lowest to highest
  {
    // copy contents of buffer into new temporary array called median_buffer
    for (i = 0; i < filter_size; i++)
    {
      median_buffer[i] = buffer[i];
    }

    while (swapped) //while iteration take place, swapped will return true, and while loop will continue
    {
      swapped = false; //when for loop no longer takes place, 'false' will result in exiting while loop
      j++;
      for (i = 0; i < filter_size - 1; i++)  // loop iterates to move lower value left in array to ensure median_buffer tends from lowest highest
      {
        if (median_buffer[i] > median_buffer[i + 1]) // only takes place if value to left larger
        {
          temp = median_buffer[i];
          median_buffer[i] = median_buffer[i + 1];
          median_buffer[i + 1] = temp;
          swapped = true;  // when true, while loop continues
        }

      }
      return median_buffer[filter_size / 2]; //this value we want (i.e, the median when values are set from low to high)
    }
  }
}

// buzzersound function activates the buzzer to play beep
void buzzersound() {
  buzzer.playNote(NOTE_A(5), 200, 15);
  delay(400);
}
