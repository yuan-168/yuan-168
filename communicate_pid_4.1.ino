#include <Encoder.h>
#define PPR 82.77
#define GR 100

// Define pin constants
const int encoder1PinA = 2;
const int encoder1PinB = 12;

const int encoder2PinA = 3;
const int encoder2PinB = 13;

const int motor1Pin1 = 4;
const int motor1Pin2 = 5;
const int enablePin1 = 6;

const int motor2Pin1 = 10;
const int motor2Pin2 = 11;
const int enablePin2 = 9;

const int sensorPin = A1;
const int RS = 10;
const int RL = 10000;

// Encoder objects
Encoder myEncoder1(encoder1PinA, encoder1PinB);
Encoder myEncoder2(encoder2PinA, encoder2PinB);

// Sensor parameters
int sensorValue;
float current, voltage;
float baseCurrent = 0.1;
float thresholdCurrent = 0.0;

// PID parameters
float kp1 = 5.0, ki1 = 0.005, kd1 = 35.0;
float kp2 = 15.0, ki2 = 0.0001, kd2 = 3.0;

// Target angles
float setpoint1 = 0.0;
float setpoint2 = 0.0; 

// PID variables for motor 1
double degree1, dTime1, error1, P1, I1, D1;
double prevTime1 = 0;
double prevError1 = 0;
int PWM1;
// PID variables for motor 2
double degree2, dTime2, error2, P2, I2, D2;
double prevTime2 = 0;
double prevError2 = 0;
int PWM2;

int i = 0;                // counter
String matlabStr = " ";    // receives the string from matlab, it is empty at first
bool readyToSend = false; // flag to indicate a command was received and now ready to send back to matlab

char c;                   // characters received from matlab
String val1;         // input1 from matlab
String val2;         // input2 from matlab
const int angleThreshold = 5; // Angle threshold for considering it "close"

// ===========================================================================

void setup() {
  Serial.begin(115200);

  pinMode(encoder1PinA, INPUT);
  pinMode(encoder1PinB, INPUT);
  pinMode(encoder2PinA, INPUT);
  pinMode(encoder2PinB, INPUT);

  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(enablePin1, OUTPUT);

  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(enablePin2, OUTPUT);

  pinMode(sensorPin, INPUT);
}

// ===========================================================================

void loop() {
  // First wait to receive data from matlab to arduino
    if (readyToSend == false) {
    if (Serial.available()>0)       // is there anything received?
    {
      c = Serial.read();            // read characters
      matlabStr = matlabStr + c;    // append characters to string as these are received
      
      if (matlabStr.indexOf(";") != -1) // have we received a semi-colon (indicates end of command from matlab)?
      {
        readyToSend = true;         // then set flag to true since we have received the full command

        // parse incomming data, e.g. C40.0,3.5;
        int posComma1 = matlabStr.indexOf(',');                     // position of comma in string
        val1 = matlabStr.substring(2, posComma1);         // float from substring from character 1 to comma position
        int posEnd = matlabStr.indexOf(';'); 
        setpoint1 = val1.toFloat();                       // position of last character
        val2 = matlabStr.substring(posComma1+1, posEnd);  // float from substring from comma+1 to end-1
        setpoint2 = val2.toFloat();
        matlabStr = " ";
      }
    }
  }
   // Then, start sending data from Ardiono to matlab
  if (readyToSend)  // arduino has received command form matlab and now is ready to send
  {             

    // Read angles
    double degree1 = readAngle(myEncoder1);
    double degree2 = readAngle(myEncoder2);  

    // PID control for motor 1
    // Calculate time difference
    double currTime1 = millis();
    dTime1 = currTime1 - prevTime1;

    // PID components
    error1 = setpoint1 - degree1;
    P1 = error1;
    I1 += error1;
    D1 = (error1 - prevError1) / dTime1;
    
    // PID calculation
    PWM1 = (int)(kp1 * P1 + ki1 * I1 * dTime1  + kd1 * D1);

    // Bound PWM value
    PWM1 = constrain(PWM1, -100, 100);
    // Motor control
    if (PWM1 >= 0) {
      analogWrite(enablePin1, PWM1);
      analogWrite(motor1Pin1, 0);
      analogWrite(motor1Pin2, round(PWM1));
    } else {
      analogWrite(enablePin1, -PWM1);
      analogWrite(motor1Pin1, round(-PWM1));
      analogWrite(motor1Pin2, 0);
    }
    prevTime1 = currTime1;
    prevError1 = error1;

    // =========================================================================

    // PID control for motor 2
      
    double currTime2 = millis();
    dTime2 = currTime2 - prevTime2;

    // PID components
    error2 = setpoint2 - degree2;
    P2 = error2;
    I2 += error2;
    D2 = (error2 - prevError2) / dTime2;

    // PID calculation
    PWM2 = (int)(kp2 * P2 + ki2 * I2 * dTime2  + kd2 * D2);

    // Bound PWM value
   PWM2 = constrain(PWM2, -100, 100);
    // Motor control
    if (PWM2 >= 0) {
      analogWrite(enablePin2, PWM2);
      analogWrite(motor2Pin1, LOW);
      analogWrite(motor2Pin2, HIGH);
    } else {
      analogWrite(enablePin2, -PWM2);
      analogWrite(motor2Pin1, HIGH);
      analogWrite(motor2Pin2, LOW);
    }

    prevTime2 = currTime2;
    prevError2 = error2;
    readyToSend = false;
   
    sensorValue = analogRead(sensorPin); 
    voltage = sensorValue * (5.0 / 1023.0); // Arduino 5V
    // Is = (Vout * 1000) / (RS * RL)
    current = (voltage * 1000) / (RS * RL); 
    // =========================================================================
   if (abs(degree1 - setpoint1) < 10 || abs(degree2 - setpoint2) < 10) { 
   //send command to Matlab
    Serial.print("c");                    // command
    Serial.print(degree1);                      // series 1
    Serial.print(",");                    // delimiter
    Serial.print(degree2);                  // series 2
    Serial.write(13);                     // carriage return (CR)
    Serial.write(10);                     // new line (NL)
    }
     i += 1;
     
  }
  
}
// ===========================================================================

// Function to read encoder position and convert to degrees
double readAngle(Encoder &Encoder) {
  long positions = Encoder.read();
  double degrees = positions * 360.0 / (PPR * GR);
  if (degrees < -360.0){
    positions += ((PPR*GR) * (degrees / 360));
    Encoder.write(positions);
    degrees = ((positions * 360.0) / (PPR * GR));
  } else if (degrees > 360.0){
    positions -= ((PPR*GR) * (degrees / 360));
    Encoder.write(positions);
    degrees = ((positions * 360.0) / (PPR * GR));
  }
  return degrees;
}