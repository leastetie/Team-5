const int UnderVolt = 80;
const int MaxVelocity = 160;

// Define constants for pulses per revolution (PPR) and gear ratio (GR)
const float GR = 297.924;

// Variables for tracking encoder positions
volatile long counter_m1 = 0;
volatile long counter_m2 = 0;
int aLastState_m1;
int aLastState_m2;

// Pins for reading encoders of motor 1 and 2
const int encoderPinA_m1 = 2;
const int encoderPinB_m1 = 3;
const int encoderPinA_m2 = 12;
const int encoderPinB_m2 = 11;

// Pins for setting the direction of motor 1 and 2
const int motorPin1_m1 = 4;
const int motorPin2_m1 = 5;
const int motorPin1_m2 = 7;
const int motorPin2_m2 = 8;

// Pins for setting the speed of rotation (Enable pin) of motors 1 and 2
const int enablePin_m1 = 6;
const int enablePin_m2 = 9;

// Variables for encoder positions and desired positions
long currentPosition_m1 = 0;
long currentPosition_m2 = 0;
float demandPositionInDegrees_m1;
float demandPositionInDegrees_m2;
float currentPositionInDegrees_m1;
float currentPositionInDegrees_m2;

// Time parameters
unsigned long currentTime;
unsigned long previousTime = 0;
unsigned long deltaT;

// PID gains
float Kp_m1 = 400, Kd_m1 = 10000000, Ki_m1 = 0;
float Kp_m2 = 400, Kd_m2 = 10000000, Ki_m2 = 0;

// Error values
float errorPositionInDegrees_prev_m1 = 0, errorPositionInDegrees_sum_m1 = 0;
float errorPositionInDegrees_prev_m2 = 0, errorPositionInDegrees_sum_m2 = 0;

// Communication

char c;                   // characters received from matlab
float val1 = 0.0;         // input1 from matlab
float val2 = 0.0;         // input2 from matlab

int i = 0;                // counter
String matlabStr = "";
int LED_pin = LED_BUILTIN;


//states
bool readyToSend = false; 
int resting = 0;

bool firstInputReceived = false;
bool secondInputReceived = false;

bool firstInputExecuted = false;
bool secondInputExecuted = false;


void setup() {
  Serial.begin(115200);
  pinMode(LED_pin, OUTPUT);

  digitalWrite(LED_pin, LOW);

  //Initialize the pins using pinMode and attachInterrupt functions
  pinMode(motorPin1_m1, OUTPUT);
  pinMode(motorPin2_m1, OUTPUT);
  pinMode(enablePin_m1, OUTPUT);

  pinMode(motorPin1_m2, OUTPUT);
  pinMode(motorPin2_m2, OUTPUT);
  pinMode(enablePin_m2, OUTPUT);

  pinMode(encoderPinA_m1, INPUT_PULLUP);
  pinMode(encoderPinB_m1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPinA_m1), updateEncoder_m1, CHANGE);

  pinMode(encoderPinA_m2, INPUT_PULLUP);
  pinMode(encoderPinB_m2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPinA_m2), updateEncoder_m2, CHANGE);
  
  delay(100);

  previousTime = micros();
}

void loop() {

  //demandPositionInDegrees_m1 = 360;
  //demandPositionInDegrees_m2 = 70;

  if (readyToSend == false) {
    if (Serial.available()>0)       // is there anything received?
    {
      //digitalWrite(LED_pin, LOW);

      c = Serial.read();            // read characters
      matlabStr = matlabStr + c;    // append characters to string as these are received

      //if (firstInputReceived == false){
        //Serial.println("received first input");
        //Serial.println(matlabStr);
      //  firstInputReceived = true;
      //}

      //if ( (secondInputReceived == false) && (firstInputExecuted == true) ){
        //Serial.println("received second input");
        //Serial.println(matlabStr);
      //  secondInputReceived = true;
      //}
      
      if (matlabStr.indexOf(";") != -1) // have we received a semi-colon (indicates end of command from matlab)?
        {
        // parse incomming data, e.g. C40.0,3.5;
        int posComma1 = matlabStr.indexOf(",");                     // position of comma in string
        val1 = matlabStr.substring(1, posComma1).toFloat();         // float from substring from character 1 to comma position
        int posEnd = matlabStr.indexOf(";");                        // position of last character
        val2 = matlabStr.substring(posComma1+1, posEnd).toFloat();  // float from substring from comma+1 to end-1

        demandPositionInDegrees_m1 = val1;
        demandPositionInDegrees_m2 = val2;

        readyToSend = true;
        digitalWrite(LED_pin, HIGH);

        /*
        Serial.println("demanded position");
        Serial.println(matlabStr);
        Serial.println(val1);
        Serial.println(val2);
        */
        matlabStr = "";
      }
    }
  }

  currentPositionInDegrees_m1 = ((counter_m1 * 360) / (GR * 6));
  currentPositionInDegrees_m2 = ((counter_m2 * 360) / (GR * 6));

  currentTime = micros();
  deltaT = currentTime - previousTime;

  if (deltaT > 400) {

    //Compute error (P,I,D), and ensure that the previous error is updated
    float errorPositionInDegrees_m1 = currentPositionInDegrees_m1 - demandPositionInDegrees_m1; 
    float errorPositionInDegrees_diff_m1 = (errorPositionInDegrees_m1 - errorPositionInDegrees_prev_m1) / deltaT;
    errorPositionInDegrees_sum_m1 += errorPositionInDegrees_m1;
    errorPositionInDegrees_prev_m1 = errorPositionInDegrees_m1;

    float errorPositionInDegrees_m2 = currentPositionInDegrees_m2 - demandPositionInDegrees_m2; 
    float errorPositionInDegrees_diff_m2 = (errorPositionInDegrees_m2 - errorPositionInDegrees_prev_m2) / deltaT;
    errorPositionInDegrees_sum_m2 += errorPositionInDegrees_m2;
    errorPositionInDegrees_prev_m2 = errorPositionInDegrees_m2;

    //Compute the PID output
    float controllerOutput_m1 = errorPositionInDegrees_m1 * Kp_m1 + errorPositionInDegrees_diff_m1 * Kd_m1 + errorPositionInDegrees_sum_m1 * Ki_m1 * deltaT;
    controllerOutput_m1 = constrain(controllerOutput_m1, -255, 255);

    float controllerOutput_m2 = errorPositionInDegrees_m2 * Kp_m2 + errorPositionInDegrees_diff_m2 * Kd_m2 + errorPositionInDegrees_sum_m2 * Ki_m2 * deltaT;
    controllerOutput_m2 = constrain(controllerOutput_m2, -255, 255);

    //To stop undervoltage
    if ( (-UnderVolt < controllerOutput_m1) && (controllerOutput_m1 < UnderVolt) ) {
      controllerOutput_m1 = 0;
    }

    if ( (-UnderVolt < controllerOutput_m2) && (controllerOutput_m2 < UnderVolt) ) {
      controllerOutput_m2 = 0;
    }

    //To stop fast movement
    if  (MaxVelocity < controllerOutput_m1) {
      controllerOutput_m1 = MaxVelocity;
    }

    if  (MaxVelocity < controllerOutput_m2) {
      controllerOutput_m2 = MaxVelocity;
    }

    //Send voltage to motors
    if (controllerOutput_m1 > 0) {
      digitalWrite(motorPin1_m1, HIGH);
      digitalWrite(motorPin2_m1, LOW);
      analogWrite(enablePin_m1, controllerOutput_m1);
    } else {
      digitalWrite(motorPin1_m1, LOW);
      digitalWrite(motorPin2_m1, HIGH);
      analogWrite(enablePin_m1, -controllerOutput_m1);
    }

    if (controllerOutput_m2 > 0) {
      digitalWrite(motorPin1_m2, HIGH);
      digitalWrite(motorPin2_m2, LOW);
      analogWrite(enablePin_m2, controllerOutput_m2);
    } else {
      digitalWrite(motorPin1_m2, LOW);
      digitalWrite(motorPin2_m2, HIGH);
      analogWrite(enablePin_m2, -controllerOutput_m2);
    }

    if ( (controllerOutput_m1 == 0) && (controllerOutput_m2 == 0) && (readyToSend == true) ) {
      resting += 1;
      if  (resting > 10) {
        resting = 0;
        readyToSend = false;
        digitalWrite(LED_pin, LOW);
      }
    }

    previousTime = currentTime;

  }

  if (readyToSend)  // arduino has received command form matlab and now is ready to send
  {
    
    Serial.print("c");                    // command
    Serial.print(i);                      // series 1
    Serial.print(",");                    // delimiter
    Serial.print(currentPositionInDegrees_m1);
    Serial.print(",");
    Serial.println(currentPositionInDegrees_m2); // series 2
    i += 1;

  }

}

void updateEncoder_m1() {
  // Code to update counter_m1 based on the state of the encoder pins
  // Read current states of channels A and B
  int aState_m1 = digitalRead(encoderPinA_m1);
  int bState_m1 = digitalRead(encoderPinB_m1);


  // Check if the state of channel A has changed
  if (aState_m1 != aLastState_m1) {
    // Determine the direction of rotation by comparing A and B states
    if (aState_m1 != bState_m1) {
      counter_m1++;  // Clockwise rotation
    } else {
      counter_m1--;  // Counterclockwise rotation
    }
  }

  // Update the last known state of channel A
  aLastState_m1 = aState_m1;
}

void updateEncoder_m2() {
  // Code to update counter_m2 based on the state of the encoder pins
  // Read current states of channels A and B
  int aState_m2 = digitalRead(encoderPinA_m2);
  int bState_m2 = digitalRead(encoderPinB_m2);

  // Check if the state of channel A has changed
  if (aState_m2 != aLastState_m2) {
    // Determine the direction of rotation by comparing A and B states
    if (aState_m2 != bState_m2) {
      counter_m2++;  // Clockwise rotation
    } else {
      counter_m2--;  // Counterclockwise rotation
    }
  }

  // Update the last known state of channel A
  aLastState_m2 = aState_m2;
}
