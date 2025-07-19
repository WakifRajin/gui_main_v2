//  REFERENCE FOR BTS DRIVER PINS
/* software ---->
 A -> left
 B -> right

 A1 -> front
 A2 -> back

 B1 -> front
 B2 -> back
*/

// left
#define ARPWM 6
#define ALPWM 9
#define A1ENRF 7
#define A2ENBL 12

// right
#define BRPWM 3
#define BLPWM 5
//#define B1ENLF 10
#define B1ENLF A3  // <---------- changed
#define B2ENBR 2

// Global variables for serial communication
String inputString = "";         // To store incoming serial data
bool stringComplete = false;     // Flag to indicate when to process the input

/* FUNCTIONS FOR MOVEMENTS (NO NEED FOR MODIFICATION, SAME AS PREVIOUS YEAR)*/
void Stop() {
  // LF + LB
  analogWrite(A1ENRF, 255);
  analogWrite(A2ENBL, 255);
  digitalWrite(ARPWM, LOW);
  analogWrite(ALPWM, LOW);

  // RF + RB
  analogWrite(B1ENLF, 255);
  analogWrite(B2ENBR, 255);
  digitalWrite(BRPWM, LOW);
  analogWrite(BLPWM, LOW);
}

void turnLeft(int speed) {
  // LF + LB
  analogWrite(A1ENRF, 255);
  analogWrite(A2ENBL, 255);
  digitalWrite(ARPWM, LOW);
  analogWrite(ALPWM, speed);

  // RF + RB
  analogWrite(B1ENLF, 255);
  analogWrite(B2ENBR, 255);
  digitalWrite(BRPWM, LOW);
  analogWrite(BLPWM, speed);
}

void turnRight(int spd) {
  // LF + LB
  analogWrite(A1ENRF, 255);
  analogWrite(A2ENBL, 255);
  digitalWrite(ALPWM, LOW);
  analogWrite(ARPWM, spd);

  // RF + RB
  analogWrite(B1ENLF, 255);
  analogWrite(B2ENBR, 255);
  digitalWrite(BLPWM, LOW);
  analogWrite(BRPWM, spd);
}

void moveBackward(int spd) { // turnRight
  // LF + LB
  analogWrite(A1ENRF, 255);
  analogWrite(A2ENBL, 255);
  analogWrite(ALPWM, spd);
  digitalWrite(ARPWM, LOW);

  // RF + RB
  analogWrite(B1ENLF, 255);
  analogWrite(B2ENBR, 255);
  digitalWrite(BLPWM, LOW);
  analogWrite(BRPWM, spd);
}

void moveForward(int spd) {  /// turnLeft
  // LF + LB
  analogWrite(A1ENRF, 255);
  analogWrite(A2ENBL, 255);
  digitalWrite(ALPWM, LOW);
  analogWrite(ARPWM, spd);

  // RF + RB
  analogWrite(B1ENLF, 255);
  analogWrite(B2ENBR, 255);
  analogWrite(BLPWM, spd);
  digitalWrite(BRPWM, LOW);
}

/* TASKS NEED TO BE DONE */
void velCallback(float x, float z) {
  int spd = (abs(x) + abs(z)) * 255;
  if (spd > 255) spd = 255;

  Serial.print("Processing command - x: ");  // Debug
  Serial.print(x);                           // Debug
  Serial.print(" z: ");                      // Debug
  Serial.println(z);                         // Debug

  // Determine movement
  if (x > 0 && z == 0) {
    Serial.print("Moving forward @ speed: ");  // Debug
    Serial.println(spd);
    moveForward(spd);
  } else if (x < 0 && z == 0) {
    Serial.print("Moving backward @ speed: ");  // Debug
    Serial.println(spd);
    moveBackward(spd);
  } else if (z > 0 && x == 0) {
    Serial.print("Turning left @ speed: ");  // Debug
    Serial.println(spd);
    turnLeft(spd);
  } else if (z < 0 && x == 0) {
    Serial.print("Turning right @ speed: ");  // Debug
    Serial.println(spd);
    turnRight(spd);
  } else if (x == 0 && z == 0) {
    Serial.println("Stopping");  // Debug
    Stop();
  } else {
    Serial.println("Combined movement not supported - stopping");  // Debug
    Stop();
  }
}

void setup() {
  // Set motor connections as outputs
  pinMode(ARPWM, OUTPUT);
  pinMode(ALPWM, OUTPUT);
  pinMode(BRPWM, OUTPUT);
  pinMode(BLPWM, OUTPUT);
  pinMode(A1ENRF, OUTPUT);
  pinMode(A2ENBL, OUTPUT);
  pinMode(B1ENLF, OUTPUT);
  pinMode(B2ENBR, OUTPUT);

  // Initialize serial communication
  Serial.begin(9600);

  Serial.println("Arduino controller initialized");  // Debug

  inputString.reserve(64); // Reserve buffer for incoming serial data
}

void loop() {
  while (Serial.available() > 0) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      stringComplete = true;
      Serial.print("Raw message received: ");  // Debug
      Serial.println(inputString);            // Debug
    } else {
      inputString += inChar;
    }
  }

  if (stringComplete) {
    float x = 0.0, z = 0.0;
    int parsed = sscanf(inputString.c_str(), "x %f z %f", &x, &z);
    
    if (parsed == 2) {
      Serial.println("Valid command received");  // Debug
      velCallback(x, z);
    } else {
      Serial.print("Invalid command format: ");  // Debug
      Serial.println(inputString);               // Debug
    }
    
    inputString = "";
    stringComplete = false;
  }
}

/*




*/
