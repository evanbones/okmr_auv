#include <Servo.h>
const int minThrottle = 1100; // Minimum throttle in microseconds (1ms)
const int maxThrottle = 1900; // Maximum throttle in microseconds (2ms)

int pins[] = {2, 3, 4, 5, 6, 7, 8, 9};
int killSwitchPin = 22;
int ledPin1 = 25;
int ledPin2 = 25;
int ledPin3 = 25;
int leakSensorPin = 23;
int frequency = 200;
bool killSwitchEnabled = false;

Servo motors[8];

void killSwitch() {
  int switchState = digitalRead(killSwitchPin);
  killSwitchEnabled = (switchState == LOW);  
  
  if (killSwitchEnabled) {
    // Set all throttle values to neutral when killswitch is enabled
    for (int i = 0; i < 8; i++) {
      throttle[i] = 1500;
    }
    digitalWrite(ledPin1, LOW);  // Turn off LED when killswitch is enabled
  } else {
    digitalWrite(ledPin1, HIGH); // Turn on LED when killswitch is not enabled
  }
  
  Serial.print("killswitch<");
  Serial.print(switchState);
  Serial.println();
}

void escArm() {
  delay(500);
  for (int i = 0; i < 8; i++) {
    motors[i].writeMicroseconds(1500);
  }
  delay(3000);
  for (int i = 0; i < 8; i++) {
    motors[i].writeMicroseconds(1500);
  }
}
void setup() {
  // Initialize PWM for the ESC
  for (int i = 0; i < 8; i++) {
    motors[i] = Servo();
    motors[i].attach(pins[i]);
  }

  Serial.begin(115200);
  Serial.println("Arming ESC...");
  pinMode(killSwitchPin, INPUT_PULLUP);
  pinMode(leakSensorPin, INPUT);
  pinMode(ledPin1, OUTPUT);
  pinMode(ledPin2, OUTPUT);
  pinMode(ledPin3, OUTPUT);
  escArm();
  Serial.println("ESCs Armed.");
  while(Serial.available() > 0){
    Serial.read();
  }
  
}
const byte numChars = 32;
char receivedChars[numChars];   // an array to store the received data

boolean newData = false;
void recvWithEndMarker() {
    static byte ndx = 0;
    char endMarker = '\n';
    char rc;
    
    if (Serial.available() > 0) {
        rc = Serial.read();

        if (rc != endMarker) {
            receivedChars[ndx] = rc;
            ndx++;
            if (ndx >= numChars) {
                ndx = numChars - 1;
            }
        }
        else {
            receivedChars[ndx] = '\0'; // terminate the string
            ndx = 0;
            newData = true;
        }
    }
}

float throttle[8] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
unsigned long lastThrottleTime = 0;
const unsigned long THROTTLE_TIMEOUT = 1000;  // 1 second timeout
int currMotor = 0;
void parseNewData(){
    char * strtokIndx; // this is used by strtok() as an index

    strtokIndx = strtok(receivedChars,"<");      // get the first part - the string
    currMotor=atoi(strtokIndx);

    strtokIndx = strtok(NULL, "<");
    float value = atof(strtokIndx);     // convert this part to a float
    // Clamp throttle values to safe range
    throttle[currMotor] = constrain(value, minThrottle, maxThrottle);
    lastThrottleTime = millis();  // Update last received time
    newData = false;
}

void loop() {
  // Check for serial input
  killSwitch();
  int leakReading = analogRead(leakSensorPin);
  Serial.print("leak<");
  Serial.print(leakReading);
  Serial.println();
  recvWithEndMarker();
  if(newData)parseNewData();
  
  // Check for throttle timeout
  if (millis() - lastThrottleTime > THROTTLE_TIMEOUT) {
    for (int i = 0; i < 8; i++) {
      throttle[i] = 1500;
    }
  }
  
  for (int i = 0; i < 8; i++) {
    motors[i].writeMicroseconds((int)throttle[i]);
  }
  
  delay(1000 / frequency);
}
