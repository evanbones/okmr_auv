#include <Servo.h>
const int minThrottle = 1100; // Minimum throttle in microseconds (1ms)
const int maxThrottle = 1900; // Maximum throttle in microseconds (2ms)
float throttle[8] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};

int pins[] = {2, 3, 4, 5, 6, 7, 8, 9};
int killSwitchPin = 13;
int ledPin1 = 10;
int ledPin2 = 11;
int ledPin3 = 12;
int leakSensorPin = 8;
int frequency = 40;
bool killSwitchEnabled = false;
unsigned long lastKillswitchPrintTime = 0;
const unsigned long KILLSWITCH_PRINT_INTERVAL = 100; // Print killswitch status every 100ms
int lastKillswitchState = -1; // Track previous state for change detection

Servo motors[8];

void killSwitch() {
  int switchState = digitalRead(killSwitchPin);
  killSwitchEnabled = (switchState == HIGH);

  if (killSwitchEnabled) {
    // Set all throttle values to neutral when killswitch is enabled
    for (int i = 0; i < 8; i++) {
      throttle[i] = 1500;
    }
    digitalWrite(ledPin1, LOW);  // Turn off LED when killswitch is enabled
  } else {
    digitalWrite(ledPin1, HIGH); // Turn on LED when killswitch is not enabled
  }

  // Print immediately if state changed, otherwise check time throttling
  bool stateChanged = (switchState != lastKillswitchState);
  bool timeToSend = (millis() - lastKillswitchPrintTime >= KILLSWITCH_PRINT_INTERVAL);
  
  if (stateChanged || timeToSend) {
    Serial.print("killswitch<");
    Serial.print(switchState);
    Serial.println();
    lastKillswitchPrintTime = millis();
    lastKillswitchState = switchState;
  }
}

void escArm() {
  delay(500);
  for (int i = 0; i < 8; i++) {
    motors[i].writeMicroseconds(1500);
  }
  delay(2000);
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
  //pinMode(leakSensorPin, INPUT);
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

unsigned long lastThrottleTime = 0;
const unsigned long THROTTLE_TIMEOUT = 2000;  // 2 second timeout
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
  /*
  int leakReading = analogRead(leakSensorPin);
  Serial.print("leak<");
  Serial.print(leakReading);
  Serial.println();
  */
  recvWithEndMarker();
  if(newData)parseNewData();

  // Check for throttle timeout
  if (millis() - lastThrottleTime > THROTTLE_TIMEOUT) {
    //Serial.println("RESET");
    for (int i = 0; i < 8; i++) {
      throttle[i] = 1500;
    }
  }

  for (int i = 0; i < 8; i++) {
    motors[i].writeMicroseconds((int)throttle[i]);
  }
}
