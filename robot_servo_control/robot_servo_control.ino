/*
  =============================================================================
  RoboHead - 5-Servo + Active-LOW Relay Blower Controller Firmware
  =============================================================================
  Hardware Pin Assignments:
    - Pin 9  : Main Servo    (Tilt Up / Down)
    - Pin 3  : Mouth Servo   (Mouth Open / Close Sweep)
    - Pin 6  : Third Servo   (Pan Left / Right / Home)
    - Pin 2  : Pulse Servo 1 (Heartbeat Pulse SG90)
    - Pin 12 : Pulse Servo 2 (Heartbeat Pulse SG90)
    - Pin 4  : Relay Blower  (Active LOW Relay - Air Blower / Breath Motor)

  Baud Rate: 9600 Baud (Works with Web Serial UI & Serial Monitor)

  Serial Commands:
    - UP                    -> Main Servo tilts UP (50 deg)
    - DOWN                  -> Main Servo tilts DOWN (120 deg)
    - MOUTH ON              -> Mouth starts sweeping between 90-130 deg
    - MOUTH OFF             -> Mouth stops and holds angle
    - RIGHT                 -> Third Servo pans RIGHT (50 deg)
    - LEFT                  -> Third Servo pans LEFT (140 deg)
    - HOME / CENTER         -> Third Servo & Main Servo return to CENTER (90 deg)
    - PULSE ON              -> Two SG90 pulse servos run heartbeat rhythm
    - PULSE OFF             -> Two SG90 pulse servos stop
    - TILT <angle>          -> Direct positioning of Main Servo (0-180 deg)
    - PAN <angle>           -> Direct positioning of Third Servo (0-180 deg)
    - POS <pan> <tilt>      -> Direct simultaneous positioning of Pan & Tilt
    - 1 / BLOW ON / BLOW    -> Starts Relay Blinking Cycle (Active LOW, Pin 4 LOW = ON)
    - 0 / BLOW OFF          -> Stops Relay Blinking & turns Relay OFF (Pin 4 HIGH = OFF)
    - INTERVAL <sec>        -> Set Blow On/Off Cycle duration in seconds (e.g. INTERVAL 2, INTERVAL 3)
    - ESTOP                 -> Emergency stop all active loops & return home
  =============================================================================
*/

#if defined(ESP32)
  #include <ESP32Servo.h>
#else
  #include <Servo.h>
#endif

// SERVOS
Servo mainServo;
Servo mouthServo;
Servo thirdServo;
Servo pulseServo1;
Servo pulseServo2;

// =====================================
// RELAY (PIN 4 - ACTIVE LOW MODULE)
// =====================================
const int relayPin = 4;

// Active LOW relay module definition
const int RELAY_ON = LOW;
const int RELAY_OFF = HIGH;

// RELAY BLINK / CYCLE CONTROL
bool relayBlinkActive = false;
bool relayState = false;

unsigned long relayTimer = 0;
unsigned long relayInterval = 2000;   // Default 2000 ms (2.0s)
unsigned long relayOnTime = 2000;     // Configurable ON duration
unsigned long relayOffTime = 2000;    // Configurable OFF duration

// =====================================
// MAIN SERVO (Pin 9 - Tilt Up / Down)
// =====================================
int mainAngle = 90;
int mainTarget = 90;

const int MAIN_UP = 50;
const int MAIN_DOWN = 120;
const int MAIN_HOME = 90;

unsigned long mainTimer = 0;
const int mainSpeed = 15;

// =====================================
// MOUTH SERVO (Pin 3 - Mouth Actuator)
// =====================================
int mouthAngle = 90;
int mouthDirection = 1;

const int MOUTH_MIN = 90;
const int MOUTH_MAX = 130;

unsigned long mouthTimer = 0;
const int mouthSpeed = 15;

bool mouthActive = false;

// =====================================
// FOOD PIPE ACTUATOR
// =====================================
bool foodPipeOpen = false;

// =====================================
// THIRD SERVO (Pin 6 - Pan Left / Right)
// =====================================
int thirdAngle = 90;
int thirdTarget = 90;

const int THIRD_RIGHT = 50;
const int THIRD_LEFT = 140;
const int THIRD_HOME = 90;

unsigned long thirdTimer = 0;
const int thirdSpeed = 15;

// =====================================
// PULSE SERVOS (SG90 - Pin 2 & Pin 12)
// =====================================
int pulseAngle = 0;

const int PULSE_MIN = 0;
const int PULSE_MAX = 20;

// Adjustable pulse speed (1: HIGH, 20: MEDIUM, 35: LOW)
int heartbeatSpeed = 20;
const int holdTime = 100;
const int pauseTime = 300;

bool pulseActive = false;
int pulsePhase = 0;
unsigned long pulseTimer = 0;

// SERIAL COMMAND BUFFER
String command = "";

void setup() {
  Serial.begin(9600);

#if defined(ESP32)
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  mainServo.setPeriodHertz(50);
  mouthServo.setPeriodHertz(50);
  thirdServo.setPeriodHertz(50);
  pulseServo1.setPeriodHertz(50);
  pulseServo2.setPeriodHertz(50);
#endif

  // RELAY SETUP (Active LOW - Default OFF / HIGH)
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, RELAY_OFF);

  mainServo.attach(9);
  mouthServo.attach(3);
  thirdServo.attach(6);

  pulseServo1.attach(2);
  pulseServo2.attach(12);

  mainServo.write(90);
  mouthServo.write(90);
  thirdServo.write(90);

  pulseServo1.write(0);
  pulseServo2.write(0);

  Serial.println("ROBOT SERVO CONTROL READY");
  Serial.println("UP / DOWN");
  Serial.println("MOUTH ON / MOUTH OFF");
  Serial.println("RIGHT / LEFT / HOME");
  Serial.println("PULSE ON / PULSE OFF");
  Serial.println("PULSE SPEED 1 (HIGH) / 20 (MEDIUM) / 35 (LOW)");
  Serial.println("1 = RELAY BLINK ON");
  Serial.println("0 = RELAY OFF");
  Serial.println("INTERVAL <sec> = SET BLOW CYCLE SECONDS");
}

void loop() {
  // =====================================
  // SERIAL COMMAND READING
  // =====================================
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      command.trim();
      command.toUpperCase();

      if (command.length() == 0) {
        // Empty line, ignore
      }
      else if (command == "1" || command == "BLOW ON" || command == "BLOW" || command == "RELAY ON" || command == "BREATH ON" || command == "LUNGS ON" || command == "LUNGS") {
        relayBlinkActive = true;
        relayState = true;
        digitalWrite(relayPin, RELAY_ON);
        relayTimer = millis();

        mouthActive = true;
        pulseActive = true;
        pulsePhase = 0;
        pulseTimer = millis();

        Serial.println("RELAY BLINKING STARTED");
        Serial.println("LUNGS ON");
      }
      else if (command == "0" || command == "BLOW OFF" || command == "RELAY OFF" || command == "BREATH OFF" || command == "LUNGS OFF") {
        relayBlinkActive = false;
        relayState = false;
        digitalWrite(relayPin, RELAY_OFF);

        mouthActive = false;
        foodPipeOpen = false;
        pulseActive = false;
        mouthServo.write(mouthAngle);
        pulseServo1.write(0);
        pulseServo2.write(0);

        Serial.println("LUNGS STOPPED - OFF");
      }
      else if (command == "LUNGS STEP 1" || command == "STEP 1" || command == "LUNGS 1") {
        relayInterval = 1000;
        relayOnTime = 1000;
        relayOffTime = 1000;
        Serial.println("LUNGS STEP 1 SET (1.0 SEC)");
      }
      else if (command == "LUNGS STEP 2" || command == "STEP 2" || command == "LUNGS 2") {
        relayInterval = 2000;
        relayOnTime = 2000;
        relayOffTime = 2000;
        Serial.println("LUNGS STEP 2 SET (2.0 SEC)");
      }
      else if (command == "LUNGS STEP 3" || command == "STEP 3" || command == "LUNGS 3") {
        relayInterval = 3000;
        relayOnTime = 3000;
        relayOffTime = 3000;
        Serial.println("LUNGS STEP 3 SET (3.0 SEC)");
      }
      else if (command.startsWith("INTERVAL ") || command.startsWith("INTERVAL:") ||
               command.startsWith("TIME ") || command.startsWith("TIME:") ||
               command.startsWith("BLOW_TIME ") || command.startsWith("CYCLE ") ||
               command.startsWith("LUNGS STEP ") || command.startsWith("LUNGS_STEP ")) {
        int splitIdx = command.indexOf(' ');
        if (splitIdx < 0) splitIdx = command.indexOf(':');

        if (splitIdx > 0) {
          float secVal = command.substring(splitIdx + 1).toFloat();
          if (secVal > 0.05) {
            if (secVal <= 60.0) {
              relayInterval = (unsigned long)(secVal * 1000.0);
            } else {
              relayInterval = (unsigned long)secVal;
            }
            if (relayInterval < 100) relayInterval = 100; // 100ms safety limit
            relayOnTime = relayInterval;
            relayOffTime = relayInterval;

            Serial.print("RELAY INTERVAL SET: ");
            Serial.print(relayInterval / 1000.0, 1);
            Serial.println(" SEC");
          }
        }
      }
      else if (command.startsWith("ON_TIME ")) {
        float secVal = command.substring(8).toFloat();
        if (secVal > 0.05) {
          relayOnTime = (secVal <= 60.0) ? (unsigned long)(secVal * 1000.0) : (unsigned long)secVal;
          Serial.print("RELAY ON TIME SET: ");
          Serial.print(relayOnTime / 1000.0, 1);
          Serial.println(" SEC");
        }
      }
      else if (command.startsWith("OFF_TIME ")) {
        float secVal = command.substring(9).toFloat();
        if (secVal > 0.05) {
          relayOffTime = (secVal <= 60.0) ? (unsigned long)(secVal * 1000.0) : (unsigned long)secVal;
          Serial.print("RELAY OFF TIME SET: ");
          Serial.print(relayOffTime / 1000.0, 1);
          Serial.println(" SEC");
        }
      }
      else if (command == "UP") {
        mainTarget = MAIN_UP;
        Serial.println("MAIN SERVO UP - 50");
      }
      else if (command == "DOWN") {
        mainTarget = MAIN_DOWN;
        Serial.println("MAIN SERVO DOWN - 120");
      }
      else if (command == "MOUTH ON" || command == "MOUTH OPEN") {
        mouthActive = true;
        Serial.println("MOUTH ON");
      }
      else if (command == "MOUTH OFF" || command == "MOUTH CLOSE") {
        mouthActive = false;
        mouthServo.write(mouthAngle);
        Serial.println("MOUTH STOPPED");
      }
      // FOOD PIPE ACTUATOR
      else if (command == "FOOD PIPE OPEN" || command == "FOOD PIPE ON" || command == "FOOD_PIPE OPEN" || command == "FOOD_PIPE 1" || command == "FOODPIPE OPEN" || command == "FOODPIPE ON") {
        foodPipeOpen = true;
        Serial.println("FOOD PIPE OPEN");
      }
      else if (command == "FOOD PIPE CLOSE" || command == "FOOD PIPE OFF" || command == "FOOD_PIPE CLOSE" || command == "FOOD_PIPE 0" || command == "FOODPIPE CLOSE" || command == "FOODPIPE OFF") {
        foodPipeOpen = false;
        Serial.println("FOOD PIPE CLOSED");
      }
      else if (command == "RIGHT") {
        thirdTarget = THIRD_RIGHT;
        Serial.println("THIRD SERVO RIGHT - 50");
      }
      else if (command == "LEFT") {
        thirdTarget = THIRD_LEFT;
        Serial.println("THIRD SERVO LEFT - 140");
      }
      else if (command == "HOME" || command == "CENTER") {
        thirdTarget = THIRD_HOME;
        mainTarget = MAIN_HOME;
        Serial.println("THIRD SERVO HOME - 90");
      }
      else if (command == "PULSE ON") {
        pulseActive = true;
        pulsePhase = 0;
        pulseTimer = millis();
        Serial.println("PULSE MOVEMENT ON");
      }
      else if (command == "PULSE OFF") {
        pulseActive = false;
        pulseServo1.write(pulseAngle);
        pulseServo2.write(pulseAngle);
        Serial.print("PULSE STOPPED AT: ");
        Serial.println(pulseAngle);
      }
      // =====================================
      // PULSE SERVO SPEED CONTROL
      // COMMANDS:
      //   PULSE SPEED 1   -> HIGH SPEED
      //   PULSE SPEED 20  -> MEDIUM SPEED
      //   PULSE SPEED 35  -> LOW SPEED
      // =====================================
      else if (command.startsWith("PULSE SPEED ") || command.startsWith("PULSESPEED ")) {
        String speedArg = command.substring(command.lastIndexOf(' ') + 1);
        speedArg.trim();

        if (speedArg == "HIGH" || speedArg == "1") {
          heartbeatSpeed = 1;
          Serial.println("PULSE SPEED UPDATED: 1 ms (HIGH)");
        }
        else if (speedArg == "MEDIUM" || speedArg == "MED" || speedArg == "20") {
          heartbeatSpeed = 20;
          Serial.println("PULSE SPEED UPDATED: 20 ms (MEDIUM)");
        }
        else if (speedArg == "LOW" || speedArg == "35") {
          heartbeatSpeed = 35;
          Serial.println("PULSE SPEED UPDATED: 35 ms (LOW)");
        }
        else {
          int newSpeed = speedArg.toInt();
          if (newSpeed >= 1 && newSpeed <= 100) {
            heartbeatSpeed = newSpeed;
            Serial.print("PULSE SPEED UPDATED: ");
            Serial.print(heartbeatSpeed);
            Serial.println(" ms");
          }
          else {
            Serial.println("INVALID SPEED! USE 1 (HIGH), 20 (MEDIUM), 35 (LOW)");
          }
        }
      }
      else if (command == "ESTOP") {
        relayBlinkActive = false;
        relayState = false;
        digitalWrite(relayPin, RELAY_OFF);

        mouthActive = false;
        foodPipeOpen = false;
        pulseActive = false;
        mainTarget = MAIN_HOME;
        thirdTarget = THIRD_HOME;
        pulseServo1.write(0);
        pulseServo2.write(0);
        Serial.println("EMERGENCY STOPPED - RELAY OFF");
      }
      else if (command.startsWith("TILT ")) {
        int val = command.substring(5).toInt();
        mainTarget = constrain(val, 0, 180);
        Serial.print("MAIN SERVO TILT - ");
        Serial.println(mainTarget);
      }
      else if (command.startsWith("PAN ")) {
        int val = command.substring(4).toInt();
        thirdTarget = constrain(val, 0, 180);
        Serial.print("THIRD SERVO PAN - ");
        Serial.println(thirdTarget);
      }
      else if (command.startsWith("POS ")) {
        // High-speed dual Pan & Tilt packet: POS <pan> <tilt> (e.g. POS 90 105)
        int spaceIdx = command.indexOf(' ', 4);
        if (spaceIdx > 0) {
          int pVal = command.substring(4, spaceIdx).toInt();
          int tVal = command.substring(spaceIdx + 1).toInt();
          thirdTarget = constrain(pVal, 0, 180);
          mainTarget = constrain(tVal, 0, 180);
          Serial.print("POS PAN:");
          Serial.print(thirdTarget);
          Serial.print(" TILT:");
          Serial.println(mainTarget);
        }
      }
      else if (isDigit(command.charAt(0))) {
        int val = command.toInt();
        mainTarget = constrain(val, 0, 180);
        Serial.print("MAIN SERVO DIRECT - ");
        Serial.println(mainTarget);
      }
      else {
        Serial.println("INVALID COMMAND");
      }

      command = "";
    }
    else {
      command += c;
    }
  }

  // =====================================
  // RELAY CONTINUOUS ON/OFF CYCLE
  // =====================================
  if (relayBlinkActive) {
    unsigned long currentInterval = relayState ? relayOnTime : relayOffTime;
    if (millis() - relayTimer >= currentInterval) {
      relayTimer = millis();
      relayState = !relayState;
      if (relayState) {
        digitalWrite(relayPin, RELAY_ON);
        Serial.println("RELAY ON");
      }
      else {
        digitalWrite(relayPin, RELAY_OFF);
        Serial.println("RELAY OFF");
      }
    }
  }

  // =====================================
  // MAIN SERVO SMOOTH MOVEMENT
  // =====================================
  if (millis() - mainTimer >= mainSpeed) {
    mainTimer = millis();

    if (mainAngle < mainTarget) {
      mainAngle++;
      mainServo.write(mainAngle);
    }
    else if (mainAngle > mainTarget) {
      mainAngle--;
      mainServo.write(mainAngle);
    }
  }

  // =====================================
  // MOUTH SERVO SWEEP
  // =====================================
  if (mouthActive && millis() - mouthTimer >= mouthSpeed) {
    mouthTimer = millis();

    mouthAngle += mouthDirection;

    if (mouthAngle >= MOUTH_MAX) {
      mouthAngle = MOUTH_MAX;
      mouthDirection = -1;
    }

    if (mouthAngle <= MOUTH_MIN) {
      mouthAngle = MOUTH_MIN;
      mouthDirection = 1;
    }

    mouthServo.write(mouthAngle);
  }

  // =====================================
  // THIRD SERVO SMOOTH MOVEMENT
  // =====================================
  if (millis() - thirdTimer >= thirdSpeed) {
    thirdTimer = millis();

    if (thirdAngle < thirdTarget) {
      thirdAngle++;
      thirdServo.write(thirdAngle);
    }
    else if (thirdAngle > thirdTarget) {
      thirdAngle--;
      thirdServo.write(thirdAngle);
    }
  }

  // =====================================
  // TWO SG90 PULSE SERVOS
  // =====================================
  if (pulseActive) {
    unsigned long currentTime = millis();

    switch (pulsePhase) {
      // FORWARD: 0 TO 20
      case 0:
        if (currentTime - pulseTimer >= heartbeatSpeed) {
          pulseTimer = currentTime;
          pulseAngle++;
          pulseServo1.write(pulseAngle);
          pulseServo2.write(pulseAngle);

          if (pulseAngle >= PULSE_MAX) {
            pulseAngle = PULSE_MAX;
            pulsePhase = 1;
          }
        }
        break;

      // HOLD AT 20
      case 1:
        if (currentTime - pulseTimer >= holdTime) {
          pulseTimer = currentTime;
          pulsePhase = 2;
        }
        break;

      // RETURN: 20 TO 0
      case 2:
        if (currentTime - pulseTimer >= heartbeatSpeed) {
          pulseTimer = currentTime;
          pulseAngle--;
          pulseServo1.write(pulseAngle);
          pulseServo2.write(pulseAngle);

          if (pulseAngle <= PULSE_MIN) {
            pulseAngle = PULSE_MIN;
            pulsePhase = 3;
          }
        }
        break;

      // PAUSE AT 0
      case 3:
        if (currentTime - pulseTimer >= pauseTime) {
          pulseTimer = currentTime;
          pulsePhase = 0;
        }
        break;
    }
  }
}
