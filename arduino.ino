/*
  =============================================================================
  RoboHead - 6-Servo + Active-LOW Relay Blower Controller Firmware
  =============================================================================
  Hardware Pin Assignments:
    - Pin 9  : Main Servo    (Tilt Up / Down - 50 deg to 120 deg, Home: 90 deg)
    - Pin 3  : Mouth Servo   (Mouth Sweep - 90 deg to 130 deg, Home: 90 deg)
    - Pin 6  : Third Servo   (Pan Left / Right - 50 deg to 140 deg, Home: 90 deg)
    - Pin 2  : Pulse Servo 1 (Heartbeat Pulse SG90 - 0 deg to 20 deg)
    - Pin 12 : Pulse Servo 2 (Heartbeat Pulse SG90 - 0 deg to 20 deg)
    - Pin 8  : Lung Servo    (Lung Servo - 50 deg to 150 deg, Home: 60 deg)
    - Pin 4  : Relay Blower  (Active-LOW Relay - Air Blower / Relay Blink Cycle)

  Baud Rate: 9600 Baud (Works with Mission Control Web Serial & Serial Monitor)

  Serial Commands:
    - 1                     -> Relay Blink Cycle ON (Pin 4 Active-LOW, blinks every 2000ms)
    - 0                     -> Relay OFF
    - UP                    -> Main Servo tilts UP (50 deg)
    - DOWN                  -> Main Servo tilts DOWN (120 deg)
    - MOUTH ON              -> Mouth sweeps between 90 deg and 130 deg
    - MOUTH OFF             -> Mouth stops and holds angle
    - RIGHT                 -> Third Servo pans RIGHT (50 deg)
    - LEFT                  -> Third Servo pans LEFT (140 deg)
    - HOME / CENTER         -> Third Servo & Main Servo return to CENTER (90 deg)
    - PULSE ON              -> Two SG90 pulse servos run heartbeat rhythm
    - PULSE OFF             -> Two SG90 pulse servos stop
    - PULSE SPEED <1-100>   -> Set heartbeat step interval in ms (e.g. PULSE SPEED 20)
    - LOW                   -> Lung Servo moves to 80 deg (LOW)
    - MEDIUM                -> Lung Servo moves to 110 deg (MEDIUM)
    - HIGH                  -> Lung Servo moves to 150 deg (HIGH)
    - LUNG SPEED <1-100>    -> Set lung step interval in ms (e.g. LUNG SPEED 15)
    - LUNG <50-150>         -> Set Lung Servo to exact degree (e.g. LUNG 80, LUNG 110, LUNG 150)
    - TILT <angle>          -> Direct positioning of Main Servo (50-120 deg)
    - PAN <angle>           -> Direct positioning of Third Servo (50-140 deg)
    - POS <pan> <tilt>      -> Simultaneous positioning of Pan & Tilt
    - INTERVAL <sec>        -> Set Relay Blink Interval in seconds (e.g. INTERVAL 2.0)
    - ESTOP                 -> Emergency stop all active loops & return home
  =============================================================================
*/

#if defined(ESP32)
  #include <ESP32Servo.h>
#else
  #include <Servo.h>
#endif

// =====================================
// SERVO OBJECTS
// =====================================

Servo mainServo;
Servo mouthServo;
Servo thirdServo;
Servo pulseServo1;
Servo pulseServo2;
Servo lungServo;

// =====================================
// RELAY
// =====================================

const int relayPin = 4;

const int RELAY_ON = LOW;
const int RELAY_OFF = HIGH;

bool relayBlinkActive = false;
bool relayState = false;

unsigned long relayTimer = 0;
unsigned long relayInterval = 2000;

// =====================================
// MAIN SERVO
// =====================================

int mainAngle = 90;
int mainTarget = 90;

const int MAIN_UP = 50;
const int MAIN_DOWN = 120;

unsigned long mainTimer = 0;
const int mainSpeed = 15;

// =====================================
// MOUTH SERVO
// =====================================

int mouthAngle = 90;
int mouthDirection = 1;

const int MOUTH_MIN = 90;
const int MOUTH_MAX = 130;

unsigned long mouthTimer = 0;
const int mouthSpeed = 15;

bool mouthActive = false;

// =====================================
// THIRD SERVO
// =====================================

int thirdAngle = 90;
int thirdTarget = 90;

const int THIRD_RIGHT = 50;
const int THIRD_LEFT = 140;
const int THIRD_HOME = 90;

unsigned long thirdTimer = 0;
const int thirdSpeed = 15;

// =====================================
// PULSE SERVOS (SG90)
// =====================================

int pulseAngle = 0;

const int PULSE_MIN = 0;
const int PULSE_MAX = 20;

int heartbeatSpeed = 20;

const int holdTime = 100;
const int pauseTime = 300;

bool pulseActive = false;

int pulsePhase = 0;

unsigned long pulseTimer = 0;

// =====================================
// LUNG SERVO
// =====================================

int lungAngle = 60;
int lungTarget = 60;

const int LUNG_MIN = 50;
const int LUNG_MAX = 150;

unsigned long lungTimer = 0;

int lungSpeed = 15;

// =====================================
// SERIAL COMMAND
// =====================================

String command = "";

void setup() {

  Serial.begin(9600);

  // RELAY SETUP
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, RELAY_OFF);

  // SERVO ATTACHMENTS
  mainServo.attach(9);
  mouthServo.attach(3);
  thirdServo.attach(6);

  pulseServo1.attach(2);
  pulseServo2.attach(12);

  lungServo.attach(8);

  // SERVO HOME POSITIONS
  mainServo.write(90);
  mouthServo.write(90);
  thirdServo.write(90);

  pulseServo1.write(0);
  pulseServo2.write(0);

  lungServo.write(60);

  Serial.println("ROBOT SERVO CONTROL READY");

  Serial.println("-----------------------------");

  Serial.println("MAIN SERVO: UP / DOWN");

  Serial.println("MOUTH SERVO: MOUTH ON / MOUTH OFF");

  Serial.println("THIRD SERVO: RIGHT / LEFT / HOME");

  Serial.println("PULSE SERVO: PULSE ON / PULSE OFF");

  Serial.println("PULSE SPEED 1 TO 100");

  Serial.println("-----------------------------");

  Serial.println("LUNG SERVO COMMANDS:");

  Serial.println("LUNG 80 = LOW");

  Serial.println("LUNG 110 = MEDIUM");

  Serial.println("LUNG 150 = HIGH");

  Serial.println("LUNG SPEED 1 TO 100");

  Serial.println("OTHER LUNG ANGLES: 50 TO 150");

  Serial.println("-----------------------------");

  Serial.println("1 = RELAY BLINK ON");

  Serial.println("0 = RELAY OFF");
}

// =====================================
// MAIN LOOP
// =====================================

void loop() {

  // =====================================
  // SERIAL COMMAND READING
  // =====================================

  while (Serial.available() > 0) {

    char c = Serial.read();

    if (c == '\n' || c == '\r') {

      command.trim();
      command.toUpperCase();

      // RELAY ON
      if (command == "1" || command == "BLOW ON" || command == "BLOW") {

        relayBlinkActive = true;
        relayState = true;

        digitalWrite(relayPin, RELAY_ON);

        relayTimer = millis();

        Serial.println("RELAY BLINKING STARTED");

      }

      // RELAY OFF
      else if (command == "0" || command == "BLOW OFF") {

        relayBlinkActive = false;
        relayState = false;

        digitalWrite(relayPin, RELAY_OFF);

        Serial.println("RELAY STOPPED - OFF");

      }

      // =====================================
      // MAIN SERVO
      // =====================================

      else if (command == "UP") {

        mainTarget = MAIN_UP;

        Serial.println("MAIN SERVO UP - 50");

      }

      else if (command == "DOWN") {

        mainTarget = MAIN_DOWN;

        Serial.println("MAIN SERVO DOWN - 120");

      }

      // =====================================
      // MOUTH SERVO
      // =====================================

      else if (command == "MOUTH ON" || command == "MOUTH OPEN") {

        mouthActive = true;

        Serial.println("MOUTH ON");

      }

      else if (command == "MOUTH OFF" || command == "MOUTH CLOSE") {

        mouthActive = false;

        mouthServo.write(mouthAngle);

        Serial.println("MOUTH STOPPED");

      }

      // =====================================
      // THIRD SERVO
      // =====================================

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
        mainTarget = 90;

        Serial.println("THIRD SERVO HOME - 90");

      }

      // =====================================
      // PULSE SERVO ON
      // =====================================

      else if (command == "PULSE ON") {

        pulseActive = true;

        pulsePhase = 0;

        pulseTimer = millis();

        Serial.println("PULSE MOVEMENT ON");

      }

      // PULSE SERVO OFF
      else if (command == "PULSE OFF") {

        pulseActive = false;

        pulseServo1.write(pulseAngle);
        pulseServo2.write(pulseAngle);

        Serial.print("PULSE STOPPED AT: ");
        Serial.println(pulseAngle);

      }

      // =====================================
      // PULSE SPEED CONTROL
      // =====================================

      else if (command.startsWith("PULSE SPEED ")) {

        int newSpeed = command.substring(12).toInt();

        if (newSpeed >= 1 && newSpeed <= 100) {

          heartbeatSpeed = newSpeed;

          Serial.print("PULSE SPEED UPDATED: ");

          Serial.print(heartbeatSpeed);

          Serial.println(" ms");

        }

        else {

          Serial.println("INVALID SPEED! USE 1 TO 100");

        }
      }

      // =====================================
      // LUNG SERVO SPEED CONTROL
      // =====================================

      else if (command.startsWith("LUNG SPEED ")) {

        int newSpeed = command.substring(11).toInt();

        if (newSpeed >= 1 && newSpeed <= 100) {

          lungSpeed = newSpeed;

          Serial.print("LUNG SPEED UPDATED: ");

          Serial.print(lungSpeed);

          Serial.println(" ms");

        }

        else {

          Serial.println("INVALID LUNG SPEED! USE 1 TO 100");

        }
      }

      // =====================================
      // LUNG SERVO PRESETS & STEP COMMANDS
      // =====================================

      else if (
        command == "LOW" || command == "STEP 1" || command == "STEP1" || command == "STEP_1" ||
        command == "LUNG 1" || command == "LUNGS 1" || command == "LUNG1" || command == "LUNGS1" ||
        command == "LUNG STEP 1" || command == "LUNG STEP1" || command == "LUNGS STEP 1" ||
        command == "LUNG LOW" || command == "LUNGS LOW" || command == "LUNG 80" || command == "LUNGS 80"
      ) {

        lungTarget = 80;

        Serial.println("LUNG: LOW - 80 DEGREE (STEP 1)");

      }

      else if (
        command == "MEDIUM" || command == "MED" || command == "STEP 2" || command == "STEP2" || command == "STEP_2" ||
        command == "LUNG 2" || command == "LUNGS 2" || command == "LUNG2" || command == "LUNGS2" ||
        command == "LUNG STEP 2" || command == "LUNG STEP2" || command == "LUNGS STEP 2" ||
        command == "LUNG MED" || command == "LUNG MEDIUM" || command == "LUNGS MED" || command == "LUNGS MEDIUM" ||
        command == "LUNG 110" || command == "LUNGS 110"
      ) {

        lungTarget = 110;

        Serial.println("LUNG: MEDIUM - 110 DEGREE (STEP 2)");

      }

      else if (
        command == "HIGH" || command == "STEP 3" || command == "STEP3" || command == "STEP_3" ||
        command == "LUNG 3" || command == "LUNGS 3" || command == "LUNG3" || command == "LUNGS3" ||
        command == "LUNG STEP 3" || command == "LUNG STEP3" || command == "LUNGS STEP 3" ||
        command == "LUNG HIGH" || command == "LUNGS HIGH" || command == "LUNG 150" || command == "LUNGS 150"
      ) {

        lungTarget = 150;

        Serial.println("LUNG: HIGH - 150 DEGREE (STEP 3)");

      }

      else if (command == "LUNG HOME" || command == "LUNGS HOME" || command == "LUNG 60" || command == "LUNGS 60") {

        lungTarget = 60;

        Serial.println("LUNG: HOME - 60 DEGREE");

      }

      // =====================================
      // LUNG SERVO ANGLE CONTROL (DYNAMIC)
      // =====================================

      else if (command.startsWith("LUNG ") || command.startsWith("LUNGS ") || command.startsWith("LUNG=") || command.startsWith("LUNGS=")) {

        int valIdx = command.indexOf(' ');
        if (valIdx < 0) valIdx = command.indexOf('=');
        String angleText = command.substring(valIdx + 1);
        angleText.trim();

        if (angleText == "1" || angleText == "STEP 1" || angleText == "STEP1" || angleText == "LOW") {
          lungTarget = 80;
          Serial.println("LUNG: LOW - 80 DEGREE (STEP 1)");
        }
        else if (angleText == "2" || angleText == "STEP 2" || angleText == "STEP2" || angleText == "MED" || angleText == "MEDIUM") {
          lungTarget = 110;
          Serial.println("LUNG: MEDIUM - 110 DEGREE (STEP 2)");
        }
        else if (angleText == "3" || angleText == "STEP 3" || angleText == "STEP3" || angleText == "HIGH") {
          lungTarget = 150;
          Serial.println("LUNG: HIGH - 150 DEGREE (STEP 3)");
        }
        else {
          bool validNumber = angleText.length() > 0;

          for (unsigned int i = 0; i < angleText.length(); i++) {

            if (!isDigit(angleText[i])) {
              validNumber = false;
            }

          }

          if (validNumber) {

            int newAngle = angleText.toInt();

            if (newAngle >= LUNG_MIN && newAngle <= LUNG_MAX) {

              lungTarget = newAngle;

              Serial.print("LUNG SERVO TARGET: ");

              Serial.print(lungTarget);

              if (newAngle == 80) {
                Serial.println(" - LOW (STEP 1)");
              }

              else if (newAngle == 110) {
                Serial.println(" - MEDIUM (STEP 2)");
              }

              else if (newAngle == 150) {
                Serial.println(" - HIGH (STEP 3)");
              }

              else {
                Serial.println(" DEGREE");
              }

            }

            else {

              Serial.println("INVALID ANGLE! USE 50 TO 150");

            }

          }

          else {

            Serial.println("INVALID LUNG ANGLE");

          }
        }
      }

      // =====================================
      // EXTENDED DASHBOARD PROTOCOL COMPATIBILITY
      // =====================================

      // DIRECT TILT COMMAND
      else if (command.startsWith("TILT ")) {
        int a = command.substring(5).toInt();
        if (a >= 0 && a <= 180) {
          mainTarget = constrain(a, MAIN_UP, MAIN_DOWN);
          Serial.print("MAIN TILT TARGET: ");
          Serial.println(mainTarget);
        }
      }

      // DIRECT PAN COMMAND
      else if (command.startsWith("PAN ")) {
        int a = command.substring(4).toInt();
        if (a >= 0 && a <= 180) {
          thirdTarget = constrain(a, THIRD_RIGHT, THIRD_LEFT);
          Serial.print("THIRD PAN TARGET: ");
          Serial.println(thirdTarget);
        }
      }

      // SIMULTANEOUS POS PAN TILT
      else if (command.startsWith("POS ")) {
        int spaceIdx = command.indexOf(' ', 4);
        if (spaceIdx > 0) {
          int p = command.substring(4, spaceIdx).toInt();
          int t = command.substring(spaceIdx + 1).toInt();
          thirdTarget = constrain(p, THIRD_RIGHT, THIRD_LEFT);
          mainTarget = constrain(t, MAIN_UP, MAIN_DOWN);
          Serial.print("POS SET: PAN=");
          Serial.print(thirdTarget);
          Serial.print(" TILT=");
          Serial.println(mainTarget);
        }
      }

      // RELAY BLINK INTERVAL (SECONDS)
      else if (command.startsWith("INTERVAL ")) {
        float sec = command.substring(9).toFloat();
        if (sec >= 0.1 && sec <= 10.0) {
          relayInterval = (unsigned long)(sec * 1000.0);
          Serial.print("RELAY INTERVAL UPDATED: ");
          Serial.print(sec, 1);
          Serial.println(" s");
        }
      }

      // EMERGENCY STOP
      else if (command == "ESTOP") {
        relayBlinkActive = false;
        relayState = false;
        digitalWrite(relayPin, RELAY_OFF);
        pulseActive = false;
        mouthActive = false;
        thirdTarget = THIRD_HOME;
        mainTarget = 90;
        lungTarget = 60;
        Serial.println("EMERGENCY STOP EXECUTED - ALL STOPPED");
      }

      // =====================================
      // INVALID COMMAND
      // =====================================

      else if (command.length() > 0) {

        Serial.println("INVALID COMMAND");

      }

      command = "";

    }

    else {

      command += c;

    }
  }

  // =====================================
  // RELAY BLINK CONTROL
  // =====================================

  if (relayBlinkActive) {

    if (millis() - relayTimer >= relayInterval) {

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

  if (mouthActive &&
      millis() - mouthTimer >= mouthSpeed) {

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
  // LUNG SERVO SMOOTH MOVEMENT
  // =====================================

  if (millis() - lungTimer >= lungSpeed) {

    lungTimer = millis();

    if (lungAngle < lungTarget) {

      lungAngle++;

      lungServo.write(lungAngle);

    }

    else if (lungAngle > lungTarget) {

      lungAngle--;

      lungServo.write(lungAngle);

    }
  }

  // =====================================
  // TWO SG90 PULSE SERVOS
  // =====================================

  if (pulseActive) {

    unsigned long currentTime = millis();

    switch (pulsePhase) {

      // MOVE FORWARD
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

      // HOLD POSITION
      case 1:

        if (currentTime - pulseTimer >= holdTime) {

          pulseTimer = currentTime;

          pulsePhase = 2;

        }

        break;

      // MOVE BACKWARD
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

      // PAUSE
      case 3:

        if (currentTime - pulseTimer >= pauseTime) {

          pulseTimer = currentTime;

          pulsePhase = 0;

        }

        break;

    }
  }
}
