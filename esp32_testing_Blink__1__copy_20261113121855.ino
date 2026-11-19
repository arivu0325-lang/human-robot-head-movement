/*
  =============================================================================
  RoboHead - 5-Servo Robot Controller Firmware
  =============================================================================
  Hardware Pin Assignments:
    - Pin 9  : Main Servo    (Tilt Up / Down)
    - Pin 3  : Mouth Servo   (Mouth Open / Close Sweep)
    - Pin 6  : Third Servo   (Pan Left / Right / Home)
    - Pin 2  : Pulse Servo 1 (Heartbeat Pulse SG90)
    - Pin 12 : Pulse Servo 2 (Heartbeat Pulse SG90)
    - Pin 4  : Relay Blower  (Air Blower / Breath Motor)

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
    - 1 / BLOW ON / BLOW    -> Blower Relay ON (Pin 4 HIGH), mouth sweep & wind pulse
    - 0 / BLOW OFF          -> Blower Relay OFF (Pin 4 LOW), stops mouth & pulse
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

// MAIN SERVO (Pin 9 - Tilt Up / Down)
int mainAngle = 90;
int mainTarget = 90;

const int MAIN_UP = 140;
const int MAIN_DOWN = 65;
const int MAIN_HOME = 90;

unsigned long mainTimer = 0;
const int mainSpeed = 15;

// MOUTH SERVO (Pin 3 - Mouth Actuator)
int mouthAngle = 90;
int mouthDirection = 1;

const int MOUTH_MIN = 90;
const int MOUTH_MAX = 130;

unsigned long mouthTimer = 0;
const int mouthSpeed = 15;

bool mouthActive = false;

// THIRD SERVO (Pin 6 - Pan Left / Right)
int thirdAngle = 90;
int thirdTarget = 90;

const int THIRD_RIGHT = 50;
const int THIRD_LEFT = 140;
const int THIRD_HOME = 90;

unsigned long thirdTimer = 0;
const int thirdSpeed = 15;

// PULSE SERVOS (SG90 - Pin 2 & Pin 12)
int pulseAngle = 0;

const int PULSE_MIN = 0;
const int PULSE_MAX = 20;

const int heartbeatSpeed = 10;
const int holdTime = 100;
const int pauseTime = 300;

bool pulseActive = false;
int pulsePhase = 0;
unsigned long pulseTimer = 0;

// =====================================
// RELAY (PIN 4 - AIR BLOWER / BREATH)
// =====================================
const int RELAY_PIN = 4;

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

  mainServo.attach(9);
  mouthServo.attach(3);
  thirdServo.attach(6);

  pulseServo1.attach(2);
  pulseServo2.attach(12);

  // RELAY SETUP (PIN 4 - BLOWER)
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

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
  Serial.println("1 / BLOW ON - BLOWER ON (RELAY PIN 4)");
  Serial.println("0 / BLOW OFF - BLOWER OFF (RELAY PIN 4)");
}

void loop() {
  // SERIAL COMMAND READING
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      command.trim();
      command.toUpperCase();

      if (command.length() == 0) {
        // Empty line, ignore
      }
      else if (command == "UP") {
        mainTarget = MAIN_UP;
        Serial.println("MAIN SERVO UP - 140");
      }
      else if (command == "DOWN") {
        mainTarget = MAIN_DOWN;
        Serial.println("MAIN SERVO DOWN - 65");
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
      // BLOWER ON (RELAY PIN 4 HIGH)
      // COMMANDS: 1, BLOW, BLOW ON, BREATH ON
      // =====================================
      else if (command == "1" || command == "BLOW" || command == "BLOW ON" || command == "BREATH ON") {
        digitalWrite(RELAY_PIN, HIGH);
        mouthActive = true;
        pulseActive = true;
        pulsePhase = 0;
        pulseTimer = millis();
        Serial.println("BREATH ON - RELAY ON");
      }
      // =====================================
      // BLOWER OFF (RELAY PIN 4 LOW)
      // COMMANDS: 0, BLOW OFF, BREATH OFF
      // =====================================
      else if (command == "0" || command == "BLOW OFF" || command == "BREATH OFF") {
        digitalWrite(RELAY_PIN, LOW);
        mouthActive = false;
        pulseActive = false;
        mouthServo.write(mouthAngle);
        pulseServo1.write(0);
        pulseServo2.write(0);
        Serial.println("BREATH OFF - RELAY OFF");
      }
      else if (command == "ESTOP") {
        digitalWrite(RELAY_PIN, LOW);
        mouthActive = false;
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

  // MAIN SERVO SMOOTH MOVEMENT
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

  // MOUTH SERVO SWEEP
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

  // THIRD SERVO SMOOTH MOVEMENT
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

  // TWO SG90 PULSE SERVOS (HEARTBEAT DYNAMICS)
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
