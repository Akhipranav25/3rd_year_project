// Robot Control Code 9.3 - based on 9.1; clamps time interval for yaw integral to prevent excessive actuation and reverts to setting values less than 1 to 1 for translation because it took longer to adjust and not just when at 0 degrees to target, but at other angles as well, leading to significant time wastage
//changed yaw proportional gains from 2 to 4 and yaw derivative gains from 2 to 3
#define EN_PIN  A3
#define STEP_PIN A4
#define DIR_PIN  A5

int relay_pin1 = 24;
int relay_pin2 = 25;

int directionPinA = 12;
int pwmPinA       = 3;
int brakePinA     = 9;

int directionPinB = 13;
int pwmPinB       = 11;
int brakePinB     = 8;

float commandValue = 0;

float Kp_yaw = 4;
float Kd_yaw = 3; //originally 4, then 2, then 1, then 1.5, then 0, then
float Kd = 4000; //started with 2500 because assume sample time is 250ms (based on Python code command interval), so then -4*2500/250=40, which seemed like a good value to reduce control effort by.
float previous_left_commandval = 20;
long prev_l_Time;
float previous_right_commandval = 20;
long prev_r_Time;
float previous_yl_commandval = 90;
long prev_yl_Time;
float previous_yr_commandval = 90;
long prev_yr_Time;
float previous_f_commandval = 0;
long prev_f_Time;
float(TL_integral_sum) = 0;
float(TR_integral_sum) = 0;
float(YL_integral_sum) = 0;
float(YR_integral_sum) = 0;
float Forward_integral_sum = 0;
float Ki = 0.002; // originally 0.001, then 0.005, then 0.001, then 0.01, then 0.005, then 0.002
float Ki_yaw = 0.1; //originally 0.0005, then 0.001, then 0.01, then 0.1
float Ki_forward = 0.00001; // originally 0.0005, then 0.0001, then 0.00001

bool integral_enable = 0;

// ── Non-blocking stepper state ──────────────────────────────────────────────
// Previously moveSteps() used a blocking for-loop with delayMicroseconds().
// With noOfSteps=250 and microSecondsDelay=100 that was 250*2*100 = 50 ms of
// pure blocking per command — long enough to noticeably stall serial comms.
// Now the stepper is driven by micros() so loop() stays free.

int   stepDelay    = 200;          // µs between state transitions (half-period)
long  stepsRemaining = 0;          // >0 = forward, <0 = backward, 0 = idle
bool  stepPinState  = false;       // current STEP_PIN level
unsigned long lastStepMicros = 0;

long  step_counter = 0;            // absolute position tracking

// Call once per loop() to advance the stepper one half-step if due
void updateStepper() {
  if (stepsRemaining == 0) return;

  unsigned long now = micros();
  if (now - lastStepMicros < (unsigned long)stepDelay) return;

  lastStepMicros = now;

  stepPinState = !stepPinState;
  digitalWrite(STEP_PIN, stepPinState ? HIGH : LOW);

  // A full step = one LOW→HIGH transition
  if (stepPinState == HIGH) {
    if (stepsRemaining > 0) {
      stepsRemaining--;
      step_counter++;
    } else {
      stepsRemaining++;
      step_counter--;
    }
  }
}
// ────────────────────────────────────────────────────────────────────────────

int pwmValue = 0;

String data = "";

int percentage_array[10];
String instruction[10];
int command[6] = {-1,-1,-1,-1,-1,-1};
float commandval[6] = {-1.0,-1.0,-1.0,-1.0,-1.0,-1.0};
int percentage_count = 0;

int startSpace     = 0;
int startSeparator = 0;

void processData(String data) {
  int index = data.indexOf('%');

  while (index != -1) {
    percentage_array[percentage_count] = index;
    //Serial.print(percentage_array[percentage_count]);
    //Serial.print(" ");
    percentage_count++;
    index = data.indexOf('%', index + 1);
  }

  for (int p = 0; p < percentage_count; p++) {
    int endSpace   = percentage_array[p];
    instruction[p] = data.substring(startSpace, endSpace);
    //Serial.print(" | ");
    //Serial.print(instruction[p]);
    startSpace = endSpace + 1;
  }
  startSpace = 0;

  for (int m = 0; m < percentage_count; m++) {
    startSeparator = 0;
    int endSeparator = instruction[m].indexOf("?");
    if (endSeparator == -1) {
      command[m] = instruction[m].substring(startSeparator).toInt();
      commandval[m] = 0;
    } else {
      command[m] = instruction[m].substring(startSeparator, endSeparator).toInt();
      startSeparator = endSeparator+1;
      commandval[m] = instruction[m].substring(startSeparator).toFloat();
    }
    /*Serial.print(m);
    Serial.print(" ");
    Serial.print(command[m]);
    Serial.print(" ");
    Serial.print(commandval[m]);
    Serial.print(" | ");*/
  }
  
  startSeparator = 0;
  percentage_count = 0;
}

void setup() {
  Serial.begin(1000000);

  pinMode(directionPinA, OUTPUT);
  pinMode(pwmPinA,       OUTPUT);
  pinMode(brakePinA,     OUTPUT);

  pinMode(directionPinB, OUTPUT);
  pinMode(pwmPinB,       OUTPUT);
  pinMode(brakePinB,     OUTPUT);

  pinMode(EN_PIN,   OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN,  OUTPUT);
  pinMode(relay_pin1, OUTPUT);
  pinMode(relay_pin2, OUTPUT);

  prev_l_Time = millis();
  prev_r_Time = millis();
  prev_yl_Time = millis();
  prev_yr_Time = millis();
  prev_f_Time = millis();
}

void loop() {

  // ── Always service the stepper first, every loop iteration ────────────────
  updateStepper();
  // ──────────────────────────────────────────────────────────────────────────

  if (Serial.available() > 0) {

    while (Serial.available()) {
      char c = Serial.read();

      if (c == '<') {
        data = "";          // start of new message
      } else if (c == '>') {
        for (int command_iter = 0; command_iter < 6; command_iter++){
          command[command_iter] = -1;
          commandval[command_iter] = 0.0;
        }
        processData(data);  // full message received
      } else {
        data += c;
      }
    }

    for (int com_num = 0; com_num < (sizeof(command) / sizeof(command[0])); com_num++) {

      if (command[com_num] == 1) {
        // Translate left
        TR_integral_sum = 0;
        /*if (commandval[com_num] < 1){
          Ki = 0.4;
        }*/
        long current_l_Time = millis();
        pwmValue = constrain((commandval[com_num] * 1.1 + Kd*((commandval[com_num]-previous_left_commandval)/(current_l_Time-prev_l_Time)) + Ki*TL_integral_sum), 10, 255);
        digitalWrite(relay_pin2, LOW);
        //digitalWrite(brakePinA, HIGH);
        //analogWrite(pwmPinA, 0);
        digitalWrite(directionPinB, HIGH);
        digitalWrite(brakePinB, LOW);
        analogWrite(pwmPinB, pwmValue);
        if (integral_enable == 1){
          if (commandval[com_num] <= 1){
            commandval[com_num] = 1;
            TL_integral_sum = TL_integral_sum + commandval[com_num]*(current_l_Time-prev_l_Time);
          }
          TL_integral_sum = TL_integral_sum + commandval[com_num]*(250);
        }
        previous_left_commandval = commandval[com_num];
        prev_l_Time = current_l_Time;
        //Ki = 0.001;
      }
      else if (command[com_num] == 0) {
        // Translate right
        TL_integral_sum = 0;
        /*if (commandval[com_num] < 1){
          Ki = 0.4;
        }*/
        long current_r_Time = millis();
        pwmValue = constrain((commandval[com_num] * 2 + Kd*((commandval[com_num]-previous_right_commandval)/(current_r_Time-prev_r_Time)) + Ki*TR_integral_sum), 10, 255);
        digitalWrite(relay_pin2, LOW);
        //digitalWrite(brakePinA, HIGH);
        //analogWrite(pwmPinA, 0);
        digitalWrite(directionPinB, LOW);
        digitalWrite(brakePinB, LOW);
        analogWrite(pwmPinB, pwmValue);
        if (integral_enable == 1){
          if (commandval[com_num] <= 1){
            commandval[com_num] = 1;
            TR_integral_sum = TR_integral_sum + commandval[com_num]*(current_r_Time-prev_r_Time);
          }
          TR_integral_sum = TR_integral_sum + commandval[com_num]*(250);
        }
        previous_right_commandval = commandval[com_num];
        prev_r_Time = current_r_Time;
        //Ki = 0.001;
      }
      else if (command[com_num] == 2) {
        // ── FIX: queue steps instead of blocking ──────────────────────────
        // Was: moveSteps(noOfSteps)  — a 50 ms blocking for-loop.
        // Now: set direction once and add to stepsRemaining so updateStepper()
        //      fires them off asynchronously over many loop() iterations.
        if (step_counter <= 40000 && stepsRemaining == 0) {
          digitalWrite(DIR_PIN, LOW);
          stepsRemaining = 250;   // same as noOfSteps
        }
        // ──────────────────────────────────────────────────────────────────
      }
      else if (command[com_num] == 3) {
        // ── FIX: queue steps instead of blocking ──────────────────────────
        if (step_counter >= 0 && stepsRemaining == 0) {
          digitalWrite(DIR_PIN, HIGH);
          stepsRemaining = -250;  // negative = reverse direction
        }
        // ──────────────────────────────────────────────────────────────────
      }
      else if (command[com_num] == 4) {
        pwmValue = constrain((commandval[com_num] * 10), 40, 255);
        digitalWrite(brakePinB, HIGH);
        analogWrite(pwmPinB, 0);
        digitalWrite(directionPinA, LOW);
        digitalWrite(brakePinA, LOW);
        analogWrite(pwmPinA, pwmValue);
      }
      else if (command[com_num] == 5) {
        pwmValue = constrain((commandval[com_num] * 10), 40, 255);
        digitalWrite(brakePinB, HIGH);
        analogWrite(pwmPinB, 0);
        digitalWrite(directionPinA, HIGH);
        digitalWrite(brakePinA, LOW);
        analogWrite(pwmPinA, pwmValue);
      }
      else if (command[com_num] == 6) {
        // Yaw left
        YR_integral_sum = 0;
        long current_yl_Time = millis();
        pwmValue = constrain(commandval[com_num]*Kp_yaw + Kd_yaw*((commandval[com_num]-previous_yl_commandval)/(current_yl_Time-prev_yl_Time)) + Ki_yaw*YL_integral_sum, 10, 150); // + Ki*YL_integral_sum); constraint minimum was originally 60, and max was 255, then 200, then 150
        //digitalWrite(brakePinA, HIGH);
        //analogWrite(pwmPinA, 0);
        digitalWrite(relay_pin2, HIGH);
        digitalWrite(directionPinB, LOW);
        digitalWrite(brakePinB, LOW);
        analogWrite(pwmPinB, pwmValue);
        if (integral_enable == 1){
          if (commandval[com_num] <= 1){
            //commandval[com_num] = 1;
            if (current_yl_Time-prev_yl_Time > 500){
              YL_integral_sum = YL_integral_sum + commandval[com_num]*(500);
            }
            else{
              YL_integral_sum = YL_integral_sum + commandval[com_num]*(current_yl_Time-prev_yl_Time);
            }
          }
          YL_integral_sum = YL_integral_sum + commandval[com_num]*(250);
        }
        previous_yl_commandval = commandval[com_num];
        prev_yl_Time = current_yl_Time;
      }
      else if (command[com_num] == 7) {
        // Yaw right
        YL_integral_sum = 0;
        long current_yr_Time = millis();
        pwmValue = constrain(commandval[com_num]*Kp_yaw + Kd_yaw*((commandval[com_num]-previous_yr_commandval)/(current_yr_Time-prev_yr_Time)) + Ki_yaw*YR_integral_sum, 10, 150); // + Ki*YR_integral_sum; constraint minimum was originally 60, and max was 255, then 200, then 150
        //digitalWrite(brakePinA, HIGH);
        //analogWrite(pwmPinA, 0);
        digitalWrite(relay_pin2, HIGH);
        digitalWrite(directionPinB, HIGH);
        digitalWrite(brakePinB, LOW);
        analogWrite(pwmPinB, pwmValue);
        if (integral_enable == 1){
          if (commandval[com_num] <= 1){
            //commandval[com_num] = 1;
            if (current_yr_Time-prev_yr_Time > 500){
              YR_integral_sum = YR_integral_sum + commandval[com_num]*(500);
            }
            else{
              YR_integral_sum = YR_integral_sum + commandval[com_num]*(current_yr_Time-prev_yr_Time);
            }
          }
          YR_integral_sum = YR_integral_sum + commandval[com_num]*(250);
        }
        previous_yr_commandval = commandval[com_num];
        prev_yr_Time = current_yr_Time;
      }
      else if (command[com_num] == 10) {
        // Move forward
        //integral_enable = 0;
        long current_f_Time = millis();
        pwmValue = constrain((commandval[com_num]-20)*2 + Kd*((commandval[com_num]-previous_f_commandval)/(current_f_Time-prev_f_Time)) + Ki_forward*Forward_integral_sum, 40, 255);
        //digitalWrite(brakePinB, HIGH);
        //analogWrite(pwmPinB, 0);
        digitalWrite(directionPinA, HIGH);
        digitalWrite(brakePinA, LOW);
        analogWrite(pwmPinA, pwmValue);
        if (commandval[com_num] < 50){
          Forward_integral_sum = Forward_integral_sum + commandval[com_num]*(current_f_Time-prev_f_Time);
        }
        previous_f_commandval = commandval[com_num];
        prev_f_Time = current_f_Time;
      }
      else if (command[com_num] == 11) {
        // Stop
        digitalWrite(brakePinB, HIGH);
        analogWrite(pwmPinB, 0);
        digitalWrite(brakePinA, HIGH);
        analogWrite(pwmPinA, 0);
      }
      else if (command[com_num] == 12) {
        // Stop left/right motion
        digitalWrite(brakePinB, HIGH);
        analogWrite(pwmPinB, 0);
      }
      else if (command[com_num] == 13) {
        // Stop forward motion
        digitalWrite(brakePinA, HIGH);
        analogWrite(pwmPinA, 0);
        integral_enable = 1;
        //Kd = 0;
      }
      else if (command[com_num] == 14) {
        // Reset integral sums to zero
        integral_enable = 0;
        TL_integral_sum = 0;
        TR_integral_sum = 0;
        YL_integral_sum = 0;
        YR_integral_sum = 0;
        Forward_integral_sum = 0;
        previous_left_commandval = 100;
        prev_l_Time = millis();
        previous_right_commandval = 100;
        prev_r_Time = millis();
        previous_yl_commandval = 100;
        prev_yl_Time = millis();
        previous_yr_commandval = 100;
        prev_yr_Time = millis();
        previous_f_commandval = 500;
        prev_f_Time = millis();
        Kd = 4000;
      }
    }
  }
}