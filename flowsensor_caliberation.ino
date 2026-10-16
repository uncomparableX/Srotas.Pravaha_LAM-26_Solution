#include <Arduino.h>
#include <vector>
#include <math.h>
#include <stdio.h>
#include <string.h>

#if __has_include(<esp_arduino_version.h>)
  #include <esp_arduino_version.h>
#endif

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  #define USE_LEDC_CORE3 1
#else
  #define USE_LEDC_CORE3 0
#endif

// YF-S401 FLOW SENSOR CALIBRATION - ESP32
//
// Command "s" starts the system AND immediately starts a trial.
// Default duration: 10 seconds.
// Optional duration: "s 15" starts a 15-second trial.
//
// At the end of every trial:
//   1. All pumps turn OFF.
//   2. The solenoid closes.
//   3. Enter collected volume using: v 950  (950 mL example)
//   4. Send s to begin the next trial immediately.
//
// Pump 1 = 100% while trial runs
// Pump 2 = 100% while trial runs
// Pump 3 = OFF
// Solenoid = OPEN while trial runs
//
// f = N / t
// Q_actual = V(L) / t(s) * 60
// Calibration fit: Q = a*f + b

// ---------------------- PIN ASSIGNMENTS ----------------------
const uint8_t PIN_FLOW = 14;
const uint8_t PIN_SOLENOID = 27;

// Pump 1 driver
const uint8_t P1_R_PWM = 33;
const uint8_t P1_L_PWM = 32;
const uint8_t P1_R_EN  = 26;
const uint8_t P1_L_EN  = 13;

// Pump 2 driver
const uint8_t P2_R_PWM = 19;
const uint8_t P2_L_PWM = 18;
const uint8_t P2_R_EN  = 17;
const uint8_t P2_L_EN  = 16;

// Pump 3 driver (OFF during calibration)
const uint8_t P3_R_PWM = 25;
const uint8_t P3_L_PWM = 23;
const uint8_t P3_R_EN  = 22;
const uint8_t P3_L_EN  = 21;

// ---------------------- CONFIGURATION ------------------------
const uint32_t PWM_FREQUENCY_HZ = 20000;
const uint8_t PWM_RESOLUTION_BITS = 8;

const uint32_t DEFAULT_TRIAL_SECONDS = 10;
const uint32_t MAX_TRIAL_SECONDS = 3600;

// Nominal comparison only. Verify against your exact sensor variant.
// Experimental calibration uses independently measured water volume.
const float NOMINAL_PULSES_PER_LITER = 5880.0f;

// Assumes GPIO27 HIGH opens the solenoid.
// Change if your interface is active-LOW.
const uint8_t SOLENOID_OPEN_LEVEL = HIGH;

// Pump direction conventions from the existing project sketch.
const bool P1_FORWARD_USES_R = true;
const bool P2_FORWARD_USES_R = true;
const bool P3_FORWARD_USES_R = true;

// ---------------------- PWM CHANNELS -------------------------
const uint8_t P1_R_CH = 0;
const uint8_t P1_L_CH = 1;
const uint8_t P2_R_CH = 2;
const uint8_t P2_L_CH = 3;
const uint8_t P3_R_CH = 4;
const uint8_t P3_L_CH = 5;

// ---------------------- FLOW COUNTER -------------------------
volatile uint32_t flowPulseCount = 0;

void IRAM_ATTR flowPulseISR() {
  flowPulseCount++;
}

// ---------------------- TRIAL STORAGE ------------------------
struct TrialData {
  uint32_t trialNumber;
  uint32_t pulses;
  float elapsedSeconds;
  float frequencyHz;
  float volumeLiters;
  float actualFlowLpm;
  float nominalSensorFlowLpm;
};

// No fixed five-trial limit; limited by available ESP32 RAM.
std::vector<TrialData> trials;

// ---------------------- RUNTIME STATE ------------------------
bool trialRunning = false;
bool awaitingVolume = false;

uint32_t trialStartMillis = 0;
uint32_t trialDurationMillis = 0;

uint32_t lastTrialPulses = 0;
float lastTrialElapsedSeconds = 0.0f;

char inputLine[80];
size_t inputLength = 0;


void attachPwmPin(uint8_t pin, uint8_t channel) {
#if USE_LEDC_CORE3
  (void)channel;
  ledcAttach(pin, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
#else
  ledcSetup(channel, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
  ledcAttachPin(pin, channel);
#endif
}

void writePwmPin(
  uint8_t pin,
  uint8_t channel,
  uint32_t duty
) {
#if USE_LEDC_CORE3
  (void)channel;
  ledcWrite(pin, duty);
#else
  (void)pin;
  ledcWrite(channel, duty);
#endif
}

void initPumpPwm(
  uint8_t rPwm,
  uint8_t lPwm,
  uint8_t rChannel,
  uint8_t lChannel
) {
  pinMode(rPwm, OUTPUT);
  pinMode(lPwm, OUTPUT);

  attachPwmPin(rPwm, rChannel);
  attachPwmPin(lPwm, lChannel);

  writePwmPin(rPwm, rChannel, 0);
  writePwmPin(lPwm, lChannel, 0);
}

void setPump(
  uint8_t rPwm,
  uint8_t lPwm,
  uint8_t rEn,
  uint8_t lEn,
  uint8_t rChannel,
  uint8_t lChannel,
  bool forwardUsesR,
  uint8_t percent
) {
  if (percent == 0) {
    writePwmPin(rPwm, rChannel, 0);
    writePwmPin(lPwm, lChannel, 0);

    digitalWrite(rEn, LOW);
    digitalWrite(lEn, LOW);
    return;
  }

  uint32_t duty =
    (percent >= 100)
      ? 255U
      : ((uint32_t)percent * 255U + 50U) / 100U;

  digitalWrite(rEn, HIGH);
  digitalWrite(lEn, HIGH);

  if (forwardUsesR) {
    writePwmPin(rPwm, rChannel, duty);
    writePwmPin(lPwm, lChannel, 0);
  } else {
    writePwmPin(rPwm, rChannel, 0);
    writePwmPin(lPwm, lChannel, duty);
  }
}

void setPump1(uint8_t percent) {
  setPump(
    P1_R_PWM, P1_L_PWM, P1_R_EN, P1_L_EN,
    P1_R_CH, P1_L_CH, P1_FORWARD_USES_R, percent
  );
}

void setPump2(uint8_t percent) {
  setPump(
    P2_R_PWM, P2_L_PWM, P2_R_EN, P2_L_EN,
    P2_R_CH, P2_L_CH, P2_FORWARD_USES_R, percent
  );
}

void setPump3(uint8_t percent) {
  setPump(
    P3_R_PWM, P3_L_PWM, P3_R_EN, P3_L_EN,
    P3_R_CH, P3_L_CH, P3_FORWARD_USES_R, percent
  );
}

void setSolenoidOpen(bool open) {
  digitalWrite(
    PIN_SOLENOID,
    open
      ? SOLENOID_OPEN_LEVEL
      : (SOLENOID_OPEN_LEVEL == HIGH ? LOW : HIGH)
  );
}

void stopHardwareOutputs() {
  setPump1(0);
  setPump2(0);
  setPump3(0);
  setSolenoidOpen(false);
}

void printHelp() {
  Serial.println();
  Serial.println(F("======= YF-S401 CALIBRATION COMMANDS ======="));
  Serial.println(F("s        Start pumps/solenoid and a 10-second trial"));
  Serial.println(F("s 15     Start immediately with a 15-second trial"));
  Serial.println(F("v 950    Save measured volume for completed trial (mL)"));
  Serial.println(F("summary  Print trial table and calibration equation"));
  Serial.println(F("done     Stop hardware and print final summary"));
  Serial.println(F("x        Stop/abort; unfinished trial is discarded"));
  Serial.println(F("?        Show this help"));
  Serial.println();
  Serial.println(F("After each trial: hardware stops automatically."));
  Serial.println(F("Enter v <mL>, then send s to start the next trial."));
  Serial.println(F("Serial Monitor: 115200 baud, Newline."));
}

void printTrialTable() {
  Serial.println();
  Serial.println(F("TRIAL RESULTS - CSV FORMAT"));

  Serial.println(
    F("trial,pulses,time_s,frequency_Hz,volume_L,"
      "actual_flow_Lmin,nominal_sensor_flow_Lmin")
  );

  for (size_t i = 0; i < trials.size(); i++) {
    const TrialData &t = trials[i];

    Serial.printf(
      "%lu,%lu,%.3f,%.4f,%.4f,%.4f,%.4f\n",
      (unsigned long)t.trialNumber,
      (unsigned long)t.pulses,
      t.elapsedSeconds,
      t.frequencyHz,
      t.volumeLiters,
      t.actualFlowLpm,
      t.nominalSensorFlowLpm
    );
  }
}

void printCalibrationSummary() {
  printTrialTable();

  if (trials.empty()) {
    Serial.println(F("No completed trials yet."));
    return;
  }

  double sumPulses = 0.0;
  double sumVolumeL = 0.0;
  double sumF = 0.0;
  double sumQ = 0.0;
  double sumF2 = 0.0;
  double sumFQ = 0.0;

  for (size_t i = 0; i < trials.size(); i++) {
    const TrialData &t = trials[i];

    sumPulses += t.pulses;
    sumVolumeL += t.volumeLiters;
    sumF += t.frequencyHz;
    sumQ += t.actualFlowLpm;
    sumF2 += (double)t.frequencyHz * t.frequencyHz;
    sumFQ += (double)t.frequencyHz * t.actualFlowLpm;
  }

  Serial.println();
  Serial.println(F("========== CALIBRATION SUMMARY =========="));

  Serial.printf(
    "Completed trials: %u\n",
    (unsigned)trials.size()
  );

  if (sumVolumeL > 0.0) {
    double pulsesPerLiter = sumPulses / sumVolumeL;

    Serial.printf(
      "Empirical pulses/litre: %.2f\n",
      pulsesPerLiter
    );

    Serial.printf(
      "Zero-intercept estimate: Q = %.7f*f (L/min)\n",
      60.0 / pulsesPerLiter
    );
  }

  if (trials.size() < 2) {
    Serial.println(
      F("Need at least 2 trials to fit Q = a*f + b.")
    );
    return;
  }

  double n = (double)trials.size();
  double denom = n * sumF2 - sumF * sumF;

  if (fabs(denom) < 1e-12) {
    Serial.println(
      F("Cannot fit calibration: all trial frequencies are identical.")
    );
    return;
  }

  // Least-squares linear regression: Q = a*f + b.
  double a = (n * sumFQ - sumF * sumQ) / denom;
  double b = (sumQ - a * sumF) / n;
  double meanQ = sumQ / n;

  double sse = 0.0;
  double sst = 0.0;
  double maxErrorPct = 0.0;
  double sumErrorPct = 0.0;
  unsigned errorCount = 0;

  for (size_t i = 0; i < trials.size(); i++) {
    const TrialData &t = trials[i];

    double predicted = a * t.frequencyHz + b;
    double residual = t.actualFlowLpm - predicted;

    sse += residual * residual;

    double deviation = t.actualFlowLpm - meanQ;
    sst += deviation * deviation;

    if (t.actualFlowLpm > 1e-9) {
      double errorPct =
        fabs(predicted - t.actualFlowLpm) /
        t.actualFlowLpm * 100.0;

      if (errorPct > maxErrorPct) {
        maxErrorPct = errorPct;
      }

      sumErrorPct += errorPct;
      errorCount++;
    }
  }

  double rSquared =
    (sst > 1e-12) ? 1.0 - sse / sst : 1.0;

  double avgErrorPct =
    (errorCount > 0) ? sumErrorPct / errorCount : 0.0;

  Serial.printf(
    "Calibration equation: Q = %.7f*f + %.7f\n",
    a, b
  );

  Serial.printf("a (L/min per Hz) = %.7f\n", a);
  Serial.printf("b (L/min)        = %.7f\n", b);
  Serial.printf("R^2              = %.6f\n", rSquared);

  Serial.printf(
    "Maximum absolute percentage residual = %.2f%%\n",
    maxErrorPct
  );

  Serial.printf(
    "Average absolute percentage residual = %.2f%%\n",
    avgErrorPct
  );

  Serial.println(
    F("Residuals are fitted-data errors, not independent validation errors.")
  );
  Serial.println(F("=========================================="));
}


// START SYSTEM AND BEGIN TRIAL IMMEDIATELY

void startSystemAndTrial(uint32_t durationSeconds) {
  if (trialRunning) {
    Serial.println(F("A trial is already running."));
    return;
  }

  if (awaitingVolume) {
    Serial.println(
      F("Enter the previous trial volume first: v <mL>.")
    );
    return;
  }

  if (durationSeconds < 1 ||
      durationSeconds > MAX_TRIAL_SECONDS) {
    Serial.printf(
      "Duration must be 1-%lu seconds.\n",
      (unsigned long)MAX_TRIAL_SECONDS
    );
    return;
  }

  // Turn the hardware on. The timed trial starts immediately afterward.
  setSolenoidOpen(true);
  setPump3(0);
  setPump1(100);
  setPump2(100);

  trialDurationMillis = durationSeconds * 1000UL;

  // Reset the pulse counter and record the trial start time atomically.
  noInterrupts();
  flowPulseCount = 0;
  trialStartMillis = millis();
  interrupts();

  trialRunning = true;

  Serial.println();
  Serial.printf(
    "SYSTEM STARTED - TRIAL %u STARTED IMMEDIATELY\n",
    (unsigned)(trials.size() + 1)
  );

  Serial.println(
    F("Pump 1 = 100%, Pump 2 = 100%, Pump 3 = OFF, solenoid = OPEN")
  );

  Serial.printf(
    "Timed interval: %lu seconds\n",
    (unsigned long)durationSeconds
  );

  Serial.println(
    F("Do not adjust the valve during this timed trial.")
  );
}


//  TRIAL AND STOP HARDWARE

void finishTrialCapture(uint32_t now) {
  noInterrupts();
  uint32_t count = flowPulseCount;
  interrupts();

  uint32_t elapsedMs = now - trialStartMillis;
  float elapsedSeconds = elapsedMs / 1000.0f;

  if (elapsedSeconds <= 0.0f) {
    elapsedSeconds = 0.001f;
  }

  // Stop all hardware as soon as the timed trial ends.
  stopHardwareOutputs();

  lastTrialPulses = count;
  lastTrialElapsedSeconds = elapsedSeconds;

  trialRunning = false;
  awaitingVolume = true;

  float frequency = count / elapsedSeconds;

  float nominalFlow =
    frequency * 60.0f / NOMINAL_PULSES_PER_LITER;

  Serial.println();
  Serial.println(F("TRIAL COMPLETE - SYSTEM STOPPED"));

  Serial.println(
    F("Pump 1 = OFF; Pump 2 = OFF; Pump 3 = OFF; solenoid = CLOSED")
  );

  Serial.printf(
    "Trial number: %u\n",
    (unsigned)(trials.size() + 1)
  );

  Serial.printf("Pulses, N: %lu\n", (unsigned long)count);

  Serial.printf(
    "Actual elapsed time: %.3f s\n",
    elapsedSeconds
  );

  Serial.printf(
    "Frequency f=N/t: %.4f Hz\n",
    frequency
  );

  Serial.printf(
    "Nominal sensor flow: %.4f L/min (%.1f pulses/L)\n",
    nominalFlow,
    NOMINAL_PULSES_PER_LITER
  );

  Serial.println(
    F("Measure volume from this same timed interval and enter: v <volume_mL>")
  );

  Serial.println(
    F("After saving the volume, send s (or s <seconds>) for the next trial.")
  );
}

// SAVE INDEPENDENT VOLUME MEASUREMENT

void acceptMeasuredVolume(float volumeMl) {
  if (!awaitingVolume) {
    Serial.println(
      F("No completed trial is waiting for volume. Send s to run a trial.")
    );
    return;
  }

  if (!(volumeMl > 0.0f && volumeMl <= 100000.0f)) {
    Serial.println(
      F("Volume must be >0 and <=100000 mL.")
    );
    return;
  }

  TrialData t;

  t.trialNumber = (uint32_t)trials.size() + 1;
  t.pulses = lastTrialPulses;
  t.elapsedSeconds = lastTrialElapsedSeconds;

  t.frequencyHz =
    lastTrialPulses / lastTrialElapsedSeconds;

  t.volumeLiters = volumeMl / 1000.0f;

  t.actualFlowLpm =
    t.volumeLiters / lastTrialElapsedSeconds * 60.0f;

  t.nominalSensorFlowLpm =
    t.frequencyHz * 60.0f / NOMINAL_PULSES_PER_LITER;

  trials.push_back(t);
  awaitingVolume = false;

  Serial.println();
  Serial.println(F("TRIAL SAVED - SYSTEM REMAINS STOPPED"));

  Serial.printf(
    "Trial:                %lu\n",
    (unsigned long)t.trialNumber
  );

  Serial.printf(
    "Pulses:               %lu\n",
    (unsigned long)t.pulses
  );

  Serial.printf(
    "Time:                 %.3f s\n",
    t.elapsedSeconds
  );

  Serial.printf(
    "Frequency:            %.4f Hz\n",
    t.frequencyHz
  );

  Serial.printf(
    "Collected volume:     %.4f L\n",
    t.volumeLiters
  );

  Serial.printf(
    "Actual flow Q=V/t:    %.4f L/min\n",
    t.actualFlowLpm
  );

  Serial.printf(
    "Nominal sensor flow:  %.4f L/min\n",
    t.nominalSensorFlowLpm
  );

  Serial.println(
    F("Set the next valve opening, then send s to start the next trial immediately.")
  );
}

// Serial conections

void processCommand(const char *line) {
  while (*line == ' ' || *line == '\t') {
    line++;
  }

  if (*line == '\0') {
    return;
  }

  if (strcmp(line, "?") == 0 ||
      strcmp(line, "help") == 0) {
    printHelp();
    return;
  }

  // "s" starts a 10-second trial immediately.
  // "s 20" starts a 20-second trial immediately.
  if (line[0] == 's' &&
      (line[1] == '\0' ||
       line[1] == ' ' ||
       line[1] == '\t')) {

    uint32_t durationSeconds = DEFAULT_TRIAL_SECONDS;

    if (line[1] != '\0') {
      unsigned long parsed = 0;

      if (sscanf(line + 1, "%lu", &parsed) != 1) {
        Serial.println(F("Usage: s or s <seconds>, e.g. s 10"));
        return;
      }

      durationSeconds = (uint32_t)parsed;
    }

    startSystemAndTrial(durationSeconds);
    return;
  }

  // Save measured volume in millilitres, e.g. v 950.
  if (line[0] == 'v' &&
      (line[1] == ' ' || line[1] == '\t')) {

    float volumeMl = 0.0f;

    if (sscanf(line + 1, "%f", &volumeMl) == 1) {
      acceptMeasuredVolume(volumeMl);
    } else {
      Serial.println(F("Usage: v <volume_mL>, e.g. v 950"));
    }

    return;
  }

  if (strcmp(line, "summary") == 0) {
    if (trialRunning || awaitingVolume) {
      Serial.println(
        F("Finish the timed trial and enter its volume before the summary.")
      );
    } else {
      printCalibrationSummary();
    }

    return;
  }

  if (strcmp(line, "done") == 0) {
    if (trialRunning) {
      Serial.println(
        F("Cannot finish while a trial is running. Send x to abort it.")
      );
      return;
    }

    if (awaitingVolume) {
      Serial.println(
        F("Enter the current trial volume with v <mL>, or x to discard it.")
      );
      return;
    }

    stopHardwareOutputs();

    Serial.println(
      F("DONE: all pumps OFF, solenoid CLOSED.")
    );

    printCalibrationSummary();
    return;
  }

  if (strcmp(line, "x") == 0 ||
      strcmp(line, "stop") == 0) {

    if (trialRunning) {
      Serial.println(
        F("Current trial aborted and not saved.")
      );
    }

    stopHardwareOutputs();

    trialRunning = false;
    awaitingVolume = false;

    Serial.println(
      F("STOPPED: all pumps OFF, solenoid CLOSED.")
    );

    printCalibrationSummary();
    return;
  }

  Serial.println(F("Unknown command. Type ? for help."));
}

void handleSerialInput() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      inputLine[inputLength] = '\0';

      processCommand(inputLine);

      inputLength = 0;
    } else if (inputLength < sizeof(inputLine) - 1) {
      inputLine[inputLength++] = c;
    } else {
      inputLength = 0;

      Serial.println(
        F("Input line too long; command cleared.")
      );
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(P1_R_EN, OUTPUT);
  pinMode(P1_L_EN, OUTPUT);

  pinMode(P2_R_EN, OUTPUT);
  pinMode(P2_L_EN, OUTPUT);

  pinMode(P3_R_EN, OUTPUT);
  pinMode(P3_L_EN, OUTPUT);

  pinMode(PIN_SOLENOID, OUTPUT);

  initPumpPwm(
    P1_R_PWM, P1_L_PWM, P1_R_CH, P1_L_CH
  );

  initPumpPwm(
    P2_R_PWM, P2_L_PWM, P2_R_CH, P2_L_CH
  );

  initPumpPwm(
    P3_R_PWM, P3_L_PWM, P3_R_CH, P3_L_CH
  );

  // Safe startup state.
  stopHardwareOutputs();

  // GPIO14 must receive a 3.3-V-safe pulse signal.
  pinMode(PIN_FLOW, INPUT);

  attachInterrupt(
    digitalPinToInterrupt(PIN_FLOW),
    flowPulseISR,
    FALLING
  );

  Serial.println();
  Serial.println(F("YF-S401 FLOW TRANSMITTER CALIBRATION"));
  Serial.println(F("ESP32 | Serial Monitor 115200 baud | Newline"));

  Serial.println(
    F("Startup state: pumps OFF, solenoid CLOSED.")
  );

  Serial.println(
    F("Send s to start the system and begin a 10-second trial immediately.")
  );

  printHelp();
}

// MAIN LOOP

void loop() {
  
  if (trialRunning) {
    uint32_t now = millis();

    if ((uint32_t)(now - trialStartMillis) >= trialDurationMillis) {
      finishTrialCapture(now);
    }
  }

  handleSerialInput();
}
