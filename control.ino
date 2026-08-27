/*
 * ============================================================
 *  3-RPS PARALLEL ROBOT — ADMITTANCE CONTROL SYSTEM
 * ============================================================
 *  Model   : Hill-Zajac Muscle Model (Zajac 1989)
 *  Control : Admittance (M=0, First-Order) + CTC + PD
 *  Integrator : Backward Euler
 *
 *  K & B dinamis dari Hill-Zajac:
 *    K = (1-a) * F_MAX * fv(0) * |fl'(l0)|
 *    B = (1-a) * F_MAX * fl(l0) * |fv'(0)|
 *
 *  Retreat trigger (soft):
 *    yank = (F_k - F_{k-1}) / Ts   [backward difference, Ts=0.1s]
 *    |yank| > THRESHOLD_YANK → YANK_PAUSE (400ms, bukan full retreat)
 *
 *  Serial Commands:
 *    S<p1,p2,p3,v1,v2,v3,f1,f2,f3>  Forward trajectory
 *    R<p1,p2,p3,v1,v2,v3,f1,f2,f3>  Retreat trajectory
 *    RETREAT_COMPLETE                Host confirms retreat done
 *    X                               System reset
 *    E                               Emergency stop
 *    1 / 2 / 0                       Manual fwd / rev / stop
 *    K<motor,kp,kd>                  Set outer loop gains
 *    P<motor,kpc,kdc>                Set inner loop gains
 *    ADM,G,<val>                     Set admittance gain
 *    ADMITTANCE_ON / OFF / RESET     Admittance toggle
 *    ADMITTANCE_STATUS               Print admittance state
 * ============================================================
 */

// ============================================================
//  PIN DEFINITIONS
// ============================================================

// Motor PWM pins (RPWM = forward, LPWM = reverse)
const int RPWM1 = 3,  LPWM1 = 5;
const int RPWM2 = 6,  LPWM2 = 9;
const int RPWM3 = 10, LPWM3 = 11;

// Encoder pins
const int ENC1 = 4, ENC2 = 2, ENC3 = 8;

// Current sensor pins
const int CurrSen1 = A0, CurrSen2 = A1, CurrSen3 = A2;

// Load cell (HX711) pins
const int LOADCELL_DOUT_PIN = 12;
const int LOADCELL_SCK_PIN  = 13;

// ============================================================
//  TIMING INTERVALS (ms)
// ============================================================
const int INTERVAL_LOAD        = 10;    // Load cell read (sinkron admittance)
const int INTERVAL_YANK        = 100;   // Yank dF/dt (Ts=0.1s, threshold tetap valid)
const int INTERVAL_ENCODER     = 1;     // Encoder read
const int INTERVAL_VELOCITY    = 10000; // Velocity estimation
const int INTERVAL_CTC         = 100;   // Outer loop (CTC)
const int INTERVAL_PD          = 100;   // Inner loop (PD)
const int INTERVAL_ADMITTANCE  = 10;    // Admittance update
const int INTERVAL_TELEMETRY   = 100;   // Kirim data telemetri ke mini PC
const bool ENABLE_LOCAL_PRINT  = false; // Debug lokal Arduino IDE (false = monitor via mini PC)

// ============================================================
//  MOTOR & DRIVE PARAMETERS
// ============================================================
const float GR = 0.2786;   // Gear ratio
const float kt = 0.0663;   // Motor torque constant (Nm/A)

const int MANUAL_SPEED   = 125;   // PWM for manual mode
const int RETREAT_SPEED  = 150;   // PWM for retreat mode (unused directly)

const float RETREAT_VELOCITY_SCALE = 1.5;   // Velocity multiplier during retreat

// ============================================================
//  CURRENT SENSOR PARAMETERS
// ============================================================
const int   ADC_MAX   = 1023;
const int   N_SAMPLES = 3;
const float CAL_FAC   = 3.40;   // Calibration factor

// ============================================================
//  LOAD CELL
// ============================================================
long  loadCellOffset  = 0;
float latestValidLoad = 0.0;

// ============================================================
//  YANK (dF/dt) — Backward Difference
//  yank = (F_k - F_{k-1}) / Ts,  Ts = 0.1 s
//  |yank| > THRESHOLD_YANK (N kali berturut) → trigger pause
// ============================================================
const float THRESHOLD_YANK     = 70.0;   // unit/s — dinaikkan dari 30 agar tidak terlalu sensitif
const int   YANK_PAUSE_MS      = 400;    // Jeda singkat saat spike (bukan full retreat)
const int   YANK_DEBOUNCE_REQ  = 3;     // Harus N kali berturut-turut sebelum trigger

float yank            = 0.0;
float F_prev          = 0.0;
int   yankDebounceCount = 0;   // Counter debounce yank

// ============================================================
//  WAYPOINT FEEDBACK
//  Arduino eksekusi waypoint → sampai → kirim WAYPOINT_REACHED
//  Mini PC baru boleh kirim waypoint berikutnya
// ============================================================
const float WAYPOINT_TOL       = 3.0;   // mm — error dianggap "sampai"
const long  WAYPOINT_SETTLE_MS = 300;   // ms — harus di dalam toleransi sebelum ACK

bool  waypointActive    = false;   // Sedang tracking waypoint
bool  waypointAckSent   = false;   // Sudah kirim WAYPOINT_REACHED?
long  waypointReachedAt = 0;       // Kapan pertama kali masuk toleransi

// ============================================================
//  LOAD-BASED ADAPTIVE SCALING
// ============================================================
const float LOAD_MAX          = 100.0;
const float LOAD_ALPHA        = 0.3;    // EMA smoothing factor
const float LOAD_SCALE_MIN    = 0.15;
const float LOAD_SCALE_MAX    = 1.0;
const float KP_DAMPING_FACTOR = 0.6;
const bool  ENABLE_ADAPTIVE_MANUAL = true;

float smoothedLoad = 0.0;

// ============================================================
//  ADMITTANCE CONTROL  (M = 0, First-Order)
//  Transfer function : B*Z' + K*Z = F_ext
//  Backward Euler    : Z[n+1] = (B*Z[n] + F_ext*dt) / (B + K*dt)
// ============================================================
bool  admittanceEnabled = true;
float admittanceGain    = 1.0;

float Z_adm      = 0.0;   // Virtual displacement (m)
float Zdot_adm   = 0.0;   // Virtual velocity (m/s)
float Z_adm_prev = 0.0;

float B_adm = 500.0;   // Virtual damping  (N.s/m) — set by Hill-Zajac
float K_adm = 614.1;   // Virtual stiffness (N/m)  — set by Hill-Zajac

// ============================================================
//  HILL-ZAJAC MUSCLE MODEL  (Zajac 1989)
//  Compliance prinsip:
//    F kecil → a kecil → a_inv besar → K & B besar (kaku)
//    F besar → a besar → a_inv kecil → K & B kecil (compliant)
// ============================================================
const float F_MAX    = 6000.0;
const float FL_OPT   = 1.0;
const float FV_ZERO  = 1.0;
const float FL_SLOPE = 4.0;
const float FV_SLOPE = 4.7;
const float K_MIN    = 50.0;    // Lebih compliant sejak awal kontak
const float B_MIN    = 30.0;
const float K_SCALE  = 0.035;   // Naikkan agar K turun lebih cepat saat ada gaya
const float B_SCALE  = 0.025;
const float ACT_POWER = 0.5;    // sqrt(a): aktivasi lebih awal saat gaya kecil

// Hill-Zajac monitoring
float currentActivation = 0.0;
float currentK = 614.1;
float currentB = 500.0;

// ============================================================
//  TRAJECTORY PAUSE  (saat F_ext > FORCE_PAUSE_THRESHOLD)
// ============================================================
const float FORCE_PAUSE_THRESHOLD  = 5.0;   // N (unit) — pause trajectory
const float FORCE_RESUME_THRESHOLD = 2.5;   // N (unit) — resume (hysteresis)

bool  trajectoryPaused = false;
float pausedRefPos1  = 0.0, pausedRefPos2  = 0.0, pausedRefPos3  = 0.0;
float pausedRefVelo1 = 0.0, pausedRefVelo2 = 0.0, pausedRefVelo3 = 0.0;
float pausedRefFc1   = 0.0, pausedRefFc2   = 0.0, pausedRefFc3   = 0.0;

// ============================================================
//  SYSTEM STATE
// ============================================================
int  operatingMode   = 0;   // 0=idle, 1=forward, 2=retreat
int  manualCommand   = 0;   // 0=stop, 1=fwd, 2=rev
int  manipulatorState = 0;  // 0=running, 1=paused/retreat

bool retreatHasBeenTriggered = false;
bool retreatRequestSent      = false;
long yankPauseUntil          = 0;   // Soft pause setelah spike yank

// Helper: reset semua state waypoint
void resetWaypointState() {
    waypointActive    = false;
    waypointAckSent   = false;
    waypointReachedAt = 0;
}

String receivedData = "";

// ============================================================
//  MOTOR VARIABLES — Reference & Actual
// ============================================================

// --- Outer loop gains (CTC) ---
float kp1 = 110.0, kd1 = 0.1;
float kp2 = 142.0, kd2 = 0.6;
float kp3 = 150.0, kd3 = 0.3;

// --- Inner loop gains (PD current) ---
float kpc1 = 30.0, kdc1 = 0.1;
float kpc2 = 33.0, kdc2 = 0.1;
float kpc3 = 38.0, kdc3 = 0.1;

// --- Reference signals ---
float refPos1 = 0.0, refVelo1 = 0.0, refFc1 = 0.0;
float refPos2 = 0.0, refVelo2 = 0.0, refFc2 = 0.0;
float refPos3 = 0.0, refVelo3 = 0.0, refFc3 = 0.0;

// --- Actual signals ---
float ActPos1 = 0.0, ActVelo1 = 0.0, ActCurrent1 = 0.0;
float ActPos2 = 0.0, ActVelo2 = 0.0, ActCurrent2 = 0.0;
float ActPos3 = 0.0, ActVelo3 = 0.0, ActCurrent3 = 0.0;

// --- Encoder counters ---
int position1 = 0, prevState1 = 0;
int position2 = 0, prevState2 = 0;
int position3 = 0, prevState3 = 0;

// --- Control signals ---
float refCurrent1 = 0.0, refCurrent2 = 0.0, refCurrent3 = 0.0;
float controlValue1 = 0.0, controlValue2 = 0.0, controlValue3 = 0.0;
float ErrPos1 = 0.0, ErrPos2 = 0.0, ErrPos3 = 0.0;

// --- Error history (for PD derivative) ---
float prevError1 = 0.0, prevError2 = 0.0, prevError3 = 0.0;
float prevPos1   = 0.0, prevPos2   = 0.0, prevPos3   = 0.0;

// ============================================================
//  TIMING VARIABLES
// ============================================================
long lastLoadTime       = 0;
long lastYankTime       = 0;
long lastEncTime        = 0;
long lastVeloTime       = 0;
long lastCTCCalcTime    = 0;
long lastPDCalcTime     = 0;
long lastAdmittanceTime = 0;
long lastTelemTime      = 0;
long lastSerialRxTime   = 0;   // Watchdog: last time any serial byte was received

// ============================================================
//  WATCHDOG — Host Disconnect Detection
//  Leonardo USB CDC: Serial == false saat host disconnect
//  Jika tidak ada data masuk > SERIAL_TIMEOUT_MS → emergency stop
// ============================================================
const long SERIAL_TIMEOUT_MS = 3000;   // 3 detik tanpa data = host mati
bool hostWasConnected = false;          // Track state untuk edge detection

// ============================================================
//  MODE HELPERS
// ============================================================

bool isAutoMotion() {
    return operatingMode == 1 || operatingMode == 2;
}

bool isAdmittanceActive() {
    return admittanceEnabled && isAutoMotion();
}

// ============================================================
//  HELPER: BASIC CONTROL MATH
// ============================================================

float computeError(float ref, float act) {
    return ref - act;
}

// CTC outer loop: outputs reference current
float CTC(float posErr, float kp, float veloErr, float kd,
          float forceFeedforward, float gearRatio, float motorKt) {
    return (posErr * kp + veloErr * kd + forceFeedforward) * gearRatio * motorKt;
}

// PD inner loop: outputs PWM control value
float PD(float err, float kpc, float errDeriv, float kdc) {
    return err * kpc + errDeriv * kdc;
}

// ============================================================
//  LOAD-BASED ADAPTIVE HELPERS
// ============================================================

// Returns PWM scale factor [LOAD_SCALE_MIN, LOAD_SCALE_MAX]
float getLoadScaling() {
    smoothedLoad = LOAD_ALPHA * latestValidLoad + (1.0 - LOAD_ALPHA) * smoothedLoad;
    float ratio  = constrain(smoothedLoad / LOAD_MAX, 0.0, 1.0);
    return LOAD_SCALE_MAX - ratio * (LOAD_SCALE_MAX - LOAD_SCALE_MIN);
}

// Returns Kp scaled down under heavy load
float getAdaptiveKp(float baseKp) {
    float ratio = constrain(smoothedLoad / LOAD_MAX, 0.0, 1.0);
    return baseKp * (1.0 - ratio * KP_DAMPING_FACTOR);
}

// ============================================================
//  YANK  — Backward Difference
// ============================================================
void updateYank(float F_curr) {
    const float Ts = INTERVAL_YANK / 1000.0;   // 0.1 s — tetap untuk threshold empiris
    yank   = (F_curr - F_prev) / Ts;
    F_prev = F_curr;
}

// ============================================================
//  HILL-ZAJAC MUSCLE MODEL
// ============================================================
void updateMuscleAdmittance(float F_external) {
    float a_lin = constrain(F_external / LOAD_MAX, 0.0, 1.0);
    float a     = pow(a_lin, ACT_POWER);   // Lebih compliant sejak gaya kecil
    float a_inv = 1.0 - a;

    K_adm = max(a_inv * F_MAX * FV_ZERO * FL_SLOPE * K_SCALE, K_MIN);
    B_adm = max(a_inv * F_MAX * FL_OPT  * FV_SLOPE * B_SCALE, B_MIN);

    currentActivation = a;
    currentK = K_adm;
    currentB = B_adm;
}

// ============================================================
//  ADMITTANCE CONTROL  (M=0, Backward Euler)
// ============================================================
void updateAdmittanceControl(float F_external, float dt) {
    float F_scaled = F_external * admittanceGain;
    float denom    = B_adm + K_adm * dt;
    if (denom < 1e-6) denom = 1e-6;

    float Z_new  = (B_adm * Z_adm_prev + F_scaled * dt) / denom;
    Zdot_adm     = (Z_new - Z_adm_prev) / dt;
    Z_adm        = Z_new;
    Z_adm_prev   = Z_new;
}

void resetAdmittance() {
    Z_adm = 0.0; Z_adm_prev = 0.0;
    Zdot_adm = 0.0;
    K_adm = 614.1; B_adm = 500.0;
    currentK = 614.1; currentB = 500.0;
    currentActivation = 0.0;
    yank = 0.0; F_prev = 0.0;
    yankPauseUntil = 0;
}

// Baca load cell jika data siap (non-blocking)
bool readLoadCellIfReady(float& loadOut) {
    if (digitalRead(LOADCELL_DOUT_PIN) != LOW) return false;
    long raw = readHX711() - loadCellOffset;
    loadOut = constrain(raw / 10000.0, 0.0, 100.0);
    latestValidLoad = loadOut;
    return true;
}

// ============================================================
//  SENSOR READING
// ============================================================

// HX711 raw read (24-bit, 2's complement)
long readHX711() {
    long result = 0;
    while (digitalRead(LOADCELL_DOUT_PIN));   // Wait until ready

    for (int i = 0; i < 24; i++) {
        digitalWrite(LOADCELL_SCK_PIN, HIGH);
        delayMicroseconds(1);
        result = (result << 1) | digitalRead(LOADCELL_DOUT_PIN);
        digitalWrite(LOADCELL_SCK_PIN, LOW);
        delayMicroseconds(1);
    }
    // Pulse 25th clock to set gain = 128
    digitalWrite(LOADCELL_SCK_PIN, HIGH); delayMicroseconds(1);
    digitalWrite(LOADCELL_SCK_PIN, LOW);  delayMicroseconds(1);

    if (result & 0x800000) result |= ~0xFFFFFF;   // Sign extend
    return result;
}

// Averaged ADC read for each current sensor
float readCurrentSensor(int pin) {
    float val = 0;
    for (int i = 0; i < N_SAMPLES; i++) { val += analogRead(pin); delay(1); }
    return val / ADC_MAX / N_SAMPLES;
}

// ============================================================
//  COMMAND PARSING
// ============================================================

void parseTrajectoryCommand(String data, bool isRetreat) {
    // Strip prefix 'S' or 'R'
    data.replace("S", "");
    data.replace("R", "");

    // Parse 9 comma-separated floats: p1,p2,p3,v1,v2,v3,f1,f2,f3
    float vals[9] = {0};
    for (int i = 0; i < 9; i++) {
        int comma = data.indexOf(',');
        if (comma < 0) { vals[i] = data.toFloat(); break; }
        vals[i] = data.substring(0, comma).toFloat();
        data    = data.substring(comma + 1);
    }

    if (trajectoryPaused) return;   // Ignore new commands while paused

    refPos1 = vals[0]; refPos2 = vals[1]; refPos3 = vals[2];
    refVelo1 = vals[3]; refVelo2 = vals[4]; refVelo3 = vals[5];
    refFc1   = vals[6]; refFc2   = vals[7]; refFc3   = vals[8];

    if (isRetreat) {
        refVelo1 *= RETREAT_VELOCITY_SCALE;
        refVelo2 *= RETREAT_VELOCITY_SCALE;
        refVelo3 *= RETREAT_VELOCITY_SCALE;
    }

    // Reset waypoint feedback state untuk titik baru
    waypointActive    = true;
    waypointAckSent   = false;
    waypointReachedAt = 0;
}

void parseOuterLoopGains(String data) {
    // Format: K<motor>,<kp>,<kd>
    data.replace("K", "");
    int c1 = data.indexOf(',');
    int motor = data.substring(0, c1).toInt();
    data = data.substring(c1 + 1);
    int c2 = data.indexOf(',');
    float newKp = data.substring(0, c2).toFloat();
    float newKd = data.substring(c2 + 1).toFloat();

    if      (motor == 1) { kp1 = newKp; kd1 = newKd; }
    else if (motor == 2) { kp2 = newKp; kd2 = newKd; }
    else if (motor == 3) { kp3 = newKp; kd3 = newKd; }

    Serial.print("Motor "); Serial.print(motor);
    Serial.print(" Kp="); Serial.print(newKp);
    Serial.print(" Kd="); Serial.println(newKd);
}

void parseInnerLoopGains(String data) {
    // Format: P<motor>,<kpc>,<kdc>
    data.replace("P", "");
    int c1 = data.indexOf(',');
    int motor = data.substring(0, c1).toInt();
    data = data.substring(c1 + 1);
    int c2 = data.indexOf(',');
    float newKpc = data.substring(0, c2).toFloat();
    float newKdc = data.substring(c2 + 1).toFloat();

    if      (motor == 1) { kpc1 = newKpc; kdc1 = newKdc; }
    else if (motor == 2) { kpc2 = newKpc; kdc2 = newKdc; }
    else if (motor == 3) { kpc3 = newKpc; kdc3 = newKdc; }

    Serial.print("Motor "); Serial.print(motor);
    Serial.print(" Kpc="); Serial.print(newKpc);
    Serial.print(" Kdc="); Serial.println(newKdc);
}

void parseAdmittanceParams(String data) {
    // Format: ADM,G,<val>   (only gain is tunable; K & B = Hill-Zajac)
    data.replace("ADM", "");
    int c = data.indexOf(',');
    String param = data.substring(0, c);
    float  val   = data.substring(c + 1).toFloat();

    if (param == "G") {
        admittanceGain = val;
        Serial.print("Admittance gain = "); Serial.println(admittanceGain);
    } else {
        Serial.println("ERR: Unknown ADM param. Use ADM,G,<val>");
        Serial.println("     K & B dikontrol Hill-Zajac via K_SCALE/B_SCALE");
    }
}

// ============================================================
//  SYSTEM CONTROL
// ============================================================

void stopAllMotors() {
    analogWrite(RPWM1, 0); analogWrite(LPWM1, 0);
    analogWrite(RPWM2, 0); analogWrite(LPWM2, 0);
    analogWrite(RPWM3, 0); analogWrite(LPWM3, 0);
}

void resetSystem() {
    operatingMode            = 0;
    manualCommand            = 0;
    manipulatorState         = 0;
    retreatHasBeenTriggered  = false;
    retreatRequestSent       = false;
    trajectoryPaused         = false;
    yankPauseUntil           = 0;
    yankDebounceCount        = 0;

    resetWaypointState();
    stopAllMotors();

    // Re-tare load cell
    loadCellOffset  = readHX711();
    smoothedLoad    = 0.0;
    latestValidLoad = 0.0;

    // Reset encoder & control states
    position1 = 0; position2 = 0; position3 = 0;
    ActPos1 = 0.0; ActPos2 = 0.0; ActPos3 = 0.0;
    prevPos1 = 0.0; prevPos2 = 0.0; prevPos3 = 0.0;
    ActVelo1 = 0.0; ActVelo2 = 0.0; ActVelo3 = 0.0;
    ErrPos1  = 0.0; ErrPos2  = 0.0; ErrPos3  = 0.0;
    prevError1 = 0.0; prevError2 = 0.0; prevError3 = 0.0;

    resetAdmittance();

    Serial.println("System Reset OK");
}

void emergencyStop() {
    operatingMode           = 0;
    manualCommand           = 0;
    retreatHasBeenTriggered = false;
    retreatRequestSent      = false;
    stopAllMotors();
    Serial.println("EMERGENCY_STOP");
}

// ============================================================
//  MOTOR CONTROL — Encoder, Velocity, CTC, PD, Apply
// ============================================================

void updateEncoders() {
    // Motor 1
    int s1 = digitalRead(ENC1);
    if (s1 > prevState1) {
        if      (controlValue1 > 0) position1++;
        else if (controlValue1 < 0) position1--;
    }
    ActPos1   = position1 * 0.245;
    prevState1 = s1;

    // Motor 2
    int s2 = digitalRead(ENC2);
    if (s2 > prevState2) {
        if      (controlValue2 > 0) position2++;
        else if (controlValue2 < 0) position2--;
    }
    ActPos2    = position2 * 0.245;
    prevState2 = s2;

    // Motor 3
    int s3 = digitalRead(ENC3);
    if (s3 > prevState3) {
        if      (controlValue3 > 0) position3++;
        else if (controlValue3 < 0) position3--;
    }
    ActPos3    = position3 * 0.245;
    prevState3 = s3;
}

void updateVelocities() {
    // Ts = veloInterval / 1e6 s → divider = 10 (for mm/s consistency)
    ActVelo1 = (ActPos1 - prevPos1) / 10.0;
    ActVelo2 = (ActPos2 - prevPos2) / 10.0;
    ActVelo3 = (ActPos3 - prevPos3) / 10.0;
    prevPos1 = ActPos1;
    prevPos2 = ActPos2;
    prevPos3 = ActPos3;
}

void calculateCTC() {
    ErrPos1 = computeError(refPos1, ActPos1);
    ErrPos2 = computeError(refPos2, ActPos2);
    ErrPos3 = computeError(refPos3, ActPos3);

    refCurrent1 = CTC(ErrPos1, getAdaptiveKp(kp1), computeError(refVelo1, ActVelo1), kd1, refFc1, GR, kt);
    refCurrent2 = CTC(ErrPos2, getAdaptiveKp(kp2), computeError(refVelo2, ActVelo2), kd2, refFc2, GR, kt);
    refCurrent3 = CTC(ErrPos3, getAdaptiveKp(kp3), computeError(refVelo3, ActVelo3), kd3, refFc3, GR, kt);
}

void calculateCTCWithAdmittance() {
    // Modify reference position & velocity by admittance displacement
    float rp1 = refPos1 - (Z_adm * 1000.0);   // convert m → mm
    float rp2 = refPos2 - (Z_adm * 1000.0);
    float rp3 = refPos3 - (Z_adm * 1000.0);

    float Zdot_mm = Zdot_adm * 1000.0;   // m/s → mm/s (sesuai satuan refVelo)
    float rv1 = refVelo1 - Zdot_mm;
    float rv2 = refVelo2 - Zdot_mm;
    float rv3 = refVelo3 - Zdot_mm;

    ErrPos1 = computeError(rp1, ActPos1);
    ErrPos2 = computeError(rp2, ActPos2);
    ErrPos3 = computeError(rp3, ActPos3);

    refCurrent1 = CTC(ErrPos1, getAdaptiveKp(kp1), computeError(rv1, ActVelo1), kd1, refFc1, GR, kt);
    refCurrent2 = CTC(ErrPos2, getAdaptiveKp(kp2), computeError(rv2, ActVelo2), kd2, refFc2, GR, kt);
    refCurrent3 = CTC(ErrPos3, getAdaptiveKp(kp3), computeError(rv3, ActVelo3), kd3, refFc3, GR, kt);
}

void calculatePD() {
    float err1 = computeError(refCurrent1, ActCurrent1);
    float err2 = computeError(refCurrent2, ActCurrent2);
    float err3 = computeError(refCurrent3, ActCurrent3);

    const float Ts_pd = INTERVAL_PD / 1000.0;   // 0.1 s

    controlValue1 = PD(err1, kpc1, (err1 - prevError1) / Ts_pd, kdc1);
    controlValue2 = PD(err2, kpc2, (err2 - prevError2) / Ts_pd, kdc2);
    controlValue3 = PD(err3, kpc3, (err3 - prevError3) / Ts_pd, kdc3);

    prevError1 = err1;
    prevError2 = err2;
    prevError3 = err3;

    // Apply load-based scaling in auto mode (forward + retreat)
    float scale = isAutoMotion() ? getLoadScaling() : 1.0;
    controlValue1 = constrain(controlValue1 * scale, -255, 255);
    controlValue2 = constrain(controlValue2 * scale, -255, 255);
    controlValue3 = constrain(controlValue3 * scale, -255, 255);
}

void applyMotorControl() {
    // Motor 1
    if      (ErrPos1 > 0) { analogWrite(RPWM1, abs(controlValue1)); analogWrite(LPWM1, 0); }
    else if (ErrPos1 < 0) { analogWrite(RPWM1, 0); analogWrite(LPWM1, abs(controlValue1)); }
    else                  { analogWrite(RPWM1, 0); analogWrite(LPWM1, 0); }

    // Motor 2
    if      (ErrPos2 > 0) { analogWrite(RPWM2, abs(controlValue2)); analogWrite(LPWM2, 0); }
    else if (ErrPos2 < 0) { analogWrite(RPWM2, 0); analogWrite(LPWM2, abs(controlValue2)); }
    else                  { analogWrite(RPWM2, 0); analogWrite(LPWM2, 0); }

    // Motor 3
    if      (ErrPos3 > 0) { analogWrite(RPWM3, abs(controlValue3)); analogWrite(LPWM3, 0); }
    else if (ErrPos3 < 0) { analogWrite(RPWM3, 0); analogWrite(LPWM3, abs(controlValue3)); }
    else                  { analogWrite(RPWM3, 0); analogWrite(LPWM3, 0); }
}

void sendTelemetry() {
    Serial.print(F("s:")); Serial.print(manipulatorState == 0 ? "run" : "pause");
    Serial.print(F(",m:")); Serial.print(operatingMode == 1 ? "fwd" : "ret");
    Serial.print(F(",load:")); Serial.print(latestValidLoad, 2);
    Serial.print(F(",yank:")); Serial.print(yank, 2);
    Serial.print(F(",ythr:")); Serial.print(THRESHOLD_YANK, 1);
    Serial.print(F(",sc:"));  Serial.print(getLoadScaling(), 2);
    Serial.print(F(",p1:")); Serial.print(ActPos1, 2);
    Serial.print(F(",p2:")); Serial.print(ActPos2, 2);
    Serial.print(F(",p3:")); Serial.print(ActPos3, 2);
    Serial.print(F(",rp1:")); Serial.print(refPos1, 2);
    Serial.print(F(",rp2:")); Serial.print(refPos2, 2);
    Serial.print(F(",rp3:")); Serial.print(refPos3, 2);
    Serial.print(F(",ep1:")); Serial.print(ErrPos1, 2);
    Serial.print(F(",ep2:")); Serial.print(ErrPos2, 2);
    Serial.print(F(",ep3:")); Serial.print(ErrPos3, 2);
    Serial.print(F(",v1:")); Serial.print(ActVelo1, 2);
    Serial.print(F(",v2:")); Serial.print(ActVelo2, 2);
    Serial.print(F(",v3:")); Serial.print(ActVelo3, 2);
    Serial.print(F(",c1:")); Serial.print(ActCurrent1, 2);
    Serial.print(F(",c2:")); Serial.print(ActCurrent2, 2);
    Serial.print(F(",c3:")); Serial.print(ActCurrent3, 2);
    Serial.print(F(",pwm1:")); Serial.print(controlValue1, 0);
    Serial.print(F(",pwm2:")); Serial.print(controlValue2, 0);
    Serial.print(F(",pwm3:")); Serial.print(controlValue3, 0);
    if (isAdmittanceActive()) {
        Serial.print(F(",act:"));  Serial.print(currentActivation, 3);
        Serial.print(F(",K:"));    Serial.print(currentK, 2);
        Serial.print(F(",B:"));    Serial.print(currentB, 2);
        Serial.print(F(",tau:"));  Serial.print(currentB / currentK, 4);
        Serial.print(F(",Z:"));    Serial.print(Z_adm * 1000, 3);
        Serial.print(F(",Zd:"));   Serial.print(Zdot_adm * 1000, 3);
        Serial.print(F(",tpause:")); Serial.print(trajectoryPaused ? 1 : 0);
    }
    Serial.println();
}

void localPrint(const __FlashStringHelper* msg) {
    if (ENABLE_LOCAL_PRINT) Serial.println(msg);
}

void localPrintLine(const String& msg) {
    if (ENABLE_LOCAL_PRINT) Serial.println(msg);
}

void manualModeControl() {
    int speed = ENABLE_ADAPTIVE_MANUAL
                ? (int)(MANUAL_SPEED * getLoadScaling())
                : MANUAL_SPEED;

    if      (manualCommand == 1) {
        analogWrite(RPWM1, speed); analogWrite(LPWM1, 0);
        analogWrite(RPWM2, speed); analogWrite(LPWM2, 0);
        analogWrite(RPWM3, speed); analogWrite(LPWM3, 0);
    }
    else if (manualCommand == 2) {
        analogWrite(RPWM1, 0); analogWrite(LPWM1, speed);
        analogWrite(RPWM2, 0); analogWrite(LPWM2, speed);
        analogWrite(RPWM3, 0); analogWrite(LPWM3, speed);
    }
    else { stopAllMotors(); }
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
    Serial.begin(115200);

    // Motor PWM
    pinMode(RPWM1, OUTPUT); pinMode(LPWM1, OUTPUT);
    pinMode(RPWM2, OUTPUT); pinMode(LPWM2, OUTPUT);
    pinMode(RPWM3, OUTPUT); pinMode(LPWM3, OUTPUT);

    // Encoders & sensors
    pinMode(ENC1, INPUT); pinMode(ENC2, INPUT); pinMode(ENC3, INPUT);
    pinMode(CurrSen1, INPUT); pinMode(CurrSen2, INPUT); pinMode(CurrSen3, INPUT);

    // Load cell
    pinMode(LOADCELL_DOUT_PIN, INPUT);
    pinMode(LOADCELL_SCK_PIN,  OUTPUT);

    while (!Serial) { ; }

    loadCellOffset = readHX711();

    Serial.println(F("READY"));
    if (ENABLE_LOCAL_PRINT) {
        Serial.println(F("=============================================="));
        Serial.println(F("  3-RPS Parallel Robot Control System"));
        Serial.println(F("  Admittance aktif: forward + retreat"));
        Serial.println(F("  Telemetri -> mini PC console"));
        Serial.println(F("=============================================="));
    }
}

// ============================================================
//  MAIN LOOP
// ============================================================
void loop() {
    long now = millis();

    // ----------------------------------------------------------
    //  HOST DISCONNECT DETECTION (Leonardo USB CDC)
    //  Serial == false  →  USB host (mini PC) sudah disconnect
    // ----------------------------------------------------------
    if (!Serial) {
        // USB host terputus: stop semua motor langsung
        if (hostWasConnected) {
            stopAllMotors();
            operatingMode  = 0;
            manualCommand  = 0;
            hostWasConnected = false;
        }
        return;   // Tunggu sampai host connect kembali
    }
    hostWasConnected = true;

    // ----------------------------------------------------------
    //  WATCHDOG — Timeout jika host tiba-tiba tidak kirim data
    //  (fallback kalau USB tidak terdeteksi disconnect)
    // ----------------------------------------------------------
    if (lastSerialRxTime > 0 &&
        (now - lastSerialRxTime) > SERIAL_TIMEOUT_MS &&
        (operatingMode != 0 || manualCommand != 0)) {
        stopAllMotors();
        operatingMode = 0;
        manualCommand = 0;
        lastSerialRxTime = now;   // Reset agar tidak trigger berulang
    }

    // ----------------------------------------------------------
    //  SERIAL COMMAND PARSING
    // ----------------------------------------------------------
    if (Serial.available() > 0) {
        lastSerialRxTime = now;   // Update watchdog setiap ada data masuk
        while (Serial.available() > 0) {
            char c = Serial.read();
            receivedData += c;

            if (c == '\n') {
                receivedData.trim();

                // --- Trajectory commands ---
                if (receivedData.startsWith("S")) {
                    operatingMode           = 1;
                    retreatHasBeenTriggered = false;
                    retreatRequestSent      = false;
                    manipulatorState        = 0;
                    parseTrajectoryCommand(receivedData, false);
                }
                else if (receivedData.startsWith("R") && receivedData.indexOf(',') > 0) {
                    operatingMode    = 2;
                    manipulatorState = 0;
                    parseTrajectoryCommand(receivedData, true);
                }
                else if (receivedData == "RETREAT_COMPLETE") {
                    operatingMode           = 0;
                    manualCommand           = 0;
                    retreatHasBeenTriggered = false;
                    retreatRequestSent      = false;
                    stopAllMotors();
                    Serial.println("ACK_RETREAT_COMPLETE");
                }

                // --- System commands ---
                else if (receivedData.startsWith("X")) { resetSystem(); }
                else if (receivedData.startsWith("E")) { emergencyStop(); }

                // --- Manual mode ---
                else if (receivedData == "1") {
                    operatingMode = 0; manualCommand = 1;
                    retreatHasBeenTriggered = false; retreatRequestSent = false;
                }
                else if (receivedData == "2") {
                    operatingMode = 0; manualCommand = 2;
                    retreatHasBeenTriggered = false; retreatRequestSent = false;
                }
                else if (receivedData == "0") {
                    operatingMode = 0; manualCommand = 0;
                }

                // --- Gain tuning ---
                else if (receivedData.startsWith("K")) {
                    operatingMode = 0; manualCommand = 0;
                    parseOuterLoopGains(receivedData);
                }
                else if (receivedData.startsWith("P")) {
                    operatingMode = 0; manualCommand = 0;
                    parseInnerLoopGains(receivedData);
                }

                // --- Admittance commands ---
                else if (receivedData == "ADMITTANCE_ON") {
                    admittanceEnabled = true;
                    Serial.println("Admittance ON");
                }
                else if (receivedData == "ADMITTANCE_OFF") {
                    admittanceEnabled = false;
                    Serial.println("Admittance OFF");
                }
                else if (receivedData == "ADMITTANCE_RESET") {
                    resetAdmittance();
                    Serial.println("Admittance RESET");
                }
                else if (receivedData.startsWith("ADM") && receivedData.indexOf(',') > 0) {
                    parseAdmittanceParams(receivedData);
                }
                else if (receivedData == "ADMITTANCE_STATUS") {
                    Serial.println(F("\n=== Admittance Status ==="));
                    Serial.print(F("Enabled    : ")); Serial.println(admittanceEnabled ? "YES" : "NO");
                    Serial.print(F("Activation : ")); Serial.println(currentActivation, 4);
                    Serial.print(F("K_adm      : ")); Serial.print(currentK, 2); Serial.println(F(" N/m"));
                    Serial.print(F("B_adm      : ")); Serial.print(currentB, 2); Serial.println(F(" N.s/m"));
                    Serial.print(F("tau (B/K)  : ")); Serial.print(currentB / currentK, 4); Serial.println(F(" s"));
                    Serial.print(F("Z_adm      : ")); Serial.print(Z_adm * 1000, 4); Serial.println(F(" mm"));
                    Serial.print(F("Zdot_adm   : ")); Serial.print(Zdot_adm * 1000, 4); Serial.println(F(" mm/s"));
                    Serial.print(F("F_ext      : ")); Serial.print(latestValidLoad, 2); Serial.println(F(" unit"));
                    Serial.print(F("Yank       : ")); Serial.print(yank, 2); Serial.println(F(" unit/s"));
                    Serial.print(F("Yank thresh: ")); Serial.print(THRESHOLD_YANK, 1); Serial.println(F(" unit/s"));
                    Serial.println(F("=========================\n"));
                }

                else {
                    Serial.print(F("ERR: Unknown command -> ")); Serial.println(receivedData);
                }

                receivedData = "";
            }
        }
    }

    // ----------------------------------------------------------
    //  AUTO MODE (FORWARD = 1 | RETREAT = 2)
    // ----------------------------------------------------------
    if (operatingMode == 1 || operatingMode == 2) {

        // --- Load cell (forward + retreat, 10ms) ---
        if (isAutoMotion() && now - lastLoadTime >= INTERVAL_LOAD) {
            float load = latestValidLoad;
            if (readLoadCellIfReady(load)) {
                lastLoadTime = now;
            }
        }

        // --- Yank detection (forward + retreat, 100ms) ---
        if (isAutoMotion() && now - lastYankTime >= INTERVAL_YANK) {
            updateYank(latestValidLoad);

            // Debounce: harus N kali berturut-turut melewati threshold
            if (abs(yank) > THRESHOLD_YANK) {
                yankDebounceCount++;
                if (yankDebounceCount >= YANK_DEBOUNCE_REQ) {
                    yankPauseUntil    = now + YANK_PAUSE_MS;
                    yankDebounceCount = 0;
                    waypointReachedAt = 0;   // Paksa settle ulang setelah pause
                    Serial.print(F("YANK_PAUSE yank="));
                    Serial.print(yank, 2);
                    Serial.println(F(" unit/s"));
                }
            } else {
                yankDebounceCount = 0;   // Reset jika tidak berturut-turut
            }

            lastYankTime = now;
        }

        // Tentukan manipulatorState: soft yank pause ATAU full retreat
        if (isAutoMotion()) {
            if (retreatHasBeenTriggered) {
                manipulatorState = 1;
            } else if (yankPauseUntil > 0 && now < yankPauseUntil) {
                manipulatorState = 1;
            } else {
                manipulatorState = 0;
                yankPauseUntil = 0;
            }
        }

        // --- Admittance update (forward + retreat) ---
        if (isAdmittanceActive() &&
            now - lastAdmittanceTime >= INTERVAL_ADMITTANCE) {

            float load = latestValidLoad;
            readLoadCellIfReady(load);   // Update load setiap 10ms jika tersedia

            float dt = INTERVAL_ADMITTANCE / 1000.0;
            updateMuscleAdmittance(load);
            updateAdmittanceControl(load, dt);

            // Decay Z_adm saat gaya turun agar resume lebih cepat
            if (load <= FORCE_RESUME_THRESHOLD) {
                Z_adm      *= 0.92;
                Z_adm_prev  = Z_adm;
                Zdot_adm    = 0.0;
            }

            // Trajectory pause/resume dengan hysteresis
            if (load > FORCE_PAUSE_THRESHOLD && !trajectoryPaused) {
                trajectoryPaused = true;
                pausedRefPos1  = refPos1;  pausedRefPos2  = refPos2;  pausedRefPos3  = refPos3;
                pausedRefVelo1 = refVelo1; pausedRefVelo2 = refVelo2; pausedRefVelo3 = refVelo3;
                pausedRefFc1   = refFc1;   pausedRefFc2   = refFc2;   pausedRefFc3   = refFc3;
                Serial.println(F("PAUSE_TRAJECTORY"));
            }
            else if (load <= FORCE_RESUME_THRESHOLD && trajectoryPaused) {
                trajectoryPaused  = false;
                waypointReachedAt = 0;   // Harus settle ulang sebelum kirim ACK
                waypointAckSent   = false;
                refPos1  = pausedRefPos1;  refPos2  = pausedRefPos2;  refPos3  = pausedRefPos3;
                refVelo1 = pausedRefVelo1; refVelo2 = pausedRefVelo2; refVelo3 = pausedRefVelo3;
                refFc1   = pausedRefFc1;   refFc2   = pausedRefFc2;   refFc3   = pausedRefFc3;
                Serial.println(F("RESUME_TRAJECTORY"));
            }

            lastAdmittanceTime = now;
        }

        // --- Encoder (only when running) ---
        if (manipulatorState == 0 && now - lastEncTime >= INTERVAL_ENCODER) {
            updateEncoders();
            lastEncTime = now;
        }

        // --- Control loops (only when running) ---
        if (manipulatorState == 0) {

            if (now - lastVeloTime >= INTERVAL_VELOCITY) {
                updateVelocities();
                lastVeloTime = now;
            }

            if (now - lastCTCCalcTime >= INTERVAL_CTC) {
                isAdmittanceActive()
                    ? calculateCTCWithAdmittance()
                    : calculateCTC();
                lastCTCCalcTime = now;
            }

            if (now - lastPDCalcTime >= INTERVAL_PD) {
                calculatePD();
                lastPDCalcTime = now;
            }
        }

        // --- Waypoint reached check ---
        // Cek apakah semua motor sudah dalam toleransi posisi
        // Hanya kirim ACK jika: tidak sedang pause, manipulatorState running,
        // dan belum kirim ACK untuk waypoint ini
        if (waypointActive && !waypointAckSent
            && !trajectoryPaused && manipulatorState == 0) {

            bool inTol = (abs(ErrPos1) < WAYPOINT_TOL)
                      && (abs(ErrPos2) < WAYPOINT_TOL)
                      && (abs(ErrPos3) < WAYPOINT_TOL);

            if (inTol && waypointReachedAt == 0) {
                waypointReachedAt = now;          // Mulai settle timer
            } else if (!inTol) {
                waypointReachedAt = 0;            // Keluar toleransi, reset timer
            }

            if (waypointReachedAt > 0
                && (now - waypointReachedAt) >= WAYPOINT_SETTLE_MS) {
                Serial.println(F("WAYPOINT_REACHED"));
                waypointAckSent = true;
            }
        }

        // --- Motor output ---
        (manipulatorState == 0) ? applyMotorControl() : stopAllMotors();

        // --- Telemetry ke mini PC (bukan Serial Monitor Arduino) ---
        if (now - lastTelemTime >= INTERVAL_TELEMETRY) {
            sendTelemetry();
            lastTelemTime = now;
        }
    }

    // ----------------------------------------------------------
    //  MANUAL MODE
    // ----------------------------------------------------------
    else {
        if (ENABLE_ADAPTIVE_MANUAL && now - lastLoadTime >= INTERVAL_LOAD) {
            float load = latestValidLoad;
            if (readLoadCellIfReady(load)) {
                updateYank(load);
                lastLoadTime = now;
            }
        }
        manualModeControl();
    }
}
