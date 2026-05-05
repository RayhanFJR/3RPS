//==================================================================
// ARDUINO MOTOR CONTROL SYSTEM - ADMITTANCE CONTROL VERSION
// Features:
// - Admittance Control dengan K dan B DINAMIS (Hill-Zajac Muscle Model)
// - M = 0: Z(s) = F_ext / (B(t)*s + K(t))  ← First-Order System
// - Backward Euler integration untuk stabilitas
// - K dan B berubah real-time berdasarkan load cell
// - CTC with feedforward compensation
// - Load-based adaptive scaling
// - Trajectory-based retreat communication
// - Manual/Auto modes with safety system
// - Retreat trigger via gradient load cell (Central Difference)
//
// Referensi: Zajac, F.E. (1989). Muscle and tendon: properties, models,
//            scaling, and application to biomechanics and motor control.
//            Critical Reviews in Biomedical Engineering, 17(4), 359-411.
//==================================================================

//==================================================================
// CONSTANTS & CONFIGURATION
//==================================================================

// Motor Control Speed
const int MANUAL_SPEED  = 125;
const int RETREAT_SPEED = 150;

// Load Cell Configuration
const int LOADCELL_DOUT_PIN = 12;
const int LOADCELL_SCK_PIN  = 13;
long  loadCellOffset    = 0;
float latestValidLoad   = 0.0;

// ---------------------------------------------------------------
// Threshold 1 DIHAPUS → admittance yang handle softening
// Threshold 2 DIGANTI ROC (lihat bagian ROC di bawah)
// ---------------------------------------------------------------

// ---------------------------------------------------------------
// Rate of Change (ROC) via Central Difference — pengganti threshold2
//
//   gradient[n] = (F[n+1] - F[n-1]) / (2 * dt)
//
//   Karena real-time (F[n+1] belum ada saat sample n tiba),
//   implementasi pakai 3-sample ring buffer:
//     buf[0]=F[n-1], buf[1]=F[n], buf[2]=F[n+1]
//   Gradient dihitung 1 sample terlambat (delay = 1 * dt = 100ms)
//   menggunakan buf[0] dan buf[2] yang sudah lengkap.
//
//   ROC > THRESHOLD_ROC → trigger RETREAT
// ---------------------------------------------------------------
const float THRESHOLD_ROC = 30.0;    // N/s  → tuning di sini
float loadRateOfChange    = 0.0;     // hasil central difference (N/s)
float rocBuf[3]           = {0,0,0}; // ring buffer: [n-1, n, n+1]
int   rocBufIdx           = 0;       // indeks sampel terbaru
int   rocBufCount         = 0;       // jumlah sampel yang sudah masuk (max 3)

// Adaptive Load Control Parameters
const float LOAD_SCALE_MIN      = 0.15;
const float LOAD_SCALE_MAX      = 1.0;
const float KP_DAMPING_FACTOR   = 0.6;
const bool  ENABLE_ADAPTIVE_MANUAL = true;
float smoothedLoad  = 0.0;
const float LOAD_ALPHA = 0.3;

// Retreat Control Parameters
const float RETREAT_VELOCITY_SCALE = 1.5;
bool retreatRequestSent = false;

// Motor Control Parameters
const float GR = 0.2786;
const float kt = 0.0663;

// Current Sensor Parameters
const int   adcMax   = 1023;
const int   nSamples = 3;
const float CalFac   = 3.40;

// Timing Intervals (milliseconds)
const int loadCellInterval        = 100;
const int encoderInterval         = 1;
const int veloInterval            = 10000;
const int CTCcalculationInterval  = 100;
const int PDcalculationInterval   = 100;
const int admittanceUpdateInterval = 10;

//==================================================================
// ADMITTANCE CONTROL PARAMETERS
// M = 0 → First-Order System:
//   Transfer Function: B(t)*Z' + K(t)*Z = F_ext
//   Backward Euler:    Z[n+1] = (B*Z[n] + F_ext*dt) / (B + K*dt)
//
// [COMMENTED - Second-order (M != 0) tidak dihapus]
// float M_adm = 101.7;   // Virtual mass (kg) - tidak digunakan saat M=0
//==================================================================

// float M_adm = 101.7;   // [COMMENTED] Virtual mass - M=0 sesuai permintaan dosen
float B_adm         = 500.0;   // Virtual damping (N.s/m) - DINAMIS dari Hill-Zajac
float K_adm         = 614.1;   // Virtual stiffness (N/m) - DINAMIS dari Hill-Zajac
float admittanceGain = 1.0;

//==================================================================
// HILL-ZAJAC MUSCLE MODEL PARAMETERS (Zajac 1989)
// Prinsip compliance:
//   Gaya kecil → a kecil → a_inv besar → K & B besar (kaku)
//   Gaya besar → a besar → a_inv kecil → K & B kecil (compliant)
//
// K = a_inv * F_MAX * fv(0) * |fl'(l0)|   (pennation α = 0)
// B = a_inv * F_MAX * fl(l0) * |fv'(0)|
//==================================================================

const float F_MAX    = 6000.0;  // Peak isometric force (N)
const float FL_OPT   = 1.0;     // f_l(l0) nilai di optimal length
const float FV_ZERO  = 1.0;     // f_v(0) nilai saat isometric
const float FL_SLOPE = 4.0;     // |f_l'(l0)| slope force-length
const float FV_SLOPE = 4.7;     // |f_v'(0)| slope force-velocity
const float K_MIN    = 100.0;   // Batas bawah K (N/m)
const float B_MIN    = 50.0;    // Batas bawah B (N.s/m)
const float K_SCALE  = 0.025;   // Scaling K → ~600 saat load=0
const float B_SCALE  = 0.018;   // Scaling B → ~500 saat load=0
const float LOAD_MAX = 100.0;   // Range maksimum load cell (N)

// Variabel monitoring Hill-Zajac
float currentActivation = 0.0;
float currentK = 614.1;
float currentB = 500.0;

// Trajectory pause
const float FORCE_PAUSE_THRESHOLD = 5.0;
bool  trajectoryPaused = false;
float pausedRefPos1 = 0.0, pausedRefPos2 = 0.0, pausedRefPos3 = 0.0;
float pausedRefVelo1 = 0.0, pausedRefVelo2 = 0.0, pausedRefVelo3 = 0.0;
float pausedRefFc1 = 0.0, pausedRefFc2 = 0.0, pausedRefFc3 = 0.0;

// Admittance state variables
// [COMMENTED] Zddot_adm tidak digunakan saat M=0 (first-order system)
// float Zddot_adm = 0.0;
float Z_adm         = 0.0;
float Zdot_adm      = 0.0;
float Z_adm_prev    = 0.0;
float Zdot_adm_prev = 0.0;  // disimpan untuk monitoring/logging

bool  admittanceEnabled = true;
float Z_offset = 0.0;

//==================================================================
// PIN DEFINITIONS
//==================================================================

const int RPWM1 = 3,  LPWM1 = 5;
const int RPWM2 = 6,  LPWM2 = 9;
const int RPWM3 = 10, LPWM3 = 11;
const int ENC1 = 4, ENC2 = 2, ENC3 = 8;
const int CurrSen1 = A0, CurrSen2 = A1, CurrSen3 = A2;

//==================================================================
// SYSTEM STATE VARIABLES
//==================================================================

int  operatingMode  = 0;
int  manualCommand  = 0;
int  manipulatorState = 0;
bool retreatHasBeenTriggered = false;
String receivedData = "";

//==================================================================
// MOTOR 1 VARIABLES
//==================================================================
float kp1 = 110.0, kd1 = 0.1, kpc1 = 30.0, kdc1 = 0.1;
float refPos1 = 0.0, refVelo1 = 0.0, refFc1 = 0.0, refCurrent1 = 0.0;
float ActPos1 = 0.0, ActVelo1 = 0.0, ActCurrent1 = 0.0;
int   position1 = 0, prevState1 = 0;
float controlValue1 = 0.0, ErrPos1 = 0.0, error1 = 0.0;
float prevError1 = 0.0, prevPos1 = 0.0;

//==================================================================
// MOTOR 2 VARIABLES
//==================================================================
float kp2 = 142.0, kd2 = 0.6, kpc2 = 33.0, kdc2 = 0.1;
float refPos2 = 0.0, refVelo2 = 0.0, refFc2 = 0.0, refCurrent2 = 0.0;
float ActPos2 = 0.0, ActVelo2 = 0.0, ActCurrent2 = 0.0;
int   position2 = 0, prevState2 = 0;
float controlValue2 = 0.0, ErrPos2 = 0.0, error2 = 0.0;
float prevError2 = 0.0, prevPos2 = 0.0;

//==================================================================
// MOTOR 3 VARIABLES
//==================================================================
float kp3 = 150.0, kd3 = 0.3, kpc3 = 38.0, kdc3 = 0.1;
float refPos3 = 0.0, refVelo3 = 0.0, refFc3 = 0.0, refCurrent3 = 0.0;
float ActPos3 = 0.0, ActVelo3 = 0.0, ActCurrent3 = 0.0;
int   position3 = 0, prevState3 = 0;
float controlValue3 = 0.0, ErrPos3 = 0.0, error3 = 0.0;
float prevError3 = 0.0, prevPos3 = 0.0;

//==================================================================
// TIMING VARIABLES
//==================================================================
long lastLoadTime        = 0;
long lastEncTime         = 0;
long lastVeloTime        = 0;
long lastCTCCalcTime     = 0;
long lastPDCalcTime      = 0;
long lastPrnTime         = 0;
long lastAdmittanceTime  = 0;

//==================================================================
// HILL-ZAJAC MUSCLE MODEL
// Hitung K dan B dinamis berdasarkan gaya eksternal
//==================================================================
void updateMuscleAdmittance(float F_external) {
    float a     = constrain(F_external / LOAD_MAX, 0.0, 1.0);
    float a_inv = 1.0 - a;

    float K_muscle = a_inv * F_MAX * FV_ZERO * FL_SLOPE;
    float B_muscle = a_inv * F_MAX * FL_OPT  * FV_SLOPE;

    K_adm = max(K_muscle * K_SCALE, K_MIN);
    B_adm = max(B_muscle * B_SCALE, B_MIN);

    currentActivation = a;
    currentK = K_adm;
    currentB = B_adm;
}

//==================================================================
// ADMITTANCE CONTROL - FIRST ORDER (M = 0)
// Persamaan: B(t)*Z' + K(t)*Z = F_ext
//
// Backward Euler:
//   Z[n+1] = (B*Z[n] + F_ext*dt) / (B + K*dt)
//   Zdot   = (Z[n+1] - Z[n]) / dt
//
// [COMMENTED] Second-order untuk referensi:
// void updateAdmittanceControl_2ndOrder(float F_external, float dt) {
//     float F_scaled = F_external * admittanceGain;
//     Zddot_adm = (F_scaled - B_adm*Zdot_adm - K_adm*Z_adm) / M_adm;
//     Zdot_adm  = Zdot_adm_prev + Zddot_adm * dt;
//     Z_adm     = Z_adm_prev    + Zdot_adm  * dt;
//     Z_adm_prev    = Z_adm;
//     Zdot_adm_prev = Zdot_adm;
// }
//==================================================================
void updateAdmittanceControl(float F_external, float dt) {
    float F_scaled = F_external * admittanceGain;

    float denom = B_adm + K_adm * dt;
    if (denom < 1e-6) denom = 1e-6;

    float Z_new  = (B_adm * Z_adm_prev + F_scaled * dt) / denom;
    Zdot_adm     = (Z_new - Z_adm_prev) / dt;

    Z_adm         = Z_new;
    Z_adm_prev    = Z_new;
    Zdot_adm_prev = Zdot_adm;
}

void resetAdmittance() {
    Z_adm         = 0.0;
    Zdot_adm      = 0.0;
    Z_adm_prev    = 0.0;
    Zdot_adm_prev = 0.0;
    Z_offset      = 0.0;
    K_adm = 614.1;  B_adm = 500.0;
    currentK = 614.1; currentB = 500.0;
    currentActivation = 0.0;
}

//==================================================================
// ROC via CENTRAL DIFFERENCE — pengganti threshold2
//
//   gradient[n] = (F[n+1] - F[n-1]) / (2 * dt)
//
//   Implementasi real-time dengan ring buffer 3 elemen:
//     - Setiap sample masuk, geser buffer: [n-1] ← [n] ← [n+1]
//     - Gradient dihitung dari elemen terlama (n-1) dan terbaru (n+1)
//     - Delay efektif = 1 sample = 100ms (dapat diterima untuk safety)
//
//   Dipanggil setiap loadCellInterval (100ms) → dt = 0.1 s
//==================================================================
void updateLoadROC(float currentLoad) {
    const float dt_roc = loadCellInterval / 1000.0;   // 0.1 s

    // Geser ring buffer: indeks melingkar 0→1→2→0→...
    rocBuf[rocBufIdx] = currentLoad;
    rocBufIdx = (rocBufIdx + 1) % 3;
    if (rocBufCount < 3) rocBufCount++;

    // Butuh minimal 3 sampel sebelum central diff valid
    if (rocBufCount < 3) {
        loadRateOfChange = 0.0;
        return;
    }

    // rocBufIdx sekarang menunjuk ke slot terlama (F[n-1]) setelah geser
    // slot berikutnya = F[n], slot setelah itu = F[n+1]
    float F_prev = rocBuf[rocBufIdx];                  // F[n-1] (terlama)
    float F_next = rocBuf[(rocBufIdx + 2) % 3];        // F[n+1] (terbaru)

    // Central difference: gradient = (F[n+1] - F[n-1]) / (2 * dt)
    loadRateOfChange = (F_next - F_prev) / (2.0 * dt_roc);
}

//==================================================================
// LOAD-BASED ADAPTIVE FUNCTIONS
// (smoothedLoad masih dipakai untuk adaptive Kp dan manual scaling)
//==================================================================
float getLoadScaling() {
    smoothedLoad = (LOAD_ALPHA * latestValidLoad) +
                   ((1.0 - LOAD_ALPHA) * smoothedLoad);

    // Tanpa threshold1 → gunakan nilai load absolut langsung
    // Scaling linier: load 0 → skala 1.0,  load >= LOAD_MAX → skala LOAD_SCALE_MIN
    float ratio = constrain(smoothedLoad / LOAD_MAX, 0.0, 1.0);
    return LOAD_SCALE_MAX - ratio * (LOAD_SCALE_MAX - LOAD_SCALE_MIN);
}

float getAdaptiveKp(float baseKp) {
    float ratio = constrain(smoothedLoad / LOAD_MAX, 0.0, 1.0);
    return baseKp * (1.0 - ratio * KP_DAMPING_FACTOR);
}

//==================================================================
// BASIC CONTROL FUNCTIONS
//==================================================================
float Error(float ref, float act) {
    return ref - act;
}

float CTC(float PosErr, float kp, float VeloErr, float kd,
          float ForceID, float GR, float kt) {
    return ((PosErr * kp) + (VeloErr * kd) + ForceID) * GR * kt;
}

float PD(float err, float kpc, float errdev, float kdc) {
    return (err * kpc) + (errdev * kdc);
}

//==================================================================
// SENSOR READING
//==================================================================
long readHX711() {
    long result = 0;
    while (digitalRead(LOADCELL_DOUT_PIN));

    for (int i = 0; i < 24; i++) {
        digitalWrite(LOADCELL_SCK_PIN, HIGH);
        delayMicroseconds(1);
        result = result << 1;
        if (digitalRead(LOADCELL_DOUT_PIN)) result++;
        digitalWrite(LOADCELL_SCK_PIN, LOW);
        delayMicroseconds(1);
    }

    digitalWrite(LOADCELL_SCK_PIN, HIGH);
    delayMicroseconds(1);
    digitalWrite(LOADCELL_SCK_PIN, LOW);
    delayMicroseconds(1);

    if (result & 0x800000) result |= ~0xFFFFFF;
    return result;
}

float avg1() {
    float val = 0;
    for (int i = 0; i < nSamples; i++) { val += analogRead(CurrSen1); delay(1); }
    return val / adcMax / nSamples;
}

float avg2() {
    float val = 0;
    for (int i = 0; i < nSamples; i++) { val += analogRead(CurrSen2); delay(1); }
    return val / adcMax / nSamples;
}

float avg3() {
    float val = 0;
    for (int i = 0; i < nSamples; i++) { val += analogRead(CurrSen3); delay(1); }
    return val / adcMax / nSamples;
}

//==================================================================
// COMMAND PARSING
//==================================================================
void parseTrajectoryCommand(String data, bool isRetreat) {
    data.replace("S", "");
    data.replace("R", "");

    float vals[9];
    int idx = 0;
    for (int i = 0; i < 8; i++) {
        int comma = data.indexOf(',');
        if (comma < 0) break;
        vals[idx++] = data.substring(0, comma).toFloat();
        data = data.substring(comma + 1);
    }
    vals[idx] = data.toFloat();

    if (trajectoryPaused) return;

    refPos1 = vals[0]; refPos2 = vals[1]; refPos3 = vals[2];
    refVelo1 = vals[3]; refVelo2 = vals[4]; refVelo3 = vals[5];
    refFc1 = vals[6]; refFc2 = vals[7]; refFc3 = vals[8];

    if (isRetreat) {
        refVelo1 *= RETREAT_VELOCITY_SCALE;
        refVelo2 *= RETREAT_VELOCITY_SCALE;
        refVelo3 *= RETREAT_VELOCITY_SCALE;
    }
}

void parseOuterLoopGains(String data) {
    data.replace("K", "");
    int c1 = data.indexOf(',');
    int motorNum = data.substring(0, c1).toInt();
    data = data.substring(c1 + 1);
    int c2 = data.indexOf(',');
    float newKp = data.substring(0, c2).toFloat();
    float newKd = data.substring(c2 + 1).toFloat();

    if (motorNum == 1) { kp1 = newKp; kd1 = newKd; Serial.print("Motor 1 Kp="); Serial.print(kp1); Serial.print(" Kd="); Serial.println(kd1); }
    else if (motorNum == 2) { kp2 = newKp; kd2 = newKd; Serial.print("Motor 2 Kp="); Serial.print(kp2); Serial.print(" Kd="); Serial.println(kd2); }
    else if (motorNum == 3) { kp3 = newKp; kd3 = newKd; Serial.print("Motor 3 Kp="); Serial.print(kp3); Serial.print(" Kd="); Serial.println(kd3); }
}

void parseInnerLoopGains(String data) {
    data.replace("P", "");
    int c1 = data.indexOf(',');
    int motorNum = data.substring(0, c1).toInt();
    data = data.substring(c1 + 1);
    int c2 = data.indexOf(',');
    float newKpc = data.substring(0, c2).toFloat();
    float newKdc = data.substring(c2 + 1).toFloat();

    if (motorNum == 1) { kpc1 = newKpc; kdc1 = newKdc; Serial.print("Motor 1 Kpc="); Serial.print(kpc1); Serial.print(" Kdc="); Serial.println(kdc1); }
    else if (motorNum == 2) { kpc2 = newKpc; kdc2 = newKdc; Serial.print("Motor 2 Kpc="); Serial.print(kpc2); Serial.print(" Kdc="); Serial.println(kdc2); }
    else if (motorNum == 3) { kpc3 = newKpc; kdc3 = newKdc; Serial.print("Motor 3 Kpc="); Serial.print(kpc3); Serial.print(" Kdc="); Serial.println(kdc3); }
}

void parseAdmittanceParams(String data) {
    data.replace("ADM", "");
    int c = data.indexOf(',');
    String param = data.substring(0, c);
    float val = data.substring(c + 1).toFloat();

    if (param == "G") {
        admittanceGain = val;
        Serial.print("Admittance Gain = "); Serial.println(admittanceGain);
    } else if (param == "M") {
        Serial.println("INFO: M = 0 (first-order mode), parameter M diabaikan");
        Serial.println("      Tuning via: G (gain), K_SCALE, B_SCALE di kode");
    } else {
        Serial.println("Parameter tersedia: G (admittance gain)");
        Serial.println("K dan B dinamis dari Hill-Zajac → tuning via K_SCALE/B_SCALE");
        Serial.println("INFO: M=0, sistem first-order (Backward Euler)");
    }
}

// ---------------------------------------------------------------
// parseThresholds DIHAPUS karena threshold1 & threshold2 tidak
// digunakan lagi. Gradient dikonfigurasi via konstanta THRESHOLD_ROC
// di bagian atas kode.
// ---------------------------------------------------------------

//==================================================================
// SYSTEM CONTROL
//==================================================================
void stopAllMotors() {
    analogWrite(RPWM1, 0); analogWrite(LPWM1, 0);
    analogWrite(RPWM2, 0); analogWrite(LPWM2, 0);
    analogWrite(RPWM3, 0); analogWrite(LPWM3, 0);
}

void resetSystem() {
    operatingMode  = 0;
    manualCommand  = 0;
    manipulatorState = 0;
    retreatHasBeenTriggered = false;
    retreatRequestSent = false;
    trajectoryPaused = false;

    stopAllMotors();

    loadCellOffset = readHX711();
    smoothedLoad   = 0.0;
    latestValidLoad = 0.0;

    // Reset ROC state (central difference buffer)
    loadRateOfChange = 0.0;
    rocBuf[0] = 0.0; rocBuf[1] = 0.0; rocBuf[2] = 0.0;
    rocBufIdx   = 0;
    rocBufCount = 0;

    position1 = 0; position2 = 0; position3 = 0;
    ActPos1 = 0.0; ActPos2 = 0.0; ActPos3 = 0.0;
    prevPos1 = 0.0; prevPos2 = 0.0; prevPos3 = 0.0;
    ErrPos1 = 0.0; ErrPos2 = 0.0; ErrPos3 = 0.0;
    error1 = 0.0; error2 = 0.0; error3 = 0.0;
    prevError1 = 0.0; prevError2 = 0.0; prevError3 = 0.0;
    ActVelo1 = 0.0; ActVelo2 = 0.0; ActVelo3 = 0.0;

    resetAdmittance();

    Serial.println("System Reset OK\n");
}

void emergencyStop() {
    operatingMode = 0;
    manualCommand = 0;
    retreatHasBeenTriggered = false;
    retreatRequestSent = false;
    stopAllMotors();
    Serial.println("EMERGENCY_STOP");
}

//==================================================================
// MOTOR CONTROL FUNCTIONS
//==================================================================
void updateEncoders() {
    int s1 = digitalRead(ENC1);
    if (s1 > prevState1) {
        if (controlValue1 > 0) position1++;
        else if (controlValue1 < 0) position1--;
    }
    ActPos1 = position1 * 0.245;
    prevState1 = s1;

    int s2 = digitalRead(ENC2);
    if (s2 > prevState2) {
        if (controlValue2 > 0) position2++;
        else if (controlValue2 < 0) position2--;
    }
    ActPos2 = position2 * 0.245;
    prevState2 = s2;

    int s3 = digitalRead(ENC3);
    if (s3 > prevState3) {
        if (controlValue3 > 0) position3++;
        else if (controlValue3 < 0) position3--;
    }
    ActPos3 = position3 * 0.245;
    prevState3 = s3;
}

void updateVelocities() {
    ActVelo1 = (ActPos1 - prevPos1) / 10;
    ActVelo2 = (ActPos2 - prevPos2) / 10;
    ActVelo3 = (ActPos3 - prevPos3) / 10;
    prevPos1 = ActPos1;
    prevPos2 = ActPos2;
    prevPos3 = ActPos3;
}

void calculateCTC() {
    ErrPos1 = Error(refPos1, ActPos1);
    ErrPos2 = Error(refPos2, ActPos2);
    ErrPos3 = Error(refPos3, ActPos3);

    refCurrent1 = CTC(ErrPos1, getAdaptiveKp(kp1), Error(refVelo1, ActVelo1), kd1, refFc1, GR, kt);
    refCurrent2 = CTC(ErrPos2, getAdaptiveKp(kp2), Error(refVelo2, ActVelo2), kd2, refFc2, GR, kt);
    refCurrent3 = CTC(ErrPos3, getAdaptiveKp(kp3), Error(refVelo3, ActVelo3), kd3, refFc3, GR, kt);
}

void calculateCTCWithAdmittance() {
    float rp1 = refPos1 - (Z_adm * 1000.0);
    float rp2 = refPos2 - (Z_adm * 1000.0);
    float rp3 = refPos3 - (Z_adm * 1000.0);

    float rv1 = refVelo1 - Zdot_adm;
    float rv2 = refVelo2 - Zdot_adm;
    float rv3 = refVelo3 - Zdot_adm;

    ErrPos1 = Error(rp1, ActPos1);
    ErrPos2 = Error(rp2, ActPos2);
    ErrPos3 = Error(rp3, ActPos3);

    refCurrent1 = CTC(ErrPos1, getAdaptiveKp(kp1), Error(rv1, ActVelo1), kd1, refFc1, GR, kt);
    refCurrent2 = CTC(ErrPos2, getAdaptiveKp(kp2), Error(rv2, ActVelo2), kd2, refFc2, GR, kt);
    refCurrent3 = CTC(ErrPos3, getAdaptiveKp(kp3), Error(rv3, ActVelo3), kd3, refFc3, GR, kt);
}

void calculatePD() {
    error1 = Error(refCurrent1, ActCurrent1);
    error2 = Error(refCurrent2, ActCurrent2);
    error3 = Error(refCurrent3, ActCurrent3);

    controlValue1 = PD(error1, kpc1, (error1 - prevError1) / 0.1, kdc1);
    controlValue2 = PD(error2, kpc2, (error2 - prevError2) / 0.1, kdc2);
    controlValue3 = PD(error3, kpc3, (error3 - prevError3) / 0.1, kdc3);

    prevError1 = error1;
    prevError2 = error2;
    prevError3 = error3;

    if (operatingMode == 1) {
        float scale = getLoadScaling();
        controlValue1 = constrain(controlValue1 * scale, -255, 255);
        controlValue2 = constrain(controlValue2 * scale, -255, 255);
        controlValue3 = constrain(controlValue3 * scale, -255, 255);
    } else {
        controlValue1 = constrain(controlValue1, -255, 255);
        controlValue2 = constrain(controlValue2, -255, 255);
        controlValue3 = constrain(controlValue3, -255, 255);
    }
}

void applyMotorControl() {
    if (ErrPos1 > 0)      { analogWrite(RPWM1, abs(controlValue1)); analogWrite(LPWM1, 0); }
    else if (ErrPos1 < 0) { analogWrite(RPWM1, 0); analogWrite(LPWM1, abs(controlValue1)); }
    else                  { analogWrite(RPWM1, 0); analogWrite(LPWM1, 0); }

    if (ErrPos2 > 0)      { analogWrite(RPWM2, abs(controlValue2)); analogWrite(LPWM2, 0); }
    else if (ErrPos2 < 0) { analogWrite(RPWM2, 0); analogWrite(LPWM2, abs(controlValue2)); }
    else                  { analogWrite(RPWM2, 0); analogWrite(LPWM2, 0); }

    if (ErrPos3 > 0)      { analogWrite(RPWM3, abs(controlValue3)); analogWrite(LPWM3, 0); }
    else if (ErrPos3 < 0) { analogWrite(RPWM3, 0); analogWrite(LPWM3, abs(controlValue3)); }
    else                  { analogWrite(RPWM3, 0); analogWrite(LPWM3, 0); }
}

void manualModeControl() {
    int speed = ENABLE_ADAPTIVE_MANUAL ? (int)(MANUAL_SPEED * getLoadScaling()) : MANUAL_SPEED;

    if (manualCommand == 1) {
        analogWrite(RPWM1, speed); analogWrite(LPWM1, 0);
        analogWrite(RPWM2, speed); analogWrite(LPWM2, 0);
        analogWrite(RPWM3, speed); analogWrite(LPWM3, 0);
    } else if (manualCommand == 2) {
        analogWrite(RPWM1, 0); analogWrite(LPWM1, speed);
        analogWrite(RPWM2, 0); analogWrite(LPWM2, speed);
        analogWrite(RPWM3, 0); analogWrite(LPWM3, speed);
    } else {
        stopAllMotors();
    }
}

//==================================================================
// SETUP
//==================================================================
void setup() {
    Serial.begin(115200);

    pinMode(RPWM1, OUTPUT); pinMode(LPWM1, OUTPUT);
    pinMode(RPWM2, OUTPUT); pinMode(LPWM2, OUTPUT);
    pinMode(RPWM3, OUTPUT); pinMode(LPWM3, OUTPUT);

    pinMode(ENC1, INPUT); pinMode(ENC2, INPUT); pinMode(ENC3, INPUT);

    pinMode(CurrSen1, INPUT); pinMode(CurrSen2, INPUT); pinMode(CurrSen3, INPUT);

    pinMode(LOADCELL_DOUT_PIN, INPUT);
    pinMode(LOADCELL_SCK_PIN, OUTPUT);

    while (!Serial) { ; }

    manipulatorState = 0;
    operatingMode    = 0;

    Serial.println("===========================================");
    Serial.println("  3-RPS Parallel Robot Control System");
    Serial.println("  Admittance + Hill-Zajac Dynamic K & B");
    Serial.println("  M = 0 | First-Order | Backward Euler");
    Serial.println("  Retreat trigger: Central Difference gradient");
    Serial.println("  Ref: Zajac (1989)");
    Serial.println("===========================================");
    Serial.print("  K range: "); Serial.print(K_MIN);
    Serial.print(" ~ "); Serial.print(F_MAX * FV_ZERO * FL_SLOPE * K_SCALE, 1);
    Serial.println(" N/m");
    Serial.print("  B range: "); Serial.print(B_MIN);
    Serial.print(" ~ "); Serial.print(F_MAX * FL_OPT * FV_SLOPE * B_SCALE, 1);
    Serial.println(" N.s/m");
    Serial.print("  Gradient thresh : "); Serial.print(THRESHOLD_ROC, 1); Serial.println(" N/s");
    Serial.println("  Gradient method : (F[n+1]-F[n-1])/(2*dt), delay=1 sample (100ms)");
    Serial.println("  System order    : 1st (M=0, Backward Euler)");
    Serial.println("===========================================\n");
}

//==================================================================
// MAIN LOOP
//==================================================================
void loop() {
    long currentTime = millis();

    //================================================================
    // SERIAL COMMAND PROCESSING
    //================================================================
    if (Serial.available() > 0) {
        char c = Serial.read();
        receivedData += c;

        if (c == '\n') {
            receivedData.trim();

            if (receivedData.startsWith("S")) {
                operatingMode = 1;
                retreatHasBeenTriggered = false;
                retreatRequestSent = false;
                manipulatorState = 0;
                parseTrajectoryCommand(receivedData, false);
            }
            else if (receivedData.startsWith("R") && receivedData.indexOf(',') > 0) {
                operatingMode = 2;
                manipulatorState = 0;
                parseTrajectoryCommand(receivedData, true);
            }
            else if (receivedData == "RETREAT_COMPLETE") {
                operatingMode = 0;
                manualCommand = 0;
                retreatHasBeenTriggered = false;
                retreatRequestSent = false;
                stopAllMotors();
                Serial.println("ACK_RETREAT_COMPLETE");
            }
            else if (receivedData.startsWith("X")) { resetSystem(); }
            else if (receivedData.startsWith("E")) { emergencyStop(); }
            else if (receivedData == "1") {
                operatingMode = 0; manualCommand = 1;
                retreatHasBeenTriggered = false; retreatRequestSent = false;
            }
            else if (receivedData == "2") {
                operatingMode = 0; manualCommand = 2;
                retreatHasBeenTriggered = false; retreatRequestSent = false;
            }
            else if (receivedData == "0") { operatingMode = 0; manualCommand = 0; }
            else if (receivedData.startsWith("K")) {
                operatingMode = 0; manualCommand = 0;
                parseOuterLoopGains(receivedData);
            }
            else if (receivedData.startsWith("P")) {
                operatingMode = 0; manualCommand = 0;
                parseInnerLoopGains(receivedData);
            }
            // -------------------------------------------------------
            // Command "T" (threshold) DIHAPUS karena threshold1 &
            // threshold2 sudah tidak digunakan.
            // Tuning gradient → edit THRESHOLD_ROC di kode.
            // -------------------------------------------------------
            else if (receivedData == "ADMITTANCE_ON") {
                admittanceEnabled = true;
                Serial.println("Admittance ON (Hill-Zajac Dynamic K & B, M=0, Backward Euler)");
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
                Serial.println("\n=== Admittance Status (Hill-Zajac, M=0) ===");
                Serial.print("Enabled     : "); Serial.println(admittanceEnabled ? "YES" : "NO");
                Serial.println("Model       : First-order (Backward Euler)");
                Serial.print("Activation  : "); Serial.println(currentActivation, 4);
                Serial.print("K_adm       : "); Serial.print(currentK, 2); Serial.println(" N/m");
                Serial.print("B_adm       : "); Serial.print(currentB, 2); Serial.println(" N.s/m");
                Serial.print("tau (B/K)   : "); Serial.print(currentB / currentK, 4); Serial.println(" s");
                Serial.print("Z_adm       : "); Serial.print(Z_adm * 1000, 4); Serial.println(" mm");
                Serial.print("Zdot_adm    : "); Serial.print(Zdot_adm * 1000, 4); Serial.println(" mm/s");
                Serial.print("F_ext       : "); Serial.print(latestValidLoad, 2); Serial.println(" N");
                Serial.print("Gradient     : "); Serial.print(loadRateOfChange, 2); Serial.println(" N/s");
                Serial.print("Grad thresh  : "); Serial.print(THRESHOLD_ROC, 1); Serial.println(" N/s");
                Serial.println("Method       : Central difference (F[n+1]-F[n-1])/(2*dt)");
                Serial.println("============================================\n");
            }

            receivedData = "";
        }
    }

    //================================================================
    // AUTO MODE (FORWARD / RETREAT)
    //================================================================
    if (operatingMode == 1 || operatingMode == 2) {

        //------------------------------------------------------------
        // Load cell reading + ROC calculation (forward only)
        //------------------------------------------------------------
        if (operatingMode == 1 && currentTime - lastLoadTime >= loadCellInterval) {

            if (retreatHasBeenTriggered) {
                manipulatorState = 1;
            } else {
                long  raw  = readHX711() - loadCellOffset;
                float load = constrain(raw / 10000.0, 0.0, 100.0);
                latestValidLoad = load;

                // Hitung ROC (EMA-smoothed dF/dt)
                updateLoadROC(load);

                // Retreat trigger: ROC melebihi threshold
                if (loadRateOfChange > THRESHOLD_ROC) {
                    manipulatorState = 1;
                    retreatHasBeenTriggered = true;
                    if (!retreatRequestSent) {
                        Serial.print("RETREAT (gradient=");
                        Serial.print(loadRateOfChange, 2);
                        Serial.println(" N/s)");
                        retreatRequestSent = true;
                    }
                } else {
                    manipulatorState = 0;  // ← RUNNING
                }
            }
            lastLoadTime = currentTime;
        }

        //------------------------------------------------------------
        // Admittance update (Hill-Zajac → Backward Euler)
        //------------------------------------------------------------
        if (admittanceEnabled && operatingMode == 1 &&
            currentTime - lastAdmittanceTime >= admittanceUpdateInterval) {

            float F_ext   = latestValidLoad;
            float dt_adm  = admittanceUpdateInterval / 1000.0;

            updateMuscleAdmittance(F_ext);
            updateAdmittanceControl(F_ext, dt_adm);

            if (F_ext > FORCE_PAUSE_THRESHOLD && !trajectoryPaused) {
                trajectoryPaused = true;
                pausedRefPos1 = refPos1; pausedRefPos2 = refPos2; pausedRefPos3 = refPos3;
                pausedRefVelo1 = refVelo1; pausedRefVelo2 = refVelo2; pausedRefVelo3 = refVelo3;
                pausedRefFc1 = refFc1; pausedRefFc2 = refFc2; pausedRefFc3 = refFc3;
                Serial.println("PAUSE_TRAJECTORY");
            } else if (F_ext <= FORCE_PAUSE_THRESHOLD && trajectoryPaused) {
                trajectoryPaused = false;
                refPos1 = pausedRefPos1; refPos2 = pausedRefPos2; refPos3 = pausedRefPos3;
                refVelo1 = 0.0; refVelo2 = 0.0; refVelo3 = 0.0;
                refFc1 = pausedRefFc1; refFc2 = pausedRefFc2; refFc3 = pausedRefFc3;
                Serial.println("RESUME_TRAJECTORY");
            }

            lastAdmittanceTime = currentTime;
        }

        //------------------------------------------------------------
        // Encoder update
        //------------------------------------------------------------
        if (currentTime - lastEncTime >= encoderInterval && manipulatorState != 1) {
            updateEncoders();
            lastEncTime = currentTime;
        }

        //------------------------------------------------------------
        // Control calculation (hanya saat manipulatorState == 0)
        //------------------------------------------------------------
        if (manipulatorState == 0) {
            if (currentTime - lastVeloTime >= veloInterval) {
                updateVelocities();
                lastVeloTime = currentTime;
            }

            if (currentTime - lastCTCCalcTime >= CTCcalculationInterval) {
                if (admittanceEnabled && operatingMode == 1) {
                    calculateCTCWithAdmittance();
                } else {
                    calculateCTC();
                }
                lastCTCCalcTime = currentTime;
            }

            if (currentTime - lastPDCalcTime >= PDcalculationInterval) {
                calculatePD();
                lastPDCalcTime = currentTime;
            }
        }

        //------------------------------------------------------------
        // Motor execution
        //------------------------------------------------------------
        if (manipulatorState == 0) {
            applyMotorControl();
        } else {
            stopAllMotors();
        }

        //------------------------------------------------------------
        // Status reporting
        //------------------------------------------------------------
        if (currentTime - lastPrnTime >= loadCellInterval) {
            String status = (manipulatorState == 0) ? "running" : "paused";
            String mode   = (operatingMode == 1) ? "forward" : "retreat";

            Serial.print("status:"); Serial.print(status);
            Serial.print(",mode:"); Serial.print(mode);
            Serial.print(",load:"); Serial.print(latestValidLoad, 2);
            Serial.print(",gradient:"); Serial.print(loadRateOfChange, 2);
            Serial.print(",grad_thresh:"); Serial.print(THRESHOLD_ROC, 1);
            Serial.print(",scale:"); Serial.print(getLoadScaling(), 2);
            Serial.print(",pos:"); Serial.print(ActPos1, 2);
            Serial.print(","); Serial.print(ActPos2, 2);
            Serial.print(","); Serial.print(ActPos3, 2);

            if (admittanceEnabled && operatingMode == 1) {
                Serial.print(",activation:"); Serial.print(currentActivation, 3);
                Serial.print(",K_adm:"); Serial.print(currentK, 2);
                Serial.print(",B_adm:"); Serial.print(currentB, 2);
                Serial.print(",tau:"); Serial.print(currentB / currentK, 4);
                Serial.print(",Z_adm:"); Serial.print(Z_adm * 1000, 3);
                Serial.print(",Zdot_adm:"); Serial.print(Zdot_adm * 1000, 3);
                Serial.print(",traj_paused:"); Serial.print(trajectoryPaused ? "1" : "0");
            }

            Serial.println("");
            lastPrnTime = currentTime;
        }
    }

    //================================================================
    // MANUAL MODE
    //================================================================
    else {
        if (ENABLE_ADAPTIVE_MANUAL && currentTime - lastLoadTime >= loadCellInterval) {
            long  raw  = readHX711() - loadCellOffset;
            float load = constrain(raw / 10000.0, 0.0, 100.0);
            latestValidLoad = load;
            updateLoadROC(load);  // ROC tetap di-update di manual mode
            lastLoadTime = currentTime;
        }
        manualModeControl();
    }
}
