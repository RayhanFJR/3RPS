//==================================================================
// ARDUINO MOTOR CONTROL SYSTEM - ADMITTANCE CONTROL VERSION
// Features:
// - Admittance Control: Z(s) = F_ext / (101.7s² + 500s + 614.1)
// - CTC with feedforward compensation
// - Load-based adaptive scaling
// - Trajectory-based retreat communication
// - Manual/Auto modes with safety system
//==================================================================

//==================================================================
// CONSTANTS & CONFIGURATION
//==================================================================

// Motor Control Speed
const int MANUAL_SPEED = 125;
const int RETREAT_SPEED = 150;

// Load Cell Configuration
const int LOADCELL_DOUT_PIN = 12;
const int LOADCELL_SCK_PIN = 13;
int threshold1 = 20;  // Base threshold 1 (akan dijadikan dinamis)
int threshold2 = 40;  // Base threshold 2 (akan dijadikan dinamis)
long loadCellOffset = 0;
float latestValidLoad = 0.0;

// Dynamic Threshold Parameters (Rate of Change Based)
float dynamicThreshold1 = 20.0;  // Dynamic threshold 1
float dynamicThreshold2 = 40.0;  // Dynamic threshold 2
float prevLoad = 0.0;             // Previous load value for rate calculation
float loadChangeRate = 0.0;       // Rate of change (N/s)
float loadChangeMagnitude = 0.0;  // Magnitude of change (N)
const float RATE_SENSITIVITY = 2.0;  // Sensitivity multiplier for rate of change
const float MAGNITUDE_SENSITIVITY = 0.5;  // Sensitivity multiplier for magnitude change
const float MAX_THRESHOLD_MULTIPLIER = 3.0;  // Maximum threshold multiplier
const float MIN_THRESHOLD_MULTIPLIER = 0.3;  // Minimum threshold multiplier
const float RATE_SMOOTHING_ALPHA = 0.7;  // Smoothing factor for rate of change

// Adaptive Load Control Parameters
const float LOAD_SCALE_MIN = 0.15;
const float LOAD_SCALE_MAX = 1.0;
const float KP_DAMPING_FACTOR = 0.6;
const bool ENABLE_ADAPTIVE_MANUAL = true;
float smoothedLoad = 0.0;
const float LOAD_ALPHA = 0.3;

// Retreat Control Parameters
const float RETREAT_VELOCITY_SCALE = 1.5;
bool retreatRequestSent = false;

// Motor Control Parameters
const float GR = 0.2786;
const float kt = 0.0663;

// Current Sensor Parameters
const int adcMax = 1023;
const int nSamples = 3;
const float CalFac = 3.40;

// Timing Intervals (milliseconds)
const int loadCellInterval = 100;
const int encoderInterval = 1;
const int veloInterval = 10000;
const int CTCcalculationInterval = 100;
const int PDcalculationInterval = 100;
const int admittanceUpdateInterval = 10;  // 10ms = 100Hz update rate

//==================================================================
// ADMITTANCE CONTROL PARAMETERS
//==================================================================
// Transfer Function: Z(s) = F_external / (M*s^2 + B*s + K)
// Original: Z(s) = F_ext / (101.7*s^2 + 500*s + 614.1)

float M_adm = 101.7;    // Virtual mass (kg)
float B_adm = 500.0;    // Virtual damping (N·s/m)
float K_adm = 614.1;    // Virtual stiffness (N/m)

// Compliance gain multiplier (adjustable via serial command)
float admittanceGain = 1.0;

// Trajectory pause parameters
const float FORCE_PAUSE_THRESHOLD = 5.0;  // Pause trajectory if force > 5N
bool trajectoryPaused = false;
float pausedRefPos1 = 0.0;
float pausedRefPos2 = 0.0;
float pausedRefPos3 = 0.0;
float pausedRefVelo1 = 0.0;
float pausedRefVelo2 = 0.0;
float pausedRefVelo3 = 0.0;
float pausedRefFc1 = 0.0;
float pausedRefFc2 = 0.0;
float pausedRefFc3 = 0.0;

// Admittance state variables (untuk diskrit integration)
float Z_adm = 0.0;       // Displacement dari admittance (m)
float Zdot_adm = 0.0;    // Velocity dari admittance (m/s)
float Zddot_adm = 0.0;   // Acceleration dari admittance (m/s^2)

float Z_adm_prev = 0.0;
float Zdot_adm_prev = 0.0;

// Enable/disable admittance control
bool admittanceEnabled = true;

// Admittance offset (initial position reference)
float Z_offset = 0.0;

//==================================================================
// PIN DEFINITIONS 
//==================================================================

const int RPWM1 = 3, LPWM1 = 5;
const int RPWM2 = 6, LPWM2 = 9;
const int RPWM3 = 10, LPWM3 = 11;

const int ENC1 = 4;
const int ENC2 = 2;
const int ENC3 = 8;

const int CurrSen1 = A0;
const int CurrSen2 = A1;
const int CurrSen3 = A2;

//==================================================================
// SYSTEM STATE VARIABLES
//==================================================================

int operatingMode = 0;  // 0=Manual, 1=Auto Forward, 2=Auto Retreat
int manualCommand = 0;
int manipulatorState = 0;
bool retreatHasBeenTriggered = false;

String receivedData = "";

//==================================================================
// MOTOR 1 VARIABLES
//==================================================================

float kp1 = 110.0, kd1 = 0.1;
float kpc1 = 30.0, kdc1 = 0.1;

float refPos1 = 0.0, refVelo1 = 0.0, refFc1 = 0.0, refCurrent1 = 0.0;
float refAccel1 = 0.0, prevRefVelo1 = 0.0;

float ActPos1 = 0.0, ActVelo1 = 0.0, ActCurrent1 = 0.0;
int position1 = 0, prevState1 = 0;

float controlValue1 = 0.0, ErrPos1 = 0.0, error1 = 0.0;
float prevError1 = 0.0, prevPos1 = 0.0;

//==================================================================
// MOTOR 2 VARIABLES
//==================================================================

float kp2 = 142.0, kd2 = 0.6;
float kpc2 = 33.0, kdc2 = 0.1;

float refPos2 = 0.0, refVelo2 = 0.0, refFc2 = 0.0, refCurrent2 = 0.0;
float refAccel2 = 0.0, prevRefVelo2 = 0.0;

float ActPos2 = 0.0, ActVelo2 = 0.0, ActCurrent2 = 0.0;
int position2 = 0, prevState2 = 0;

float controlValue2 = 0.0, ErrPos2 = 0.0, error2 = 0.0;
float prevError2 = 0.0, prevPos2 = 0.0;

//==================================================================
// MOTOR 3 VARIABLES
//==================================================================

float kp3 = 150.0, kd3 = 0.3;
float kpc3 = 38.0, kdc3 = 0.1;

float refPos3 = 0.0, refVelo3 = 0.0, refFc3 = 0.0, refCurrent3 = 0.0;
float refAccel3 = 0.0, prevRefVelo3 = 0.0;

float ActPos3 = 0.0, ActVelo3 = 0.0, ActCurrent3 = 0.0;
int position3 = 0, prevState3 = 0;

float controlValue3 = 0.0, ErrPos3 = 0.0, error3 = 0.0;
float prevError3 = 0.0, prevPos3 = 0.0;

//==================================================================
// TIMING VARIABLES
//==================================================================

long lastLoadTime = 0;
long lastEncTime = 0;
long lastVeloTime = 0;
long lastCTCCalcTime = 0;
long lastPDCalcTime = 0;
long lastPrnTime = 0;
long lastAdmittanceTime = 0;

//==================================================================
// ADMITTANCE CONTROL FUNCTIONS
//==================================================================

void updateAdmittanceControl(float F_external, float dt) {
    // Admittance equation: M*Z'' + B*Z' + K*Z = F_external
    // Solving for Z'': Z'' = (F_external - B*Z' - K*Z) / M
    
    // Apply gain multiplier to external force
    float F_scaled = F_external * admittanceGain;
    
    // Calculate acceleration
    Zddot_adm = (F_scaled - B_adm * Zdot_adm - K_adm * Z_adm) / M_adm;
    
    // Euler integration untuk velocity
    Zdot_adm = Zdot_adm_prev + Zddot_adm * dt;
    
    // Euler integration untuk position
    Z_adm = Z_adm_prev + Zdot_adm * dt;
    
    // Update previous values
    Z_adm_prev = Z_adm;
    Zdot_adm_prev = Zdot_adm;
}

void resetAdmittance() {
    Z_adm = 0.0;
    Zdot_adm = 0.0;
    Zddot_adm = 0.0;
    Z_adm_prev = 0.0;
    Zdot_adm_prev = 0.0;
    Z_offset = 0.0;
}

void setAdmittanceOffset(float offset) {
    Z_offset = offset;
}

//==================================================================
// LOAD-BASED ADAPTIVE FUNCTIONS
//==================================================================

// Calculate rate of change and update dynamic thresholds
void updateDynamicThresholds(float currentLoad, float dt) {
    // Calculate rate of change (N/s)
    float currentRate = 0.0;
    if (dt > 0.0) {
        currentRate = (currentLoad - prevLoad) / dt;
    }
    
    // Smooth the rate of change to reduce noise
    loadChangeRate = (RATE_SMOOTHING_ALPHA * currentRate) + ((1.0 - RATE_SMOOTHING_ALPHA) * loadChangeRate);
    
    // Calculate magnitude of change
    loadChangeMagnitude = abs(currentLoad - prevLoad);
    
    // Calculate dynamic threshold multiplier based on rate and magnitude
    // Higher rate of change or larger magnitude = lower threshold (more sensitive)
    // Lower rate of change or smaller magnitude = higher threshold (less sensitive)
    
    float rateFactor = 1.0 + (abs(loadChangeRate) * RATE_SENSITIVITY);
    float magnitudeFactor = 1.0 + (loadChangeMagnitude * MAGNITUDE_SENSITIVITY);
    
    // Combine factors (use the more sensitive one)
    float combinedFactor = max(rateFactor, magnitudeFactor);
    
    // Invert: higher rate/magnitude = lower threshold (more sensitive)
    // So we divide base threshold by the factor
    float thresholdMultiplier = 1.0 / combinedFactor;
    
    // Clamp multiplier to reasonable bounds
    thresholdMultiplier = constrain(thresholdMultiplier, MIN_THRESHOLD_MULTIPLIER, MAX_THRESHOLD_MULTIPLIER);
    
    // Update dynamic thresholds using current base thresholds (which can be changed via serial)
    dynamicThreshold1 = threshold1 * thresholdMultiplier;
    dynamicThreshold2 = threshold2 * thresholdMultiplier;
    
    // Ensure threshold2 is always greater than threshold1
    if (dynamicThreshold2 <= dynamicThreshold1) {
        dynamicThreshold2 = dynamicThreshold1 + 5.0;
    }
    
    // Update previous load for next calculation
    prevLoad = currentLoad;
}

float getLoadScaling() {
    smoothedLoad = (LOAD_ALPHA * latestValidLoad) + ((1.0 - LOAD_ALPHA) * smoothedLoad);
    
    // Use dynamic thresholds instead of static ones
    if (smoothedLoad < dynamicThreshold1) {
        return LOAD_SCALE_MAX;
    } else if (smoothedLoad >= dynamicThreshold2) {
        return LOAD_SCALE_MIN;
    } else {
        float loadRatio = (smoothedLoad - dynamicThreshold1) / (dynamicThreshold2 - dynamicThreshold1);
        return LOAD_SCALE_MAX - (loadRatio * (LOAD_SCALE_MAX - LOAD_SCALE_MIN));
    }
}

float getAdaptiveKp(float baseKp) {
    // Use dynamic thresholds instead of static ones
    if (smoothedLoad < dynamicThreshold1) {
        return baseKp;
    } else if (smoothedLoad >= dynamicThreshold2) {
        return baseKp * (1.0 - KP_DAMPING_FACTOR);
    } else {
        float loadRatio = (smoothedLoad - dynamicThreshold1) / (dynamicThreshold2 - dynamicThreshold1);
        float damping = 1.0 - (loadRatio * KP_DAMPING_FACTOR);
        return baseKp * damping;
    }
}

//==================================================================
// CONTROL FUNCTIONS
//==================================================================

float Error(float ref, float act) {
    return ref - act;
}

float CTC(float PosError, float kp, float VeloError, float kd, float ForceID, float GR, float kt) {
    return ((PosError * kp) + (VeloError * kd) + ForceID) * GR * kt;
}

float PD(float error, float kpc, float errordev, float kdc) {
    return (error * kpc) + (errordev * kdc);
}

//==================================================================
// SENSOR READING FUNCTIONS
//==================================================================

long readHX711() {
    long result = 0;
    while (digitalRead(LOADCELL_DOUT_PIN));
    
    for (int i = 0; i < 24; i++) {
        digitalWrite(LOADCELL_SCK_PIN, HIGH);
        delayMicroseconds(1);
        result = result << 1;
        if (digitalRead(LOADCELL_DOUT_PIN)) {
            result++;
        }
        digitalWrite(LOADCELL_SCK_PIN, LOW);
        delayMicroseconds(1);
    }
    
    digitalWrite(LOADCELL_SCK_PIN, HIGH);
    delayMicroseconds(1);
    digitalWrite(LOADCELL_SCK_PIN, LOW);
    delayMicroseconds(1);
    
    if (result & 0x800000) {
        result |= ~0xFFFFFF;
    }
    
    return result;
}

float avg1() {
    float val1 = 0;
    for (int i = 0; i < nSamples; i++) {
        val1 += analogRead(CurrSen1);
        delay(1);
    }
    return val1 / adcMax / nSamples;
}

float avg2() {
    float val2 = 0;
    for (int i = 0; i < nSamples; i++) {
        val2 += analogRead(CurrSen2);
        delay(1);
    }
    return val2 / adcMax / nSamples;
}

float avg3() {
    float val3 = 0;
    for (int i = 0; i < nSamples; i++) {
        val3 += analogRead(CurrSen3);
        delay(1);
    }
    return val3 / adcMax / nSamples;
}

//==================================================================
// COMMAND PARSING FUNCTIONS
//==================================================================

void parseTrajectoryCommand(String data, bool isRetreat) {
    data.replace("S", "");
    data.replace("R", "");
    
    int commaIndex1 = data.indexOf(',');
    String refPos1Str = data.substring(0, commaIndex1);
    data = data.substring(commaIndex1 + 1);
    
    int commaIndex2 = data.indexOf(',');
    String refPos2Str = data.substring(0, commaIndex2);
    data = data.substring(commaIndex2 + 1);
    
    int commaIndex3 = data.indexOf(',');
    String refPos3Str = data.substring(0, commaIndex3);
    data = data.substring(commaIndex3 + 1);
    
    int commaIndex4 = data.indexOf(',');
    String refVelo1Str = data.substring(0, commaIndex4);
    data = data.substring(commaIndex4 + 1);
    
    int commaIndex5 = data.indexOf(',');
    String refVelo2Str = data.substring(0, commaIndex5);
    data = data.substring(commaIndex5 + 1);
    
    int commaIndex6 = data.indexOf(',');
    String refVelo3Str = data.substring(0, commaIndex6);
    data = data.substring(commaIndex6 + 1);
    
    int commaIndex7 = data.indexOf(',');
    String refFc1Str = data.substring(0, commaIndex7);
    data = data.substring(commaIndex7 + 1);
    
    int commaIndex8 = data.indexOf(',');
    String refFc2Str = data.substring(0, commaIndex8);
    String refFc3Str = data.substring(commaIndex8 + 1);
    
    // CRITICAL FIX: IGNORE incoming trajectory if paused
    if (trajectoryPaused) {
        // Don't update any reference, just discard this command
        return;  // Exit early, trajectory stays frozen
    }
    
    // Only update if NOT paused
    refPos1 = refPos1Str.toFloat();
    refPos2 = refPos2Str.toFloat();
    refPos3 = refPos3Str.toFloat();
    refVelo1 = refVelo1Str.toFloat();
    refVelo2 = refVelo2Str.toFloat();
    refVelo3 = refVelo3Str.toFloat();
    refFc1 = refFc1Str.toFloat();
    refFc2 = refFc2Str.toFloat();
    refFc3 = refFc3Str.toFloat();
    
    if (isRetreat) {
        refVelo1 *= RETREAT_VELOCITY_SCALE;
        refVelo2 *= RETREAT_VELOCITY_SCALE;
        refVelo3 *= RETREAT_VELOCITY_SCALE;
    }
}

void parseOuterLoopGains(String data) {
    data.replace("K", "");
    
    int commaIndex1 = data.indexOf(',');
    String motorNumStr = data.substring(0, commaIndex1);
    data = data.substring(commaIndex1 + 1);
    
    int commaIndex2 = data.indexOf(',');
    String kpStr = data.substring(0, commaIndex2);
    String kdStr = data.substring(commaIndex2 + 1);
    
    int motorNum = motorNumStr.toInt();
    float newKp = kpStr.toFloat();
    float newKd = kdStr.toFloat();
    
    if (motorNum == 1) {
        kp1 = newKp; kd1 = newKd;
        Serial.print("Motor 1 Outer Loop: Kp=");
        Serial.print(kp1); Serial.print(", Kd="); Serial.println(kd1);
    } else if (motorNum == 2) {
        kp2 = newKp; kd2 = newKd;
        Serial.print("Motor 2 Outer Loop: Kp=");
        Serial.print(kp2); Serial.print(", Kd="); Serial.println(kd2);
    } else if (motorNum == 3) {
        kp3 = newKp; kd3 = newKd;
        Serial.print("Motor 3 Outer Loop: Kp=");
        Serial.print(kp3); Serial.print(", Kd="); Serial.println(kd3);
    }
    Serial.println("");
}

void parseInnerLoopGains(String data) {
    data.replace("P", "");
    
    int commaIndex1 = data.indexOf(',');
    String motorNumStr = data.substring(0, commaIndex1);
    data = data.substring(commaIndex1 + 1);
    
    int commaIndex2 = data.indexOf(',');
    String kpcStr = data.substring(0, commaIndex2);
    String kdcStr = data.substring(commaIndex2 + 1);
    
    int motorNum = motorNumStr.toInt();
    float newKpc = kpcStr.toFloat();
    float newKdc = kdcStr.toFloat();
    
    if (motorNum == 1) {
        kpc1 = newKpc; kdc1 = newKdc;
        Serial.print("Motor 1 Inner Loop: Kpc=");
        Serial.print(kpc1); Serial.print(", Kdc="); Serial.println(kdc1);
    } else if (motorNum == 2) {
        kpc2 = newKpc; kdc2 = newKdc;
        Serial.print("Motor 2 Inner Loop: Kpc=");
        Serial.print(kpc2); Serial.print(", Kdc="); Serial.println(kdc2);
    } else if (motorNum == 3) {
        kpc3 = newKpc; kdc3 = newKdc;
        Serial.print("Motor 3 Inner Loop: Kpc=");
        Serial.print(kpc3); Serial.print(", Kdc="); Serial.println(kdc3);
    }
    Serial.println("");
}

void parseAdmittanceParams(String data) {
    data.replace("ADM", "");
    
    int commaIndex1 = data.indexOf(',');
    String paramStr = data.substring(0, commaIndex1);
    String valueStr = data.substring(commaIndex1 + 1);
    
    float value = valueStr.toFloat();
    
    // Parse parameter type: M, B, K, or G (gain)
    if (paramStr == "M") {
        M_adm = value;
        Serial.print("Virtual Mass M = "); Serial.println(M_adm);
    } else if (paramStr == "B") {
        B_adm = value;
        Serial.print("Virtual Damping B = "); Serial.println(B_adm);
    } else if (paramStr == "K") {
        K_adm = value;
        Serial.print("Virtual Stiffness K = "); Serial.println(K_adm);
    } else if (paramStr == "G") {
        admittanceGain = value;
        Serial.print("Admittance Gain = "); Serial.println(admittanceGain);
    } else {
        Serial.println("Unknown parameter. Use M, B, K, or G");
    }
}

void parseThresholds(String data) {
    data.replace("T", "");
    
    int commaIndex = data.indexOf(',');
    String t1Str = data.substring(0, commaIndex);
    String t2Str = data.substring(commaIndex + 1);
    
    // Update base thresholds (dynamic thresholds will be calculated from these)
    threshold1 = t1Str.toInt();
    threshold2 = t2Str.toInt();
    
    // Also update base constants for dynamic calculation
    // Note: We can't modify const, so we'll use threshold1/threshold2 as base
    // But dynamic thresholds will override these
    
    Serial.print("Base Thresholds Updated: T1=");
    Serial.print(threshold1);
    Serial.print(", T2=");
    Serial.println(threshold2);
    Serial.print("Current Dynamic Thresholds: T1=");
    Serial.print(dynamicThreshold1, 2);
    Serial.print(", T2=");
    Serial.print(dynamicThreshold2, 2);
    Serial.println("");
    Serial.println("(Dynamic thresholds adjust based on rate of change)");
    Serial.println("");
}

void resetSystem() {
    operatingMode = 0;
    manualCommand = 0;
    manipulatorState = 0;
    retreatHasBeenTriggered = false;
    retreatRequestSent = false;
    trajectoryPaused = false;
    
    analogWrite(RPWM1, 0); analogWrite(LPWM1, 0);
    analogWrite(RPWM2, 0); analogWrite(LPWM2, 0);
    analogWrite(RPWM3, 0); analogWrite(LPWM3, 0);
    
    loadCellOffset = readHX711();
    smoothedLoad = 0.0;
    
    position1 = 0; position2 = 0; position3 = 0;
    ActPos1 = 0.0; ActPos2 = 0.0; ActPos3 = 0.0;
    prevPos1 = 0.0; prevPos2 = 0.0; prevPos3 = 0.0;
    ErrPos1 = 0.0; ErrPos2 = 0.0; ErrPos3 = 0.0;
    error1 = 0.0; error2 = 0.0; error3 = 0.0;
    prevError1 = 0.0; prevError2 = 0.0; prevError3 = 0.0;
    ActVelo1 = 0.0; ActVelo2 = 0.0; ActVelo3 = 0.0;
    
    // Reset dynamic threshold variables
    prevLoad = 0.0;
    loadChangeRate = 0.0;
    loadChangeMagnitude = 0.0;
    dynamicThreshold1 = threshold1;
    dynamicThreshold2 = threshold2;
    
    // Reset admittance states
    resetAdmittance();
    
    Serial.println("System Reset: Tare & Zero OK\n");
}

void emergencyStop() {
    operatingMode = 0;
    manualCommand = 0;
    retreatHasBeenTriggered = false;
    retreatRequestSent = false;
    
    analogWrite(RPWM1, 0); analogWrite(LPWM1, 0);
    analogWrite(RPWM2, 0); analogWrite(LPWM2, 0);
    analogWrite(RPWM3, 0); analogWrite(LPWM3, 0);
    
    Serial.println("EMERGENCY_STOP");
}

//==================================================================
// MOTOR CONTROL FUNCTIONS
//==================================================================

void updateEncoders() {
    int currentState1 = digitalRead(ENC1);
    if (currentState1 > prevState1) {
        if (controlValue1 > 0) position1++;
        else if (controlValue1 < 0) position1--;
    }
    ActPos1 = position1 * 0.245;
    prevState1 = currentState1;
    
    int currentState2 = digitalRead(ENC2);
    if (currentState2 > prevState2) {
        if (controlValue2 > 0) position2++;
        else if (controlValue2 < 0) position2--;
    }
    ActPos2 = position2 * 0.245;
    prevState2 = currentState2;
    
    int currentState3 = digitalRead(ENC3);
    if (currentState3 > prevState3) {
        if (controlValue3 > 0) position3++;
        else if (controlValue3 < 0) position3--;
    }
    ActPos3 = position3 * 0.245;
    prevState3 = currentState3;
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
    
    float ErrVelo1 = Error(refVelo1, ActVelo1);
    float ErrVelo2 = Error(refVelo2, ActVelo2);
    float ErrVelo3 = Error(refVelo3, ActVelo3);
    
    float kp1_adaptive = getAdaptiveKp(kp1);
    float kp2_adaptive = getAdaptiveKp(kp2);
    float kp3_adaptive = getAdaptiveKp(kp3);
    
    refCurrent1 = CTC(ErrPos1, kp1_adaptive, ErrVelo1, kd1, refFc1, GR, kt);
    refCurrent2 = CTC(ErrPos2, kp2_adaptive, ErrVelo2, kd2, refFc2, GR, kt);
    refCurrent3 = CTC(ErrPos3, kp3_adaptive, ErrVelo3, kd3, refFc3, GR, kt);
}

void calculateCTCWithAdmittance() {
    // Modify reference positions based on admittance displacement
    // Z_adm is the compliance displacement in meters
    // IMPORTANT: SUBTRACT Z_adm to make robot retreat when force is applied
    // When external force increases → Z_adm increases → robot moves backward (compliant behavior)
    
    float refPos1_modified = refPos1 - (Z_adm * 1000.0);  // SUBTRACT for compliance
    float refPos2_modified = refPos2 - (Z_adm * 1000.0);
    float refPos3_modified = refPos3 - (Z_adm * 1000.0);
    
    // Also modify reference velocities (negative for retreat)
    float refVelo1_modified = refVelo1 - Zdot_adm;
    float refVelo2_modified = refVelo2 - Zdot_adm;
    float refVelo3_modified = refVelo3 - Zdot_adm;
    
    ErrPos1 = Error(refPos1_modified, ActPos1);
    ErrPos2 = Error(refPos2_modified, ActPos2);
    ErrPos3 = Error(refPos3_modified, ActPos3);
    
    float ErrVelo1 = Error(refVelo1_modified, ActVelo1);
    float ErrVelo2 = Error(refVelo2_modified, ActVelo2);
    float ErrVelo3 = Error(refVelo3_modified, ActVelo3);
    
    float kp1_adaptive = getAdaptiveKp(kp1);
    float kp2_adaptive = getAdaptiveKp(kp2);
    float kp3_adaptive = getAdaptiveKp(kp3);
    
    refCurrent1 = CTC(ErrPos1, kp1_adaptive, ErrVelo1, kd1, refFc1, GR, kt);
    refCurrent2 = CTC(ErrPos2, kp2_adaptive, ErrVelo2, kd2, refFc2, GR, kt);
    refCurrent3 = CTC(ErrPos3, kp3_adaptive, ErrVelo3, kd3, refFc3, GR, kt);
}

void calculatePD() {
    error1 = Error(refCurrent1, ActCurrent1);
    error2 = Error(refCurrent2, ActCurrent2);
    error3 = Error(refCurrent3, ActCurrent3);
    
    float DerError1 = (error1 - prevError1) / 0.1;
    float DerError2 = (error2 - prevError2) / 0.1;
    float DerError3 = (error3 - prevError3) / 0.1;
    
    controlValue1 = PD(error1, kpc1, DerError1, kdc1);
    controlValue2 = PD(error2, kpc2, DerError2, kdc2);
    controlValue3 = PD(error3, kpc3, DerError3, kdc3);
    
    prevError1 = error1;
    prevError2 = error2;
    prevError3 = error3;
    
    // Apply load-based speed scaling (only in forward mode)
    if (operatingMode == 1) {
        float loadScale = getLoadScaling();
        controlValue1 = constrain(controlValue1 * loadScale, -255, 255);
        controlValue2 = constrain(controlValue2 * loadScale, -255, 255);
        controlValue3 = constrain(controlValue3 * loadScale, -255, 255);
    } else {
        controlValue1 = constrain(controlValue1, -255, 255);
        controlValue2 = constrain(controlValue2, -255, 255);
        controlValue3 = constrain(controlValue3, -255, 255);
    }
}

void applyMotorControl() {
    // Motor 1
    if (ErrPos1 > 0) {
        analogWrite(RPWM1, abs(controlValue1));
        analogWrite(LPWM1, 0);
    } else if (ErrPos1 < 0) {
        analogWrite(RPWM1, 0);
        analogWrite(LPWM1, abs(controlValue1));
    } else {
        analogWrite(RPWM1, 0);
        analogWrite(LPWM1, 0);
    }
    
    // Motor 2
    if (ErrPos2 > 0) {
        analogWrite(RPWM2, abs(controlValue2));
        analogWrite(LPWM2, 0);
    } else if (ErrPos2 < 0) {
        analogWrite(RPWM2, 0);
        analogWrite(LPWM2, abs(controlValue2));
    } else {
        analogWrite(RPWM2, 0);
        analogWrite(LPWM2, 0);
    }
    
    // Motor 3
    if (ErrPos3 > 0) {
        analogWrite(RPWM3, abs(controlValue3));
        analogWrite(LPWM3, 0);
    } else if (ErrPos3 < 0) {
        analogWrite(RPWM3, 0);
        analogWrite(LPWM3, abs(controlValue3));
    } else {
        analogWrite(RPWM3, 0);
        analogWrite(LPWM3, 0);
    }
}

void stopAllMotors() {
    analogWrite(RPWM1, 0); analogWrite(LPWM1, 0);
    analogWrite(RPWM2, 0); analogWrite(LPWM2, 0);
    analogWrite(RPWM3, 0); analogWrite(LPWM3, 0);
}

void manualModeControl() {
    int effectiveSpeed = MANUAL_SPEED;
    
    if (ENABLE_ADAPTIVE_MANUAL) {
        float loadScale = getLoadScaling();
        effectiveSpeed = MANUAL_SPEED * loadScale;
    }
    
    if (manualCommand == 1) {
        analogWrite(RPWM1, effectiveSpeed); analogWrite(LPWM1, 0);
        analogWrite(RPWM2, effectiveSpeed); analogWrite(LPWM2, 0);
        analogWrite(RPWM3, effectiveSpeed); analogWrite(LPWM3, 0);
    } else if (manualCommand == 2) {
        analogWrite(RPWM1, 0); analogWrite(LPWM1, effectiveSpeed);
        analogWrite(RPWM2, 0); analogWrite(LPWM2, effectiveSpeed);
        analogWrite(RPWM3, 0); analogWrite(LPWM3, effectiveSpeed);
    } else {
        stopAllMotors();
    }
}

//==================================================================
// SETUP
//==================================================================

void setup() {
    Serial.begin(115200);
    
    // Motor PWM pins
    pinMode(RPWM1, OUTPUT);
    pinMode(LPWM1, OUTPUT);
    pinMode(RPWM2, OUTPUT);
    pinMode(LPWM2, OUTPUT);
    pinMode(RPWM3, OUTPUT);
    pinMode(LPWM3, OUTPUT);
    
    // Encoder pins
    pinMode(ENC1, INPUT);
    pinMode(ENC2, INPUT);
    pinMode(ENC3, INPUT);
    
    // Current sensor pins
    pinMode(CurrSen1, INPUT);
    pinMode(CurrSen2, INPUT);
    pinMode(CurrSen3, INPUT);
    
    // Load cell pins
    pinMode(LOADCELL_DOUT_PIN, INPUT);
    pinMode(LOADCELL_SCK_PIN, OUTPUT);
    
    while (!Serial) {
        ;
    }
    
    Serial.println("===========================================");
    Serial.println("  3-RPS Parallel Robot Control System");
    Serial.println("  Admittance Control Version");
    Serial.println("  Z(s) = F / (101.7s² + 500s + 614.1)");
    Serial.println("===========================================");
    Serial.println("");
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
        char receivedChar = Serial.read();
        receivedData += receivedChar;
        
        if (receivedChar == '\n') {
            receivedData.trim();
            
            // Forward trajectory command (S)
            if (receivedData.startsWith("S")) {
                operatingMode = 1;
                retreatHasBeenTriggered = false;
                retreatRequestSent = false;
                manipulatorState = 0;
                parseTrajectoryCommand(receivedData, false);
            }
            // Retreat trajectory command (R with positions)
            else if (receivedData.startsWith("R") && receivedData.indexOf(',') > 0) {
                operatingMode = 2;
                manipulatorState = 0;
                parseTrajectoryCommand(receivedData, true);
            }
            // Retreat finished notification
            else if (receivedData == "RETREAT_COMPLETE") {
                operatingMode = 0;
                manualCommand = 0;
                retreatHasBeenTriggered = false;
                retreatRequestSent = false;
                stopAllMotors();
                Serial.println("ACK_RETREAT_COMPLETE");
            }
            // Tare and reset (X)
            else if (receivedData.startsWith("X")) {
                resetSystem();
            }
            // Emergency stop (E)
            else if (receivedData.startsWith("E")) {
                emergencyStop();
            }
            // Manual forward (1)
            else if (receivedData == "1") {
                operatingMode = 0;
                manualCommand = 1;
                retreatHasBeenTriggered = false;
                retreatRequestSent = false;
            }
            // Manual backward (2)
            else if (receivedData == "2") {
                operatingMode = 0;
                manualCommand = 2;
                retreatHasBeenTriggered = false;
                retreatRequestSent = false;
            }
            // Manual stop (0)
            else if (receivedData == "0") {
                operatingMode = 0;
                manualCommand = 0;
            }
            // Set outer loop gains (K)
            else if (receivedData.startsWith("K")) {
                operatingMode = 0;
                manualCommand = 0;
                parseOuterLoopGains(receivedData);
            }
            // Set inner loop gains (P)
            else if (receivedData.startsWith("P")) {
                operatingMode = 0;
                manualCommand = 0;
                parseInnerLoopGains(receivedData);
            }
            // Set thresholds (T)
            else if (receivedData.startsWith("T")) {
                operatingMode = 0;
                manualCommand = 0;
                parseThresholds(receivedData);
            }
            // Admittance control commands
            else if (receivedData == "ADMITTANCE_ON") {
                admittanceEnabled = true;
                Serial.println("Admittance Control ENABLED");
            }
            else if (receivedData == "ADMITTANCE_OFF") {
                admittanceEnabled = false;
                Serial.println("Admittance Control DISABLED");
            }
            else if (receivedData == "ADMITTANCE_RESET") {
                resetAdmittance();
                Serial.println("Admittance states RESET");
            }
            else if (receivedData.startsWith("ADM") && receivedData.indexOf(',') > 0) {
                parseAdmittanceParams(receivedData);
            }
            else if (receivedData.startsWith("PAUSE_THRESHOLD")) {
                // Format: PAUSE_THRESHOLD,5.0
                String data = receivedData;
                data.replace("PAUSE_THRESHOLD,", "");
                float newThreshold = data.toFloat();
                // Can't modify const, so this is just for future implementation
                Serial.print("Pause threshold would be: ");
                Serial.println(newThreshold);
                Serial.println("(Recompile to change FORCE_PAUSE_THRESHOLD)");
            }
            else if (receivedData == "ADMITTANCE_STATUS") {
                Serial.println("\n=== Admittance Control Status ===");
                Serial.print("Enabled: ");
                Serial.println(admittanceEnabled ? "YES" : "NO");
                Serial.println("\nTransfer Function Parameters:");
                Serial.print("M (Virtual Mass): "); Serial.print(M_adm); Serial.println(" kg");
                Serial.print("B (Virtual Damping): "); Serial.print(B_adm); Serial.println(" N·s/m");
                Serial.print("K (Virtual Stiffness): "); Serial.print(K_adm); Serial.println(" N/m");
                Serial.print("Gain Multiplier: "); Serial.println(admittanceGain);
                Serial.println("\nCurrent States:");
                Serial.print("Z (Displacement): "); Serial.print(Z_adm * 1000, 4); Serial.println(" mm");
                Serial.print("Zdot (Velocity): "); Serial.print(Zdot_adm * 1000, 4); Serial.println(" mm/s");
                Serial.print("Zddot (Acceleration): "); Serial.print(Zddot_adm * 1000, 4); Serial.println(" mm/s²");
                Serial.print("External Force: "); Serial.print(latestValidLoad, 2); Serial.println(" N");
                Serial.println("\nCompliance Analysis:");
                float static_compliance = 1000.0 / K_adm;  // mm/N
                Serial.print("Static Compliance: "); Serial.print(static_compliance, 2); Serial.println(" mm/N");
                Serial.println("=================================\n");
            }
            
            receivedData = "";
        }
    }
    
    //================================================================
    // AUTO MODE EXECUTION (FORWARD OR RETREAT)
    //================================================================
    
    if (operatingMode == 1 || operatingMode == 2) {
        
        // Load cell monitoring (only in forward mode)
        if (operatingMode == 1 && currentTime - lastLoadTime >= loadCellInterval) {
            if (retreatHasBeenTriggered) {
                manipulatorState = 1;
            } else {
                long effectiveValue = readHX711() - loadCellOffset;
                float rawLoad = effectiveValue / 10000.0;
                
                // FORCE TO ZERO IF NEGATIVE (prevent sensor noise)
                if (rawLoad < 0.0) {
                    rawLoad = 0.0;
                }
                
                // Optional: Add upper limit to prevent extreme values
                if (rawLoad > 100.0) {  // Max 100N
                    rawLoad = 100.0;
                }
                
                latestValidLoad = rawLoad;
                
                // Calculate time step for rate of change calculation
                float dt = loadCellInterval / 1000.0;  // Convert to seconds
                
                // Update dynamic thresholds based on rate of change
                updateDynamicThresholds(latestValidLoad, dt);
                
                // Use dynamic thresholds for comparison
                int roundValue = round(latestValidLoad);
                
                if (roundValue >= dynamicThreshold2) {
                    manipulatorState = 1;
                    retreatHasBeenTriggered = true;
                    
                    if (!retreatRequestSent) {
                        Serial.println("RETREAT");
                        retreatRequestSent = true;
                    }
                } else if (roundValue >= dynamicThreshold1) {
                    manipulatorState = 1;
                } else {
                    manipulatorState = 0;
                }
            }
            lastLoadTime = currentTime;
        }
        
        // Admittance control update (menggunakan F_external dari load cell)
        if (admittanceEnabled && operatingMode == 1 && 
            currentTime - lastAdmittanceTime >= admittanceUpdateInterval) {
            
            // External force from load cell (dalam Newton)
            float F_external = latestValidLoad;
            
            // Time step dalam detik
            float dt = admittanceUpdateInterval / 1000.0;
            
            // Update admittance dynamics
            updateAdmittanceControl(F_external, dt);
            
            // CHECK IF TRAJECTORY SHOULD BE PAUSED
            if (F_external > FORCE_PAUSE_THRESHOLD && !trajectoryPaused) {
                // Start pausing trajectory
                trajectoryPaused = true;
                
                // Save current trajectory reference (freeze it)
                pausedRefPos1 = refPos1;
                pausedRefPos2 = refPos2;
                pausedRefPos3 = refPos3;
                pausedRefVelo1 = refVelo1;
                pausedRefVelo2 = refVelo2;
                pausedRefVelo3 = refVelo3;
                pausedRefFc1 = refFc1;
                pausedRefFc2 = refFc2;
                pausedRefFc3 = refFc3;
                
                // CRITICAL: Tell PC to STOP sending trajectory
                Serial.println("PAUSE_TRAJECTORY");
            }
            // CHECK IF TRAJECTORY CAN RESUME
            else if (F_external <= FORCE_PAUSE_THRESHOLD && trajectoryPaused) {
                // Resume trajectory from paused position
                trajectoryPaused = false;
                
                // Keep the paused reference (don't jump)
                refPos1 = pausedRefPos1;
                refPos2 = pausedRefPos2;
                refPos3 = pausedRefPos3;
                refVelo1 = 0.0;  // Start with zero velocity
                refVelo2 = 0.0;
                refVelo3 = 0.0;
                refFc1 = pausedRefFc1;
                refFc2 = pausedRefFc2;
                refFc3 = pausedRefFc3;
                
                // CRITICAL: Tell PC to RESUME sending trajectory
                Serial.println("RESUME_TRAJECTORY");
            }
            
            lastAdmittanceTime = currentTime;
        }
        
        // Encoder reading
        if (currentTime - lastEncTime >= encoderInterval && manipulatorState != 1) {
            updateEncoders();
            lastEncTime = currentTime;
        }
        
        // Control calculations (only in running state)
        if (manipulatorState == 0) {
            
            // Velocity calculation
            if (currentTime - lastVeloTime >= veloInterval) {
                updateVelocities();
                lastVeloTime = currentTime;
            }
            
            // CTC (outer loop) calculation
            if (currentTime - lastCTCCalcTime >= CTCcalculationInterval) {
                if (admittanceEnabled && operatingMode == 1) {
                    calculateCTCWithAdmittance();
                } else {
                    calculateCTC();
                }
                lastCTCCalcTime = currentTime;
            }
            
            // PD (inner loop) calculation
            if (currentTime - lastPDCalcTime >= PDcalculationInterval) {
                calculatePD();
                lastPDCalcTime = currentTime;
            }
        }
        
        // Motor control execution
        if (manipulatorState == 0) {
            applyMotorControl();
        } else {
            stopAllMotors();
        }
        
        // Status reporting
        if (currentTime - lastPrnTime >= loadCellInterval) {
            String statusStr;
            String modeStr;
            
            if (operatingMode == 1) {
                modeStr = "forward";
            } else {
                modeStr = "retreat";
            }
            
            if (manipulatorState == 0) {
                statusStr = "running";
            } else if (manipulatorState == 1) {
                statusStr = "paused";
            } else if (manipulatorState == 2) {
                statusStr = "retreating";
            }
            
            float currentScale = getLoadScaling();
            Serial.print("status:");
            Serial.print(statusStr);
            Serial.print(",mode:");
            Serial.print(modeStr);
            Serial.print(",load:");
            Serial.print(latestValidLoad, 2);
            Serial.print(",scale:");
            Serial.print(currentScale, 2);
            Serial.print(",thresh1:");
            Serial.print(dynamicThreshold1, 2);
            Serial.print(",thresh2:");
            Serial.print(dynamicThreshold2, 2);
            Serial.print(",rate:");
            Serial.print(loadChangeRate, 2);
            Serial.print(",mag:");
            Serial.print(loadChangeMagnitude, 2);
            Serial.print(",pos:");
            Serial.print(ActPos1, 2);
            Serial.print(",");
            Serial.print(ActPos2, 2);
            Serial.print(",");
            Serial.print(ActPos3, 2);
            
            // Add admittance info if enabled
            if (admittanceEnabled && operatingMode == 1) {
                Serial.print(",Z_adm:");
                Serial.print(Z_adm * 1000, 3);  // mm
                Serial.print(",Zdot:");
                Serial.print(Zdot_adm * 1000, 3);  // mm/s
                Serial.print(",Zddot:");
                Serial.print(Zddot_adm * 1000, 3);  // mm/s²
                Serial.print(",traj_paused:");
                Serial.print(trajectoryPaused ? "1" : "0");
            }
            
            Serial.println("");
            
            lastPrnTime = currentTime;
        }
        
    }
    
    //================================================================
    // MANUAL MODE EXECUTION (WITH ADAPTIVE CONTROL)
    //================================================================
    
    else {
        // Read load cell even in manual mode
        if (currentTime - lastLoadTime >= loadCellInterval && ENABLE_ADAPTIVE_MANUAL) {
            long effectiveValue = readHX711() - loadCellOffset;
            float rawLoad = effectiveValue / 10000.0;
            
            // FORCE TO ZERO IF NEGATIVE
            if (rawLoad < 0.0) {
                rawLoad = 0.0;
            }
            
            // Optional: Add upper limit
            if (rawLoad > 100.0) {
                rawLoad = 100.0;
            }
            
            latestValidLoad = rawLoad;
            lastLoadTime = currentTime;
        }
        
        manualModeControl();
    }
}