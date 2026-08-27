#ifndef CONTROLHANDLER_H
#define CONTROLHANDLER_H

#include <modbus/modbus.h>
#include <boost/asio.hpp>
#include <chrono>
#include <fstream>
#include <string>
#include <iomanip>
#include "StateMachine.h"
#include "ModbusHandler.h"
#include "TrajectoryManager.h"
#include "SerialHandler.h"
#include "GraphManager.h"

using namespace boost::asio;

class ControlHandler {
public:
    ControlHandler(ModbusHandler& modbus, SerialHandler& serial, 
                   TrajectoryManager& trajectory, GraphManager& graph);
    
    // Manual control
    void handleManualControl();
    
    // Calibration
    void handleCalibration(bool& animasi_grafik);
    
    // Rehab cycle
    void startRehabCycle(bool& animasi_grafik, int& t_controller, int& t_grafik,
                        std::chrono::steady_clock::time_point& lastTraTime,
                        std::chrono::steady_clock::time_point& lastGrafikTime);
    
    // Threshold
    void updateThresholds(int& last_thresh1, int& last_thresh2);
    
    // Trajectory selection
    void handleTrajectorySelection();
    
    // Arduino feedback
    void processArduinoFeedback(std::string& arduinoFeedbackState, 
                                SystemState& currentState, int t_controller);
    
    // Retreat control
    void startRetreatSequence(int currentIndex);
    void processRetreatSequence(std::chrono::steady_clock::time_point& lastTraTime);
    
    // Auto rehab processing
    void processAutoRehab(SystemState& currentState, int& t_controller, int& t_grafik,
                         bool& animasi_grafik, 
                         std::chrono::steady_clock::time_point& lastTraTime,
                         std::chrono::steady_clock::time_point& delayStartTime);
    
    // ACK-based waypoint control
    void notifyWaypointReached();          // Dipanggil saat WAYPOINT_REACHED diterima
    bool isWaitingForWaypoint() const { return waitingForWaypoint; }
    
    // Cycle management
    int getTargetCycle() const { return target_cycle; }
    int getCurrentCycle() const { return current_cycle; }
    void resetCycle() { current_cycle = 0; target_cycle = 1; }
    void advanceToNextCycle(bool& animasi_grafik, int& t_controller, int& t_grafik);
    void startAutoReturnToZero(int controllerSteps);
    bool isAutoReturnToIdlePending() const { return autoReturnToIdle; }
    void completeAutoReturnToIdle();
    
    // Retreat status
    bool isRetreatActive() const { return retreatActive; }

private:
    ModbusHandler& modbusHandler;
    SerialHandler& serialHandler;
    TrajectoryManager& trajectoryManager;
    GraphManager& graphManager;
    
    // Cycle counter
    int target_cycle;
    int current_cycle;
    
    // Retreat control
    int retreatIndex;
    int retreatTargetIndex;
    bool retreatActive;
    int lastForwardIndex;
    bool autoReturnToIdle;
    
    // ACK-based waypoint control
    bool waitingForWaypoint;    // Tunggu WAYPOINT_REACHED sebelum kirim titik berikutnya
    
    // Ramp-up phase (traversal index 1 → gaitStart sebelum gait utama)
    bool rampUpPhase;           // true = sedang dalam fase ramp-up
    int  rampUpIndex;           // Current index saat ramp-up
    
    // Helper functions
    void sendControllerData(int t);
    void sendRetreatData(int index);
    int clampRetreatIndex(int controllerSteps) const;
    
    // Data Logging
    void initLogger();
    std::ofstream logFile;
};

#endif // CONTROLHANDLER_H

