//==================================================================
// SISTEM KONTROL REHABILITASI - MODULAR VERSION
// With Multi-Trajectory Selection + Cycle Counter + Admittance Pause
//==================================================================

#include <iostream>
#include <thread>
#include <chrono>
#include <cerrno>

#include "include/Config.h"
#include "include/StateMachine.h"
#include "include/TrajectoryManager.h"
#include "include/SerialHandler.h"
#include "include/ModbusHandler.h"
#include "include/GraphManager.h"
#include "include/ControlHandler.h"
#include "include/StateHandlers.h"

#include <boost/asio.hpp>
#include <modbus/modbus.h>

using namespace boost::asio;

int main() {
    // Initialize trajectory manager
    TrajectoryManager trajectoryManager;
    if (!trajectoryManager.loadAllTrajectoryData("data")) {
        std::cerr << "Fatal Error: Cannot load trajectory data. Exiting..." << std::endl;
        return 1;
    }
    
    // Default: Trajectory 1 active
    trajectoryManager.switchTrajectory(1);
    
    // Initialize IO context and serial
    io_context io;
    SerialHandler serialHandler(io);
    if (!serialHandler.initialize(DEFAULT_SERIAL_PORT, SERIAL_BAUD_RATE)) {
        return 1;
    }
    serialHandler.sendCommand("0");
    
    // Initialize Modbus
    ModbusHandler modbusHandler;
    if (!modbusHandler.initialize(MODBUS_HOST, MODBUS_PORT, MODBUS_SLAVE_ID)) {
        std::cerr << "Error initializing Modbus. Exiting..." << std::endl;
        return 1;
    }
    
    // Initialize graph manager
    GraphManager graphManager(modbusHandler, trajectoryManager);
    
    // Initialize control handler
    ControlHandler controlHandler(modbusHandler, serialHandler, trajectoryManager, graphManager);
    
    // System state
    SystemState currentState = SystemState::IDLE;
    std::string arduinoFeedbackState = "running";
    int t_controller = 0;
    int t_grafik = 0;
    bool animasi_grafik_berjalan = false;
    bool messageSend = false;
    int last_threshold1 = -1;
    int last_threshold2 = -1;
    
    auto lastTraTime = std::chrono::steady_clock::now();
    auto lastGrafikTime = std::chrono::steady_clock::now();
    auto delayStartTime = std::chrono::steady_clock::now();
    
    std::cout << "\n===========================================";
    std::cout << "\n  SISTEM KONTROL REHABILITASI";
    std::cout << "\n  Multi-Trajectory + Cycle Counter";
    std::cout << "\n  + Admittance Control Support";
    std::cout << "\n  + Telemetri Arduino -> console mini PC";
    std::cout << "\n  Modular Version";
    std::cout << "\n===========================================" << std::endl;
    
    while (true) {
        uint8_t query[MODBUS_TCP_MAX_ADU_LENGTH];
        int rc = modbusHandler.receive(query, MODBUS_TCP_MAX_ADU_LENGTH);
        
        if (rc > 0) {
            modbus_mapping_t* mb_mapping = modbusHandler.getMapping();
            
            // Emergency stop handling
            if (mb_mapping->tab_registers[ModbusAddr::EMERGENCY] == 1) {
                if (currentState != SystemState::EMERGENCY_STOP) {
                    currentState = SystemState::EMERGENCY_STOP;
                    animasi_grafik_berjalan = false;
                    serialHandler.sendCommand("E");
                    std::cout << "\n!!! EMERGENCY STOP DIAKTIFKAN !!!" << std::endl;
                }
                mb_mapping->tab_registers[ModbusAddr::EMERGENCY] = 0;
            }
            
            // Reset from emergency
            if (mb_mapping->tab_registers[ModbusAddr::RESET] == 1 && 
                currentState == SystemState::EMERGENCY_STOP) {
                currentState = SystemState::RESETTING;
                serialHandler.sendCommand("X");
                std::cout << "\nSistem di-reset dari Emergency Stop." << std::endl;
                mb_mapping->tab_registers[ModbusAddr::RESET] = 0;
            }
            
            // State machine
            switch (currentState) {
                case SystemState::IDLE:
                    currentState = handleIdleState(modbusHandler, controlHandler,
                                                  animasi_grafik_berjalan, t_controller, t_grafik,
                                                  lastTraTime, lastGrafikTime,
                                                  last_threshold1, last_threshold2);
                    break;
                    
                case SystemState::RESETTING:
                    currentState = handleResettingState(modbusHandler);
                    break;
                    
                case SystemState::AUTO_RETREAT:
                    currentState = handleAutoRetreatState(modbusHandler, serialHandler, controlHandler,
                                                         arduinoFeedbackState, messageSend, lastTraTime);
                    break;
                    
                case SystemState::POST_REHAB_DELAY:
                    currentState = handlePostRehabDelay(delayStartTime, serialHandler, modbusHandler,
                                                       controlHandler, animasi_grafik_berjalan,
                                                       t_controller, t_grafik, lastTraTime, lastGrafikTime);
                    break;
                    
                case SystemState::AUTO_REHAB:
                case SystemState::EMERGENCY_STOP:
                    break;
            }
            
            modbusHandler.reply(query, rc);
        }
        
        // === Process Arduino feedback (SINGLE READ) ===
        // ControlHandler will handle BOTH load cell AND pause/resume signals
        controlHandler.processArduinoFeedback(arduinoFeedbackState, currentState, t_controller);
        
        // === Process auto rehab (ACK-based, pause-aware) ===
        if (currentState == SystemState::AUTO_REHAB) {
            // processAutoRehab menangani:
            // - Cek admittance pause secara internal
            // - Tunggu WAYPOINT_REACHED (ACK-based) sebelum advance
            // - Fase ramp-up sebelum gait utama
            controlHandler.processAutoRehab(currentState, t_controller, t_grafik,
                                          animasi_grafik_berjalan, lastTraTime, delayStartTime);
        }

        
        // === Process retreat sequence ===
        if (currentState == SystemState::AUTO_RETREAT) {
            if (controlHandler.isRetreatActive()) {
                controlHandler.processRetreatSequence(lastTraTime);
            } else if (controlHandler.isAutoReturnToIdlePending()) {
                controlHandler.completeAutoReturnToIdle();
                currentState = SystemState::IDLE;
                messageSend = false;
            }
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    
    // Cleanup (never reached, but good practice)
    modbusHandler.close();
    serialHandler.close();
    
    return 0;
}