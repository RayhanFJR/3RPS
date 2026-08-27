#include "StateHandlers.h"
#include "ControlHandler.h"
#include "ModbusHandler.h"
#include "SerialHandler.h"
#include "Config.h"
#include "StateMachine.h"
#include <iostream>
#include <chrono>

SystemState handleIdleState(ModbusHandler& modbus,
                           ControlHandler& control, bool& animasi_grafik,
                           int& t_controller, int& t_grafik,
                           std::chrono::steady_clock::time_point& lastTraTime,
                           std::chrono::steady_clock::time_point& lastGrafikTime,
                           int& last_thresh1, int& last_thresh2) {
    control.handleManualControl();
    control.handleTrajectorySelection();
    
    modbus_mapping_t* mb_mapping = modbus.getMapping();
    if (mb_mapping == nullptr) return SystemState::IDLE;
    
    if (mb_mapping->tab_registers[ModbusAddr::CALIBRATE] == 1) {
        control.handleCalibration(animasi_grafik);
    }
    
    if (mb_mapping->tab_registers[ModbusAddr::START] == 1) {
        control.startRehabCycle(animasi_grafik, t_controller, t_grafik, 
                               lastTraTime, lastGrafikTime);
        return SystemState::AUTO_REHAB;
    }
    
    control.updateThresholds(last_thresh1, last_thresh2);
    
    return SystemState::IDLE;
}

SystemState handleResettingState(ModbusHandler& modbus) {
    modbus_mapping_t* mb_mapping = modbus.getMapping();
    if (mb_mapping == nullptr) return SystemState::RESETTING;
    
    if (mb_mapping->tab_registers[ModbusAddr::CALIBRATE] == 1) {
        std::cout << "\nSistem Siap. Kembali ke mode IDLE." << std::endl;
        mb_mapping->tab_registers[ModbusAddr::CALIBRATE] = 0;
        mb_mapping->tab_registers[ModbusAddr::START] = 0;
        return SystemState::IDLE;
    }
    return SystemState::RESETTING;
}

SystemState handleAutoRetreatState(ModbusHandler& modbus, SerialHandler& serial,
                                   ControlHandler& control,
                                   std::string& arduinoFeedbackState, 
                                   bool& messageSend,
                                   std::chrono::steady_clock::time_point& lastTraTime) {
    
    if (control.isRetreatActive()) {
        control.processRetreatSequence(lastTraTime);
        return SystemState::AUTO_RETREAT;
    }
    
    if (control.isAutoReturnToIdlePending()) {
        control.completeAutoReturnToIdle();
        messageSend = false;
        return SystemState::IDLE;
    }
    
    if (!control.isRetreatActive() && !messageSend) {
        std::cout << "\n=== RETREAT COMPLETED ===" << std::endl;
        std::cout << "System stopped. Ready for RESET." << std::endl;
        messageSend = true;
    }
    
    modbus_mapping_t* mb_mapping = modbus.getMapping();
    if (mb_mapping == nullptr) return SystemState::AUTO_RETREAT;
    
    if (mb_mapping->tab_registers[ModbusAddr::RESET] == 1) {
        serial.sendCommand("X");
        std::cout << "\nSistem di-reset." << std::endl;
        mb_mapping->tab_registers[ModbusAddr::RESET] = 0;
        arduinoFeedbackState = "running";
        messageSend = false;
        return SystemState::RESETTING;
    }
    
    return SystemState::AUTO_RETREAT;
}

SystemState handlePostRehabDelay(std::chrono::steady_clock::time_point& delayStartTime,
                                SerialHandler& serial, ModbusHandler& modbus,
                                ControlHandler& control,
                                bool& animasi_grafik, int& t_controller, int& t_grafik,
                                std::chrono::steady_clock::time_point& lastTraTime,
                                std::chrono::steady_clock::time_point& lastGrafikTime) {
    if ((std::chrono::steady_clock::now() - delayStartTime) >= 
        std::chrono::seconds(POST_REHAB_DELAY_SEC)) {
        
        int current_cycle = control.getCurrentCycle();
        int target_cycle = control.getTargetCycle();
        
        // Check if more cycles remaining
        if (current_cycle < target_cycle) {
            control.advanceToNextCycle(animasi_grafik, t_controller, t_grafik);
            lastTraTime = std::chrono::steady_clock::now();
            lastGrafikTime = std::chrono::steady_clock::now();
            
            return SystemState::AUTO_REHAB;
        } 
        else {
            // Semua cycle selesai — aktifkan retreat penuh ke titik awal gait
            std::cout << "\n=== SEMUA CYCLE SELESAI ===" << std::endl;
            std::cout << "Total cycle completed: " << current_cycle << std::endl;
            std::cout << "Memulai retreat otomatis ke posisi awal..." << std::endl;

            animasi_grafik = false;

            modbus_mapping_t* mb_mapping = modbus.getMapping();
            if (mb_mapping != nullptr) {
                mb_mapping->tab_registers[ModbusAddr::COMMAND_REG] = 0;
                mb_mapping->tab_registers[ModbusAddr::START] = 0;
            }

            control.startAutoReturnToZero(t_controller);
            lastTraTime = std::chrono::steady_clock::now();

            return SystemState::AUTO_RETREAT;
        }
    }
    return SystemState::POST_REHAB_DELAY;
}

