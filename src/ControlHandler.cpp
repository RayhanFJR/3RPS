#include "ControlHandler.h"
#include "Config.h"
#include "StateMachine.h"
#include <iostream>
#include <sstream>
#include <chrono>

ControlHandler::ControlHandler(ModbusHandler& modbus, SerialHandler& serial,
                               TrajectoryManager& trajectory, GraphManager& graph)
    : modbusHandler(modbus), serialHandler(serial), 
      trajectoryManager(trajectory), graphManager(graph),
      target_cycle(1), current_cycle(0),
      retreatIndex(0), retreatTargetIndex(0), retreatActive(false), lastForwardIndex(0),
      autoReturnToIdle(false) {
    initLogger();
}

void ControlHandler::initLogger() {
    // Pastikan folder exist atau minimal log bisa terbuat (biasanya perlu mkdir, tapi kita asumsikan folder data/logs/ ada)
    logFile.open(std::string(LOG_DIR) + std::string(LOG_FILENAME), std::ios::out | std::ios::app);
    if (!logFile.is_open()) {
        std::cerr << "Warning: Could not open log file at " << LOG_DIR << LOG_FILENAME << ". Creating in current dir..." << std::endl;
        logFile.open(std::string(LOG_FILENAME), std::ios::out | std::ios::app);
    }
    if (logFile.is_open()) {
        std::cout << "Data logger initialized." << std::endl;
    }
}

void ControlHandler::handleManualControl() {
    modbus_mapping_t* mb_mapping = modbusHandler.getMapping();
    if (mb_mapping == nullptr) return;
    
    if (mb_mapping->tab_registers[ModbusAddr::MANUAL_MAJU] == 1) {
        serialHandler.sendCommand("1");
        mb_mapping->tab_registers[ModbusAddr::MANUAL_MAJU] = 0;
    } else if (mb_mapping->tab_registers[ModbusAddr::MANUAL_MUNDUR] == 1) {
        serialHandler.sendCommand("2");
        mb_mapping->tab_registers[ModbusAddr::MANUAL_MUNDUR] = 0;
    } else if (mb_mapping->tab_registers[ModbusAddr::MANUAL_STOP] == 1) {
        serialHandler.sendCommand("0");
        mb_mapping->tab_registers[ModbusAddr::MANUAL_STOP] = 0;
    }
}

void ControlHandler::handleCalibration(bool& animasi_grafik) {
    serialHandler.sendCommand("X");
    std::cout << "\nPerintah Kalibrasi & Reset Grafik dikirim." << std::endl;
    
    animasi_grafik = false;
    graphManager.resetGraphData();
    modbusHandler.writeRegister(ModbusAddr::CALIBRATE, 0);
}

void ControlHandler::startRehabCycle(bool& animasi_grafik, int& t_controller, int& t_grafik,
                                     std::chrono::steady_clock::time_point& lastTraTime,
                                     std::chrono::steady_clock::time_point& lastGrafikTime) {
    modbus_mapping_t* mb_mapping = modbusHandler.getMapping();
    if (mb_mapping == nullptr) return;
    
    target_cycle = mb_mapping->tab_registers[ModbusAddr::JUMLAH_CYCLE];
    if (target_cycle < 1) target_cycle = 1;
    
    current_cycle = 1;
    
    std::cout << "\n=== MEMULAI REHABILITASI ===" << std::endl;
    std::cout << "Trajectory: " << trajectoryManager.getActiveTrajectory() << std::endl;
    std::cout << "Target Cycle: " << target_cycle << std::endl;
    std::cout << "Current Cycle: " << current_cycle << "/" << target_cycle << std::endl;
    std::cout << "Main gait range: " << trajectoryManager.getGaitStartIndex() 
              << " to " << trajectoryManager.getGaitEndIndex() << std::endl;
    
    graphManager.loadTrajectoryData();
    graphManager.clearChannel1Data();
    modbusHandler.writeFloat(ModbusAddr::REALTIME_LOAD_CELL, 0.0f);
    
    mb_mapping->tab_registers[ModbusAddr::COMMAND_REG] = 1;
    animasi_grafik = true;
    t_controller = 0;
    // Mulai dari gaitStartIndex agar hanya gait utama yang dijalankan
    t_grafik = trajectoryManager.getGaitStartIndex();
    
    // Set animasi counter sesuai offset gait terhadap data grafik
    int grafik_start = trajectoryManager.getGraphStartIndex();
    int gait_start = trajectoryManager.getGaitStartIndex();
    int initial_hmi_index = gait_start - grafik_start;
    graphManager.setAnimationCounter(initial_hmi_index);
    
    retreatActive = false;
    retreatIndex = 0;
    lastForwardIndex = 0;
    
    // Reset pause state saat start
    serialHandler.resetPauseState();
    
    lastTraTime = std::chrono::steady_clock::now();
    lastGrafikTime = std::chrono::steady_clock::now();
    
    mb_mapping->tab_registers[ModbusAddr::START] = 0;
}

void ControlHandler::advanceToNextCycle(bool& animasi_grafik, int& t_controller, int& t_grafik) {
    if (current_cycle < target_cycle) {
        current_cycle++;
    }
    
    std::cout << "\n=== MELANJUTKAN KE CYCLE " << current_cycle
              << "/" << target_cycle << " ===" << std::endl;
    
    t_controller = 0;
    // Mulai dari gaitStartIndex agar hanya gait utama yang dijalankan
    t_grafik = trajectoryManager.getGaitStartIndex();
    animasi_grafik = true;
    
    graphManager.clearChannel1Data();
    graphManager.resetAnimationCounter();
    
    // Set animasi counter sesuai offset gait terhadap data grafik
    int grafik_start = trajectoryManager.getGraphStartIndex();
    int gait_start = trajectoryManager.getGaitStartIndex();
    int initial_hmi_index = gait_start - grafik_start;
    graphManager.setAnimationCounter(initial_hmi_index);
    
    modbusHandler.writeFloat(ModbusAddr::REALTIME_LOAD_CELL, 0.0f);
    
    // Reset pause state untuk cycle baru
    serialHandler.resetPauseState();
}

int ControlHandler::clampRetreatIndex(int controllerSteps) const {
    int gaitStart = trajectoryManager.getGaitStartIndex();
    int gaitEnd = trajectoryManager.getGaitEndIndex();
    int actualIndex = gaitStart + controllerSteps - 1;
    
    if (actualIndex < gaitStart) actualIndex = gaitStart;
    if (actualIndex >= gaitEnd) actualIndex = gaitEnd - 1;
    return actualIndex;
}

void ControlHandler::startAutoReturnToZero(int controllerSteps) {
    if (retreatActive) return;
    
    int retreatStartIndex = clampRetreatIndex(controllerSteps);
    startRetreatSequence(retreatStartIndex);
    autoReturnToIdle = true;
    
    std::cout << "\n=== MENGEMBALIKAN POSISI KE TITIK AWAL ===" << std::endl;
    std::cout << "Mulai mundur dari index: " << retreatStartIndex << std::endl;
}

void ControlHandler::completeAutoReturnToIdle() {
    serialHandler.sendCommand("0");
    resetCycle();
    autoReturnToIdle = false;

    std::cout << "\nRetreat selesai. Sistem masuk IDLE.\n" << std::endl;
}

void ControlHandler::updateThresholds(int& last_thresh1, int& last_thresh2) {
    modbus_mapping_t* mb_mapping = modbusHandler.getMapping();
    if (mb_mapping == nullptr) return;
    
    int hmi_thresh1 = mb_mapping->tab_registers[ModbusAddr::THRESHOLD_1];
    int hmi_thresh2 = mb_mapping->tab_registers[ModbusAddr::THRESHOLD_2];

    if (hmi_thresh1 != last_thresh1 || hmi_thresh2 != last_thresh2) {
        std::stringstream ss;
        ss << "T" << hmi_thresh1 << "," << hmi_thresh2;
        serialHandler.sendCommand(ss.str());
        
        std::cout << "\nUpdate threshold: T1=" << hmi_thresh1 << ", T2=" << hmi_thresh2 << std::endl;

        last_thresh1 = hmi_thresh1;
        last_thresh2 = hmi_thresh2;
    }
}

void ControlHandler::handleTrajectorySelection() {
    modbus_mapping_t* mb_mapping = modbusHandler.getMapping();
    if (mb_mapping == nullptr) return;
    
    if (mb_mapping->tab_registers[ModbusAddr::TRAJEKTORI_1] == 1) {
        trajectoryManager.switchTrajectory(1);
        mb_mapping->tab_registers[ModbusAddr::TRAJEKTORI_1] = 0;
    }
    else if (mb_mapping->tab_registers[ModbusAddr::TRAJEKTORI_2] == 1) {
        trajectoryManager.switchTrajectory(2);
        mb_mapping->tab_registers[ModbusAddr::TRAJEKTORI_2] = 0;
    }
    else if (mb_mapping->tab_registers[ModbusAddr::TRAJEKTORI_3] == 1) {
        trajectoryManager.switchTrajectory(3);
        mb_mapping->tab_registers[ModbusAddr::TRAJEKTORI_3] = 0;
    }
}

void ControlHandler::processArduinoFeedback(std::string& arduinoFeedbackState,
                                           SystemState& currentState, int t_controller) {
    if (!serialHandler.hasData()) return;
    
    std::string resultString = serialHandler.readData();
    if (resultString.empty()) return;
    
    // === Data Logging ===
    if (logFile.is_open()) {
        static bool newLineStarted = true;
        for (char c : resultString) {
            if (newLineStarted) {
                auto now = std::chrono::system_clock::now();
                std::time_t now_time = std::chrono::system_clock::to_time_t(now);
                auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
                logFile << std::put_time(std::localtime(&now_time), "%Y-%m-%d %H:%M:%S.") 
                        << std::setfill('0') << std::setw(3) << ms.count() << ", ";
                newLineStarted = false;
            }
            logFile << c;
            if (c == '\n') newLineStarted = true;
        }
        logFile.flush();
    }
    
    // === CRITICAL: Check for pause/resume signals FIRST ===
    serialHandler.processIncomingData(resultString);
    
    // === Deteksi retreat eksplisit dari Arduino (bukan YANK_PAUSE) ===
    if (resultString.find("RETREAT") != std::string::npos &&
        resultString.find("ACK_RETREAT") == std::string::npos &&
        resultString.find("YANK_PAUSE") == std::string::npos &&
        currentState == SystemState::AUTO_REHAB) {
        std::cout << "\n!!! RETREAT COMMAND RECEIVED !!!" << std::endl;
        int retreatStartIndex = clampRetreatIndex(t_controller);
        startRetreatSequence(retreatStartIndex);
        currentState = SystemState::AUTO_RETREAT;
    }
    
    if (resultString.find("paused") != std::string::npos) {
        arduinoFeedbackState = "paused";
    } else if (resultString.find("running") != std::string::npos) {
        arduinoFeedbackState = "running";
    } else if (resultString.find("retreating") != std::string::npos) {
        arduinoFeedbackState = "retreating";
    }
    
    if (currentState == SystemState::AUTO_REHAB || 
        currentState == SystemState::AUTO_RETREAT) {
        float load_value = serialHandler.parseValue(resultString, "load:");
        if (load_value != -1.0f) {
            modbusHandler.writeFloat(ModbusAddr::REALTIME_LOAD_CELL, load_value);
        }
        
        float thresh1_value = serialHandler.parseValue(resultString, "thresh1:");
        if (thresh1_value != -1.0f) {
            modbusHandler.writeFloat(ModbusAddr::DYNAMIC_THRESH_1, thresh1_value);
        }
        
        float thresh2_value = serialHandler.parseValue(resultString, "thresh2:");
        if (thresh2_value != -1.0f) {
            modbusHandler.writeFloat(ModbusAddr::DYNAMIC_THRESH_2, thresh2_value);
        }
        
        float rate_value = serialHandler.parseValue(resultString, "rate:");
        if (rate_value != -1.0f) {
            modbusHandler.writeFloat(ModbusAddr::RATE_OF_CHANGE, rate_value);
        }
    } else {
        modbusHandler.writeFloat(ModbusAddr::REALTIME_LOAD_CELL, 0.0f);
        modbusHandler.writeFloat(ModbusAddr::DYNAMIC_THRESH_1, 0.0f);
        modbusHandler.writeFloat(ModbusAddr::DYNAMIC_THRESH_2, 0.0f);
        modbusHandler.writeFloat(ModbusAddr::RATE_OF_CHANGE, 0.0f);
    }
}

void ControlHandler::startRetreatSequence(int currentIndex) {
    retreatActive = true;
    retreatIndex = currentIndex - 1;
    lastForwardIndex = currentIndex;
    retreatTargetIndex = trajectoryManager.getGaitStartIndex();

    // Reset admittance pause — retreat harus jalan sampai titik awal gait
    serialHandler.resetPauseState();

    std::cout << "\n=== RETREAT SEQUENCE STARTED ===" << std::endl;
    std::cout << "From index: " << retreatIndex
              << " → target: " << retreatTargetIndex << std::endl;
}

void ControlHandler::processRetreatSequence(std::chrono::steady_clock::time_point& lastTraTime) {
    auto currentTime = std::chrono::steady_clock::now();

    if (retreatIndex >= retreatTargetIndex) {
        bool isFirstRetreatData = (retreatIndex == lastForwardIndex - 1);
        bool shouldSend = isFirstRetreatData ||
                         ((currentTime - lastTraTime) >= std::chrono::milliseconds(JEDA_KONTROLER_MS));

        if (shouldSend) {
            sendRetreatData(retreatIndex);
            retreatIndex--;
            lastTraTime = currentTime;
        }
    } else {
        serialHandler.sendCommand("RETREAT_COMPLETE");
        retreatActive = false;
        std::cout << "\n=== RETREAT SEQUENCE COMPLETED ===" << std::endl;
        std::cout << "Robot kembali ke titik awal gait (index "
                  << retreatTargetIndex << ")" << std::endl;
    }
}

void ControlHandler::processAutoRehab(SystemState& currentState, int& t_controller, int& t_grafik,
                                      bool& animasi_grafik,
                                      std::chrono::steady_clock::time_point& lastTraTime,
                                      std::chrono::steady_clock::time_point& delayStartTime) {
    auto currentTime = std::chrono::steady_clock::now();
    
    // === CRITICAL: Check if trajectory is paused by admittance control ===
    if (serialHandler.isTrajectoryPaused()) {
        // Trajectory is paused - DO NOT send new commands
        // DO NOT increment t_controller
        // This keeps the trajectory frozen at current position
        return;  // Exit early, everything stays frozen
    }
    
    if ((currentTime - lastTraTime) >= std::chrono::milliseconds(JEDA_KONTROLER_MS)) {
        std::string arduinoFeedbackState = "running"; // Should be passed as parameter
        
        if (arduinoFeedbackState == "running") {
            int jumlah_titik_gait = trajectoryManager.getGaitPointCount();
            if (t_controller < jumlah_titik_gait) {
                int actual_index = trajectoryManager.getGaitStartIndex() + t_controller;
                
                // Sinkronkan t_grafik dengan actual_index controller
                t_grafik = actual_index;
                
                // Update grafik animasi SETIAP KALI controller mengirim data untuk sinkronisasi yang tepat
                // Pastikan grafik selalu diupdate selama dalam range grafik
                int grafik_start = trajectoryManager.getGraphStartIndex();
                int grafik_end = trajectoryManager.getGraphEndIndex();
                if (t_grafik >= grafik_start && t_grafik < grafik_end) {
                    graphManager.updateGraphAnimation(t_grafik);
                    animasi_grafik = true;  // Pastikan animasi tetap aktif
                }
                
                // Kirim data controller (will be blocked automatically if paused by SerialHandler)
                sendControllerData(actual_index);
                
                t_controller++;
            } else {
                currentState = SystemState::POST_REHAB_DELAY;
                delayStartTime = currentTime;
            }
            
            lastTraTime = currentTime;
        }
    }
}

void ControlHandler::sendControllerData(int t) {
    // NOTE: sendCommand will automatically check pause state in SerialHandler
    // If paused, the command will be silently dropped
    std::stringstream ss;
    ss << "S" 
       << trajectoryManager.getActivePos1()[t] << "," 
       << trajectoryManager.getActivePos2()[t] << "," 
       << trajectoryManager.getActivePos3()[t] << ","
       << trajectoryManager.getActiveVelo1()[t] << "," 
       << trajectoryManager.getActiveVelo2()[t] << "," 
       << trajectoryManager.getActiveVelo3()[t] << ","
       << trajectoryManager.getActiveFc1()[t] << "," 
       << trajectoryManager.getActiveFc2()[t] << "," 
       << trajectoryManager.getActiveFc3()[t];
    serialHandler.sendCommand(ss.str());
}

void ControlHandler::sendRetreatData(int index) {
    // NOTE: sendCommand will automatically check pause state in SerialHandler
    // Retreat commands are also blocked during pause for consistency
    std::stringstream ss;
    ss << "R"
       << trajectoryManager.getActivePos1()[index] << "," 
       << trajectoryManager.getActivePos2()[index] << "," 
       << trajectoryManager.getActivePos3()[index] << ","
       << trajectoryManager.getActiveVelo1()[index] << "," 
       << trajectoryManager.getActiveVelo2()[index] << "," 
       << trajectoryManager.getActiveVelo3()[index] << ","
       << trajectoryManager.getActiveFc1()[index] << "," 
       << trajectoryManager.getActiveFc2()[index] << "," 
       << trajectoryManager.getActiveFc3()[index];
    serialHandler.sendCommand(ss.str());
}