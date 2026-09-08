#include "ControlHandler.h"
#include "Config.h"
#include "StateMachine.h"
#include <iostream>
#include <sstream>
#include <chrono>
#include <cmath>

ControlHandler::ControlHandler(ModbusHandler& modbus, SerialHandler& serial,
                               TrajectoryManager& trajectory, GraphManager& graph)
    : modbusHandler(modbus), serialHandler(serial), 
      trajectoryManager(trajectory), graphManager(graph),
      target_cycle(1), current_cycle(0),
      retreatIndex(0), retreatTargetIndex(0), retreatActive(false), lastForwardIndex(0),
      autoReturnToIdle(false),
      retreatStartTime(std::chrono::steady_clock::now()),
      retreatStallCount(0), stallP1(0.0f), stallP2(0.0f), stallP3(0.0f),
      rampUpPhase(false), rampUpIndex(0) {
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
    
    // Setup ramp-up phase: traversal dari graphStart → gaitStart sebelum gait utama
    // Ini mencegah motor "loncat" dari posisi home ke titik gait pertama
    if (gait_start > grafik_start) {
        rampUpPhase = true;
        rampUpIndex = grafik_start;
        std::cout << "Fase ramp-up aktif: index " << grafik_start
                  << " -> " << (gait_start - 1) << std::endl;
    } else {
        rampUpPhase = false;
        rampUpIndex = 0;
    }

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
    
    // Setup ramp-up phase untuk cycle baru (robot balik ke home via retreat sebelumnya)
    if (gait_start > grafik_start) {
        rampUpPhase = true;
        rampUpIndex = grafik_start;
        std::cout << "Fase ramp-up aktif untuk cycle baru: index " << grafik_start
                  << " -> " << (gait_start - 1) << std::endl;
    } else {
        rampUpPhase = false;
        rampUpIndex = 0;
    }
}

int ControlHandler::clampRetreatIndex(int controllerSteps) const {
    int gaitStart = trajectoryManager.getGaitStartIndex();
    int gaitEnd = trajectoryManager.getGaitEndIndex();
    int actualIndex = gaitStart + controllerSteps - 1;
    
    if (actualIndex < gaitStart) actualIndex = gaitStart;
    if (actualIndex >= gaitEnd) actualIndex = gaitEnd - 1;
    return actualIndex;
}

void ControlHandler::startAutoReturnToZero(int /*controllerSteps*/) {
    if (retreatActive) return;
    
    // Go-to-zero: kirim satu command R ke posisi home (0,0,0)
    // Arduino CTC yang bawa semua motor ke 0 langsung
    retreatActive     = true;
    autoReturnToIdle  = true;
    lastForwardIndex  = 0;
    retreatStartTime  = std::chrono::steady_clock::now();
    retreatStallCount = 0;
    stallP1 = serialHandler.getLastPos1();
    stallP2 = serialHandler.getLastPos2();
    stallP3 = serialHandler.getLastPos3();
    
    serialHandler.resetPauseState();
    serialHandler.sendCommand("R0,0,0,0,0,0,0,0,0");
    
    std::cout << "\n=== GO TO ZERO (AUTO RETURN) ===" << std::endl;
    std::cout << "Semua motor bergerak ke posisi home (0,0,0)..." << std::endl;
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
    
    // === Deteksi retreat dari Arduino (YANK_PAUSE atau eksplisit RETREAT) ===
    // HARUS dicek SEBELUM position check agar tidak salah langsung complete
    // saat posisi kebetulan dekat 0 di awal rehab bersamaan dengan YANK event.
    bool retreatTriggered = false;

    if (resultString.find("YANK_PAUSE") != std::string::npos &&
        currentState == SystemState::AUTO_REHAB &&
        !serialHandler.isTrajectoryPaused()) {
        // Guard: hanya trigger retreat jika admittance TIDAK sedang pause.
        // Saat user menahan beban (force > FORCE_PAUSE_THRESHOLD), admittance
        // pause sudah aktif — ini respons normal, bukan kondisi darurat.
        // YANK pada momen pertama kali user tekan akan selalu tinggi, jadi
        // kalau admittance pause aktif, biarkan admittance yang handle.
        std::cout << "\n!!! YANK SPIKE — TRIGGERING GO-TO-ZERO RETREAT !!!" << std::endl;
        retreatTriggered = true;
    }

    if (resultString.find("RETREAT") != std::string::npos &&
        resultString.find("ACK_RETREAT") == std::string::npos &&
        resultString.find("YANK_PAUSE") == std::string::npos &&
        currentState == SystemState::AUTO_REHAB) {
        std::cout << "\n!!! RETREAT COMMAND RECEIVED !!!" << std::endl;
        retreatTriggered = true;
    }

    if (retreatTriggered) {
        retreatActive     = true;
        autoReturnToIdle  = false;
        lastForwardIndex  = t_controller;
        retreatStartTime  = std::chrono::steady_clock::now();
        retreatStallCount = 0;
        stallP1 = serialHandler.getLastPos1();
        stallP2 = serialHandler.getLastPos2();
        stallP3 = serialHandler.getLastPos3();
        serialHandler.resetPauseState();
        serialHandler.sendCommand("R0,0,0,0,0,0,0,0,0");
        currentState = SystemState::AUTO_RETREAT;
        std::cout << "Semua motor bergerak ke posisi home (0,0,0)..." << std::endl;
        return;
    }

    // === Stall Detection: posisi tidak berubah selama STALL_PACKETS berturut-turut ===
    // Lebih reliable dari position < TOL karena tidak butuh encoder kalibrasi ke 0.
    // Saat robot mentok ke batas fisik, motor stall, posisi berhenti berubah.
    // Membaca dari serialHandler.getLastPos() yang di-parse dari complete telemetry line
    // (bukan raw chunk) — aman dari noise ep1/rp1 substring.
    if (currentState == SystemState::AUTO_RETREAT && retreatActive) {
        auto retreatElapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - retreatStartTime).count();

        const long  RETREAT_MIN_MS  = 2000; // ms minimum sebelum mulai cek (beri waktu bergerak)
        const float STALL_DELTA_TOL = 0.5f; // mm total delta per siklus dianggap "diam"
        const int   STALL_PACKETS   = 15;   // 15 paket x 100ms = 1.5 detik diam = selesai

        float p1 = serialHandler.getLastPos1();
        float p2 = serialHandler.getLastPos2();
        float p3 = serialHandler.getLastPos3();

        if (retreatElapsed >= RETREAT_MIN_MS && p1 != -1.0f) {
            float delta = std::abs(p1 - stallP1)
                        + std::abs(p2 - stallP2)
                        + std::abs(p3 - stallP3);

            if (delta < STALL_DELTA_TOL) {
                retreatStallCount++;
            } else {
                retreatStallCount = 0;
                stallP1 = p1; stallP2 = p2; stallP3 = p3;  // Update referensi
            }

            std::cout << "[STALL-DBG] elapsed=" << retreatElapsed
                      << "ms | p=(" << p1 << "," << p2 << "," << p3 << ")"
                      << " delta=" << delta
                      << " stall=" << retreatStallCount << "/" << STALL_PACKETS
                      << std::endl;

            if (retreatStallCount >= STALL_PACKETS) {
                serialHandler.sendCommand("RETREAT_COMPLETE");
                serialHandler.sendCommand("0");
                retreatActive     = false;
                retreatStallCount = 0;
                std::cout << "\n=== ROBOT STALLED AT HOME — RETREAT COMPLETE ==="  << std::endl;
                std::cout << "Posisi akhir: p1=" << p1 << " p2=" << p2
                          << " p3=" << p3 << " mm" << std::endl;
            }
        }
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

void ControlHandler::startRetreatSequence(int /*currentIndex*/) {
    retreatActive    = true;
    autoReturnToIdle = false;

    // Reset admittance pause — robot harus bisa mundur bebas dari gaya eksternal
    serialHandler.resetPauseState();

    // Go-to-zero: satu command langsung ke posisi home (0,0,0)
    // Arduino CTC loop yang bawa semua motor ke 0
    serialHandler.sendCommand("R0,0,0,0,0,0,0,0,0");

    std::cout << "\n=== GO TO ZERO RETREAT ===" << std::endl;
    std::cout << "Semua motor menuju posisi home (0,0,0)..." << std::endl;
}

void ControlHandler::processRetreatSequence(std::chrono::steady_clock::time_point& /*lastTraTime*/) {
    // Go-to-zero mode: Arduino sudah diberi target R0,0,0,0,0,0,0,0,0 saat retreat dimulai
    // Tidak perlu polling — tunggu WAYPOINT_REACHED dari Arduino
    // (ditangani di processArduinoFeedback)
}

void ControlHandler::processAutoRehab(SystemState& currentState, int& t_controller, int& t_grafik,
                                      bool& animasi_grafik,
                                      std::chrono::steady_clock::time_point& lastTraTime,
                                      std::chrono::steady_clock::time_point& delayStartTime) {
    // === CRITICAL: Check if trajectory is paused by admittance control ===
    if (serialHandler.isTrajectoryPaused()) {
        return;  // Trajectory frozen — jangan kirim apapun
    }

    // === TIME GATE: kirim titik berikutnya hanya setelah JEDA_KONTROLER_MS ===
    // Ini menggantikan peran waitingForWaypoint sebagai throttle.
    // Tanpa ini, semua titik trajektori akan habis terkirim dalam < 1 detik.
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - lastTraTime).count();
    if (elapsed < JEDA_KONTROLER_MS) {
        return;  // Belum waktunya kirim titik berikutnya
    }

    int grafik_start = trajectoryManager.getGraphStartIndex();
    int grafik_end   = trajectoryManager.getGraphEndIndex();
    
    // === FASE RAMP-UP: traversal index grafik_start → gaitStart - 1 ===
    // Ini memastikan robot tidak "loncat" dari posisi home ke gait pertama
    if (rampUpPhase) {
        int gait_start = trajectoryManager.getGaitStartIndex();
        
        if (rampUpIndex < gait_start) {
            // Update grafik animasi untuk fase ramp-up
            if (rampUpIndex >= grafik_start && rampUpIndex < grafik_end) {
                graphManager.updateGraphAnimation(rampUpIndex);
                animasi_grafik = true;
                t_grafik = rampUpIndex;
            }
            
            // Kirim waypoint ramp-up langsung (time-based, tanpa tunggu ACK)
            sendControllerData(rampUpIndex);
            rampUpIndex++;
            lastTraTime = std::chrono::steady_clock::now();
        } else {
            // Ramp-up selesai — beralih ke gait utama
            rampUpPhase = false;
            std::cout << "\n=== RAMP-UP SELESAI - MASUK GAIT UTAMA ==="  << std::endl;
        }
        return;
    }
    
    // === FASE GAIT UTAMA ===
    int jumlah_titik_gait = trajectoryManager.getGaitPointCount();
    if (t_controller < jumlah_titik_gait) {
        int actual_index = trajectoryManager.getGaitStartIndex() + t_controller;
        
        // Sinkronkan t_grafik
        t_grafik = actual_index;
        
        // Update grafik animasi
        if (t_grafik >= grafik_start && t_grafik < grafik_end) {
            graphManager.updateGraphAnimation(t_grafik);
            animasi_grafik = true;
        }
        
        // Kirim waypoint langsung (time-based, tanpa tunggu ACK)
        sendControllerData(actual_index);
        t_controller++;
        lastTraTime = std::chrono::steady_clock::now();
    } else {
        // Semua titik gait sudah dikirim → masuk delay sebelum cycle berikutnya
        currentState = SystemState::POST_REHAB_DELAY;
        delayStartTime = std::chrono::steady_clock::now();
    }
}

// notifyWaypointReached: dihapus (tidak lagi digunakan, sistem ACK-based sudah dinonaktifkan)

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