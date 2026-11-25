#include "CLIHandler.h"
#include "SerialHandler.h"
#include "ModbusHandler.h"
#include "Config.h"
#include <iostream>
#include <sstream>
#include <algorithm>

CLIHandler::CLIHandler(SerialHandler& serial, ModbusHandler& modbus)
    : serialHandler(serial), modbusHandler(modbus),
      running(false), admittanceOn(true) {
    // Default: admittance ON (sesuai default Arduino: admittanceEnabled = true)
}

CLIHandler::~CLIHandler() {
    stop();
}

void CLIHandler::start() {
    running.store(true);
    readerThread = std::thread(&CLIHandler::readerLoop, this);
}

void CLIHandler::stop() {
    running.store(false);
    // Note: getline() blocks, thread will exit saat stdin di-close atau program exit
    if (readerThread.joinable()) {
        readerThread.detach();  // Detach karena getline() blocking
    }
}

std::string CLIHandler::getCurrentModeTag() const {
    return admittanceOn.load() ? "ADM_ON" : "ADM_OFF";
}

void CLIHandler::readerLoop() {
    // Beri jeda sedikit agar banner sistem muncul duluan
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    std::cout << "\n=========================================" << std::endl;
    std::cout << "  CLI EXPERIMENT CONTROLLER" << std::endl;
    std::cout << "  Ketik 'help' untuk daftar command" << std::endl;
    std::cout << "  Ketik 'telem off' untuk mode CLI bersih" << std::endl;
    std::cout << "=========================================" << std::endl;
    std::cout << "> " << std::flush;

    std::string line;
    while (running.load() && std::getline(std::cin, line)) {
        // Trim whitespace
        size_t start = line.find_first_not_of(" \t\r\n");
        size_t end   = line.find_last_not_of(" \t\r\n");
        if (start == std::string::npos) {
            std::cout << "> " << std::flush;
            continue;
        }
        line = line.substr(start, end - start + 1);

        // Lowercase untuk parsing
        std::string lower = line;
        std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);

        if (lower == "quit" || lower == "exit") {
            std::cout << "[CLI] Shutting down..." << std::endl;
            {
                std::lock_guard<std::mutex> lock(queueMutex);
                commandQueue.push("__QUIT__");
            }
            break;
        }

        {
            std::lock_guard<std::mutex> lock(queueMutex);
            commandQueue.push(lower);
        }

        std::cout << "> " << std::flush;
    }
}

void CLIHandler::processQueuedCommands() {
    std::lock_guard<std::mutex> lock(queueMutex);
    while (!commandQueue.empty()) {
        std::string cmd = commandQueue.front();
        commandQueue.pop();

        if (cmd == "__QUIT__") {
            serialHandler.sendCommand("0");
            std::cout << "[CLI] Motor stopped. Program exiting." << std::endl;
            exit(0);
        }

        executeCommand(cmd);
    }
}

void CLIHandler::executeCommand(const std::string& input) {
    // ========== Telemetry Toggle ==========
    if (input == "telem off" || input == "telemetry off") {
        serialHandler.setTelemetryConsole(false);
        std::cout << "[CLI] Telemetri console OFF (CSV tetap logging)" << std::endl;
    }
    else if (input == "telem on" || input == "telemetry on") {
        serialHandler.setTelemetryConsole(true);
        std::cout << "[CLI] Telemetri console ON" << std::endl;
    }

    // ========== Rehab Control (via Modbus registers) ==========
    else if (input == "start") {
        modbusHandler.writeRegister(ModbusAddr::START, 1);
        std::cout << "[CLI] START rehab dikirim (via Modbus register)" << std::endl;
    }
    else if (input == "calibrate") {
        modbusHandler.writeRegister(ModbusAddr::CALIBRATE, 1);
        std::cout << "[CLI] CALIBRATE dikirim" << std::endl;
    }
    else if (input == "trajectory 1" || input == "traj 1") {
        modbusHandler.writeRegister(ModbusAddr::TRAJEKTORI_1, 1);
        std::cout << "[CLI] Trajectory 1 dipilih" << std::endl;
    }
    else if (input == "trajectory 2" || input == "traj 2") {
        modbusHandler.writeRegister(ModbusAddr::TRAJEKTORI_2, 1);
        std::cout << "[CLI] Trajectory 2 dipilih" << std::endl;
    }
    else if (input == "trajectory 3" || input == "traj 3") {
        modbusHandler.writeRegister(ModbusAddr::TRAJEKTORI_3, 1);
        std::cout << "[CLI] Trajectory 3 dipilih" << std::endl;
    }
    else if (input.compare(0, 6, "cycle ") == 0) {
        try {
            int n = std::stoi(input.substr(6));
            if (n < 1) n = 1;
            modbusHandler.writeRegister(ModbusAddr::JUMLAH_CYCLE, n);
            std::cout << "[CLI] Target cycle = " << n << std::endl;
        } catch (...) {
            std::cout << "[CLI] Usage: cycle <jumlah>" << std::endl;
        }
    }
    else if (input == "forward" || input == "fwd") {
        modbusHandler.writeRegister(ModbusAddr::MANUAL_MAJU, 1);
        std::cout << "[CLI] Manual FORWARD" << std::endl;
    }
    else if (input == "reverse" || input == "rev") {
        modbusHandler.writeRegister(ModbusAddr::MANUAL_MUNDUR, 1);
        std::cout << "[CLI] Manual REVERSE" << std::endl;
    }

    // ========== Admittance Control ==========
    else if (input == "admittance on") {
        serialHandler.sendCommand("ADMITTANCE_ON");
        admittanceOn.store(true);
        std::cout << "[CLI] Admittance ON — mode: WITH admittance" << std::endl;
    }
    else if (input == "admittance off") {
        serialHandler.sendCommand("ADMITTANCE_OFF");
        admittanceOn.store(false);
        std::cout << "[CLI] Admittance OFF — mode: WITHOUT admittance" << std::endl;
    }
    else if (input == "admittance reset") {
        serialHandler.sendCommand("ADMITTANCE_RESET");
        admittanceOn.store(true);
        std::cout << "[CLI] Admittance RESET to defaults (K=15556, B=7272, a=0.3)" << std::endl;
    }
    else if (input == "admittance status") {
        serialHandler.sendCommand("ADMITTANCE_STATUS");
    }

    // ========== Parameter Tuning ==========
    else if (input.compare(0, 4, "set ") == 0) {
        parseAndSendParam(input);
    }

    // ========== System Commands ==========
    else if (input == "status" || input == "mode") {
        printStatus();
    }
    else if (input == "help") {
        printHelp();
    }
    else if (input == "reset") {
        serialHandler.sendCommand("X");
        std::cout << "[CLI] System RESET sent to Arduino" << std::endl;
    }
    else if (input == "emergency" || input == "estop") {
        modbusHandler.writeRegister(ModbusAddr::EMERGENCY, 1);
        std::cout << "[CLI] EMERGENCY STOP" << std::endl;
    }
    else if (input == "stop") {
        modbusHandler.writeRegister(ModbusAddr::MANUAL_STOP, 1);
        std::cout << "[CLI] Motor STOP" << std::endl;
    }

    // ========== Unknown ==========
    else {
        std::cout << "[CLI] Unknown command: '" << input << "' (ketik 'help')" << std::endl;
    }
}

void CLIHandler::parseAndSendParam(const std::string& input) {
    std::istringstream iss(input);
    std::string cmd, param, valStr;
    iss >> cmd >> param >> valStr;

    if (param.empty() || valStr.empty()) {
        std::cout << "[CLI] Usage: set <activation|K|B|gain> <value>" << std::endl;
        return;
    }

    float val;
    try {
        val = std::stof(valStr);
    } catch (...) {
        std::cout << "[CLI] Error: '" << valStr << "' is not a valid number" << std::endl;
        return;
    }

    std::string arduinoCmd;
    if (param == "activation" || param == "a") {
        if (val < 0.01f || val > 1.0f) {
            std::cout << "[CLI] Warning: activation harus 0.01-1.0, got " << val << std::endl;
        }
        arduinoCmd = "ADM,A," + valStr;
    }
    else if (param == "k") {
        arduinoCmd = "ADM,K," + valStr;
    }
    else if (param == "b") {
        arduinoCmd = "ADM,B," + valStr;
    }
    else if (param == "gain" || param == "g") {
        arduinoCmd = "ADM,G," + valStr;
    }
    else {
        std::cout << "[CLI] Unknown param: '" << param << "'. Use: activation, K, B, gain" << std::endl;
        return;
    }

    serialHandler.sendCommand(arduinoCmd);
    std::cout << "[CLI] Sent to Arduino: " << arduinoCmd << std::endl;

    (void)val;
}

void CLIHandler::printStatus() {
    std::cout << "\n--- Experiment Status ---" << std::endl;
    std::cout << "  Admittance mode  : " << (admittanceOn.load() ? "ON (WITH admittance)" : "OFF (WITHOUT admittance)") << std::endl;
    std::cout << "  CSV tag          : " << getCurrentModeTag() << std::endl;
    std::cout << "  Telemetri console: " << (serialHandler.isTelemetryConsoleEnabled() ? "ON" : "OFF") << std::endl;
    std::cout << "  (Ketik 'admittance status' untuk detail K/B/Z dari Arduino)" << std::endl;
    std::cout << "-------------------------\n" << std::endl;
}

void CLIHandler::printHelp() {
    std::cout << "\n=== CLI Experiment Controller ===" << std::endl;
    std::cout << "Display:" << std::endl;
    std::cout << "  telem off/on        Toggle telemetri console (CSV tetap jalan)" << std::endl;
    std::cout << std::endl;
    std::cout << "Rehab Control:" << std::endl;
    std::cout << "  start               Mulai siklus rehabilitasi" << std::endl;
    std::cout << "  calibrate           Kalibrasi & reset posisi" << std::endl;
    std::cout << "  trajectory 1/2/3    Pilih trajektori (alias: traj 1/2/3)" << std::endl;
    std::cout << "  cycle <n>           Set jumlah cycle target" << std::endl;
    std::cout << "  forward / fwd       Manual maju" << std::endl;
    std::cout << "  reverse / rev       Manual mundur" << std::endl;
    std::cout << std::endl;
    std::cout << "Experiment:" << std::endl;
    std::cout << "  admittance on       Aktifkan admittance control" << std::endl;
    std::cout << "  admittance off      Matikan admittance control" << std::endl;
    std::cout << "  admittance reset    Reset K, B, activation ke default" << std::endl;
    std::cout << "  admittance status   Print status admittance (dari Arduino)" << std::endl;
    std::cout << std::endl;
    std::cout << "Parameter Tuning:" << std::endl;
    std::cout << "  set activation <v>  Set aktivasi otot (0.01-1.0)" << std::endl;
    std::cout << "  set K <v>           Override stiffness K (N/m)" << std::endl;
    std::cout << "  set B <v>           Override damping B (N.s/m)" << std::endl;
    std::cout << "  set gain <v>        Set admittance gain" << std::endl;
    std::cout << std::endl;
    std::cout << "System:" << std::endl;
    std::cout << "  status / mode       Print current experiment mode" << std::endl;
    std::cout << "  stop                Stop motors" << std::endl;
    std::cout << "  reset               System reset (send X to Arduino)" << std::endl;
    std::cout << "  emergency / estop   Emergency stop" << std::endl;
    std::cout << "  help                Show this help" << std::endl;
    std::cout << "  quit / exit         Graceful shutdown" << std::endl;
    std::cout << "=================================\n" << std::endl;
}
