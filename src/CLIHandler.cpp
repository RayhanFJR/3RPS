#include "CLIHandler.h"
#include "SerialHandler.h"
#include <iostream>
#include <sstream>
#include <algorithm>

CLIHandler::CLIHandler(SerialHandler& serial)
    : serialHandler(serial), running(false), admittanceOn(true) {
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
            // Kirim stop command ke Arduino
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
            // Graceful shutdown: stop motor dan exit
            serialHandler.sendCommand("0");
            std::cout << "[CLI] Motor stopped. Program exiting." << std::endl;
            exit(0);
        }

        executeCommand(cmd);
    }
}

void CLIHandler::executeCommand(const std::string& input) {
    // ========== Admittance Control ==========
    if (input == "admittance on") {
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
        admittanceOn.store(true);  // Reset ke default (ON)
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

    // ========== Direct Arduino Commands (advanced) ==========
    else if (input == "reset") {
        serialHandler.sendCommand("X");
        std::cout << "[CLI] System RESET sent to Arduino" << std::endl;
    }
    else if (input == "emergency" || input == "estop") {
        serialHandler.sendCommand("E");
        std::cout << "[CLI] EMERGENCY STOP sent to Arduino" << std::endl;
    }
    else if (input == "stop") {
        serialHandler.sendCommand("0");
        std::cout << "[CLI] Motor STOP sent to Arduino" << std::endl;
    }

    // ========== Unknown ==========
    else {
        std::cout << "[CLI] Unknown command: '" << input << "' (ketik 'help')" << std::endl;
    }
}

void CLIHandler::parseAndSendParam(const std::string& input) {
    // Format: "set <param> <value>"
    std::istringstream iss(input);
    std::string cmd, param, valStr;
    iss >> cmd >> param >> valStr;

    if (param.empty() || valStr.empty()) {
        std::cout << "[CLI] Usage: set <activation|K|B|gain> <value>" << std::endl;
        return;
    }

    // Validasi value
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

    // Suppress unused warning
    (void)val;
}

void CLIHandler::printStatus() {
    std::cout << "\n--- Experiment Status ---" << std::endl;
    std::cout << "  Admittance mode : " << (admittanceOn.load() ? "ON (WITH admittance)" : "OFF (WITHOUT admittance)") << std::endl;
    std::cout << "  CSV tag         : " << getCurrentModeTag() << std::endl;
    std::cout << "  (Ketik 'admittance status' untuk detail K/B/Z dari Arduino)" << std::endl;
    std::cout << "-------------------------\n" << std::endl;
}

void CLIHandler::printHelp() {
    std::cout << "\n=== CLI Experiment Controller ===" << std::endl;
    std::cout << "Experiment Control:" << std::endl;
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
    std::cout << "  reset               System reset (send X to Arduino)" << std::endl;
    std::cout << "  stop                Stop motors (send 0 to Arduino)" << std::endl;
    std::cout << "  emergency / estop   Emergency stop" << std::endl;
    std::cout << "  help                Show this help" << std::endl;
    std::cout << "  quit / exit         Graceful shutdown" << std::endl;
    std::cout << "=================================\n" << std::endl;
}
