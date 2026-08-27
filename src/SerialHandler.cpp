#include "SerialHandler.h"
#include "Config.h"
#include <iostream>
#include <iomanip>
#include <sys/ioctl.h>
#include <termios.h>
#include <array>

SerialHandler::SerialHandler(io_context& io) 
    : serial(io), io(io), isOpen(false), trajectoryPaused(false) {
}

SerialHandler::~SerialHandler() {
    if (isOpen) {
        close();
    }
}

bool SerialHandler::initialize(const std::string& port, int baudRate) {
    try {
        serial.open(port);
        serial.set_option(serial_port_base::baud_rate(baudRate));
        isOpen = true;
        trajectoryPaused = false;
        lineBuffer.clear();
        return true;
    } catch (const boost::system::system_error& e) {
        std::cerr << "Error membuka port serial: " << e.what() << std::endl;
        isOpen = false;
        return false;
    }
}

void SerialHandler::close() {
    if (isOpen) {
        serial.close();
        isOpen = false;
        trajectoryPaused = false;
        lineBuffer.clear();
    }
}

void SerialHandler::sendCommand(const std::string& cmd) {
    if (!isOpen) return;
    
    // Saat trajectory paused (admittance), hanya block forward command (S...)
    // Retreat command (R...) HARUS tetap lolos — robot harus bisa mundur dari bahaya
    if (trajectoryPaused && cmd[0] == 'S') {
        return;
    }
    
    std::string message = cmd + "\n";
    try {
        serial.write_some(boost::asio::buffer(message));
    } catch (const boost::system::system_error& e) {
        std::cerr << "Error sending command: " << e.what() << std::endl;
    }
}

bool SerialHandler::hasData() {
    if (!isOpen) return false;
    
    int bytes_available = 0;
    ::ioctl(serial.native_handle(), FIONREAD, &bytes_available);
    return bytes_available > 0;
}

std::string SerialHandler::readData() {
    if (!isOpen || !hasData()) return "";
    
    std::array<char, 256> buf;
    boost::system::error_code error;
    size_t len = serial.read_some(boost::asio::buffer(buf), error);
    
    if (len > 0 && !error) {
        return std::string(buf.data(), len);
    }
    
    return "";
}

float SerialHandler::parseValue(const std::string& data, const std::string& key) {
    size_t key_pos = data.find(key);
    if (key_pos == std::string::npos) return -1.0f;

    size_t value_start = key_pos + key.length();
    size_t value_end = data.find_first_of(",\n\r", value_start);
    if (value_end == std::string::npos) value_end = data.length();
    
    try {
        return std::stof(data.substr(value_start, value_end - value_start));
    } catch (const std::exception& e) {
        return -1.0f;
    }
}

void SerialHandler::processIncomingData(const std::string& chunk) {
    lineBuffer += chunk;

    size_t pos = 0;
    while ((pos = lineBuffer.find('\n')) != std::string::npos) {
        std::string line = lineBuffer.substr(0, pos);
        lineBuffer.erase(0, pos + 1);

        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }

        if (!line.empty()) {
            processLine(line);
        }
    }
}

void SerialHandler::processLine(const std::string& line) {
    processArduinoFeedback(line);

    if (ENABLE_TELEMETRY_CONSOLE && line.compare(0, 2, "s:") == 0) {
        printTelemetryToConsole(line);
        return;
    }

    printEventLine(line);
}

void SerialHandler::printTelemetryToConsole(const std::string& line) {
    std::string state = "run";
    size_t s_pos = line.find("s:");
    size_t m_pos = line.find(",m:");
    if (s_pos != std::string::npos && m_pos != std::string::npos) {
        state = line.substr(s_pos + 2, m_pos - (s_pos + 2));
    }

    std::string mode = "fwd";
    size_t load_pos = line.find(",load:");
    if (m_pos != std::string::npos && load_pos != std::string::npos) {
        mode = line.substr(m_pos + 3, load_pos - (m_pos + 3));
    }

    float load = parseValue(line, "load:");
    float yank = parseValue(line, "yank:");
    float p1   = parseValue(line, "p1:");
    float p2   = parseValue(line, "p2:");
    float p3   = parseValue(line, "p3:");
    float ep1  = parseValue(line, "ep1:");
    float ep2  = parseValue(line, "ep2:");
    float ep3  = parseValue(line, "ep3:");
    float Z    = parseValue(line, "Z:");
    float K    = parseValue(line, "K:");
    float B    = parseValue(line, "B:");
    float tpause = parseValue(line, "tpause:");

    std::cout << std::fixed << std::setprecision(2)
              << "[TELEM] " << state << "/" << mode
              << " | load=" << load << " yank=" << yank
              << " | pos=(" << p1 << "," << p2 << "," << p3 << ")"
              << " | err=(" << ep1 << "," << ep2 << "," << ep3 << ")";

    if (Z >= 0.0f) {
        std::cout << " | adm Z=" << Z << " K=" << K << " B=" << B;
    }
    if (tpause >= 0.0f) {
        std::cout << " | tpause=" << static_cast<int>(tpause);
    }
    std::cout << std::endl;
}

void SerialHandler::printEventLine(const std::string& line) {
    if (line == "READY") {
        std::cout << "[ARDUINO] Sistem siap (telemetri via mini PC)" << std::endl;
    }
    else if (line.find("YANK_PAUSE") != std::string::npos) {
        std::cout << "[SAFETY] " << line << std::endl;
    }
    else if (line.find("HOST_TIMEOUT") != std::string::npos) {
        std::cout << "[SAFETY] Host timeout — motor di-stop Arduino (bukan emergency HMI)" << std::endl;
    }
    else if (line.find("EMERGENCY_STOP") != std::string::npos ||
             line.find("System Reset OK") != std::string::npos ||
             line.find("ACK_") != std::string::npos ||
             line.find("Admittance") != std::string::npos ||
             line.find("Motor ") != std::string::npos ||
             line.find("ERR:") != std::string::npos) {
        std::cout << "[ARDUINO] " << line << std::endl;
    }
}

void SerialHandler::processArduinoFeedback(const std::string& data) {
    if (data.find("PAUSE_TRAJECTORY") != std::string::npos) {
        trajectoryPaused = true;
        std::cout << "[ADMITTANCE] Trajectory PAUSED (gaya eksternal terdeteksi)" << std::endl;
    }
    else if (data.find("RESUME_TRAJECTORY") != std::string::npos) {
        trajectoryPaused = false;
        std::cout << "[ADMITTANCE] Trajectory RESUMED (gaya dilepas)" << std::endl;
    }
    
    if (data.find("RETREAT") != std::string::npos &&
        data.find("ACK_RETREAT") == std::string::npos) {
        std::cout << "[SAFETY] Retreat requested by Arduino" << std::endl;
    }
}
