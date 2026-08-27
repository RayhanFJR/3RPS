#ifndef SERIALHANDLER_H
#define SERIALHANDLER_H

#include <boost/asio.hpp>
#include <string>

using namespace boost::asio;

class SerialHandler {
private:
    serial_port serial;
    io_context& io;
    bool isOpen;
    bool trajectoryPaused;
    std::string lineBuffer;

    void processLine(const std::string& line);
    void printTelemetryToConsole(const std::string& line);
    void printEventLine(const std::string& line);

public:
    SerialHandler(io_context& io);
    ~SerialHandler();
    
    bool initialize(const std::string& port, int baudRate);
    void close();
    void sendCommand(const std::string& cmd);
    bool hasData();
    std::string readData();
    float parseValue(const std::string& data, const std::string& key);
    
    void processIncomingData(const std::string& chunk);
    void processArduinoFeedback(const std::string& data);
    bool isTrajectoryPaused() const { return trajectoryPaused; }
    void resetPauseState() { trajectoryPaused = false; }
};

#endif // SERIALHANDLER_H
