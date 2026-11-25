#ifndef CLIHANDLER_H
#define CLIHANDLER_H

#include <string>
#include <queue>
#include <mutex>
#include <thread>
#include <atomic>

// Forward declarations
class SerialHandler;
class ModbusHandler;

class CLIHandler {
public:
    CLIHandler(SerialHandler& serial, ModbusHandler& modbus);
    ~CLIHandler();

    void start();   // Launch stdin reader thread
    void stop();    // Signal thread to stop

    // Called from main loop to process queued commands
    void processQueuedCommands();

    // Experiment mode tracking (for CSV tagging)
    bool isAdmittanceEnabled() const { return admittanceOn.load(); }
    std::string getCurrentModeTag() const;

private:
    SerialHandler& serialHandler;
    ModbusHandler& modbusHandler;

    std::thread readerThread;
    std::atomic<bool> running;
    std::atomic<bool> admittanceOn;   // Track mode for CSV tag

    // Threshold tracking (CTC-only mode, saat admittance OFF)
    float currentT1;   // Current pause threshold
    float currentT2;   // Current retreat threshold

    std::queue<std::string> commandQueue;
    std::mutex queueMutex;

    void readerLoop();                              // Thread function
    void executeCommand(const std::string& input);
    void parseAndSendParam(const std::string& input);
    void printHelp();
    void printStatus();
};

#endif // CLIHANDLER_H
