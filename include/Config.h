#ifndef CONFIG_H
#define CONFIG_H

//==================================================================
// KONSTANTA & KONFIGURASI SISTEM
//==================================================================

// *** JUMLAH TITIK PER TRAJEKTORI ***
const int JUMLAH_TITIK_T1 = 816;
const int JUMLAH_TITIK_T2 = 1370;
const int JUMLAH_TITIK_T3 = 1201;
const int MAX_TITIK = 1370;  // Maximum untuk alokasi array

// Timing Configuration
const int JEDA_KONTROLER_MS = 100;
const int JEDA_GRAFIK_MS = 100;
const int POST_REHAB_DELAY_SEC = 5;

// Telemetri: tampilkan di console mini PC (bukan Serial Monitor Arduino)
const bool ENABLE_TELEMETRY_CONSOLE = true;

// *** KONFIGURASI FILTER GRAFIK HMI (PER TRAJEKTORI) ***
// Trajektori 1
const int GRAFIK_START_INDEX_T1 = 1;
const int GRAFIK_END_INDEX_T1 = 816;
const int GAIT_START_INDEX_T1 = 1;
const int GAIT_END_INDEX_T1 = 816;

// Trajektori 2
const int GRAFIK_START_INDEX_T2 = 1;
const int GRAFIK_END_INDEX_T2 = 1370;
const int GAIT_START_INDEX_T2 = 1;
const int GAIT_END_INDEX_T2 = 1370;

// Trajektori 3
const int GRAFIK_START_INDEX_T3 = 1;
const int GRAFIK_END_INDEX_T3 = 1201;
const int GAIT_START_INDEX_T3 = 1;
const int GAIT_END_INDEX_T3 = 1201;

// Modbus Register Addresses
namespace ModbusAddr {
    // Control Registers
    const int MANUAL_MAJU = 99;
    const int MANUAL_STOP = 100;
    const int MANUAL_MUNDUR = 101;
    const int CALIBRATE = 102;
    const int START = 103;
    const int EMERGENCY = 104;
    const int RESET = 105;
    
    // Trajectory Selection
    const int TRAJEKTORI_1 = 106;
    const int TRAJEKTORI_2 = 107;
    const int TRAJEKTORI_3 = 108;
    
    // Threshold Registers
    const int THRESHOLD_1 = 130;
    const int THRESHOLD_2 = 131;
    
    // Cycle Counter
    const int JUMLAH_CYCLE = 132;
    
    // Dynamic Threshold (Float, 2 registers each)
    const int DYNAMIC_THRESH_1 = 134;
    const int DYNAMIC_THRESH_2 = 136;
    const int RATE_OF_CHANGE = 138;

    
    // Graph Registers
    const int COMMAND_REG = 120;
    const int NUM_OF_DATA_CH0 = 121;
    const int NUM_OF_DATA_CH1 = 122;
    const int REALTIME_LOAD_CELL = 126;
    const int X_DATA_CH0_START = 200;
    const int Y_DATA_CH0_START = 2000;
    const int X_DATA_CH1_START = 4000;
    const int Y_DATA_CH1_START = 6000;
}

// Serial Port Configuration
constexpr const char DEFAULT_SERIAL_PORT[] = "/dev/ttyACM0";
const int SERIAL_BAUD_RATE = 115200;

// Modbus Configuration
constexpr const char MODBUS_HOST[] = "0.0.0.0";
const int MODBUS_PORT = 5020;
const int MODBUS_SLAVE_ID = 1;
const int MODBUS_REGISTER_COUNT = 8000;

// Data Logging Configuration
constexpr const char LOG_DIR[] = "data/logs/";
constexpr const char LOG_FILENAME[] = "arduino_log.csv";

#endif // CONFIG_H

