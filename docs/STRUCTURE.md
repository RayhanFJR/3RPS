# Struktur Modul 3RPS Modular

## Overview

Sistem telah dipecah menjadi modul-modul terpisah dengan arsitektur C++ berorientasi objek untuk memudahkan *maintenance*, keterbacaan, dan pengembangan fitur baru.

---

## 📦 Modul-Modul Sistem

### 1. Config (`include/Config.h`)
**Tanggung Jawab**: Menyimpan seluruh parameter konfigurasi global dan konstanta sistem.
- Jumlah titik per trajektori (`JUMLAH_TITIK_T1..T3`, `MAX_TITIK`).
- Timing loop kendali (`JEDA_KONTROLER_MS`, `JEDA_GRAFIK_MS`, `POST_REHAB_DELAY_SEC`).
- Pemetaan register Modbus (`ModbusAddr`).
- Konfigurasi port serial (`DEFAULT_SERIAL_PORT`, `SERIAL_BAUD_RATE`).
- Parameter server Modbus TCP (`MODBUS_HOST`, `MODBUS_PORT`, `MODBUS_SLAVE_ID`).

### 2. StateMachine (`include/StateMachine.h`)
**Tanggung Jawab**: Definisi state sistem.
- `SystemState::IDLE`: Menunggu perintah dari HMI/operator.
- `SystemState::AUTO_REHAB`: Mengirim titik lintasan gerak secara periodik.
- `SystemState::POST_REHAB_DELAY`: Jeda antar siklus sebelum kembali ke awal.
- `SystemState::AUTO_RETREAT`: Mengembalikan manipulator ke posisi aman (*home*).
- `SystemState::EMERGENCY_STOP`: Penghentian segera seluruh gerakan motor.
- `SystemState::RESETTING`: Pemulihan sistem setelah E-stop.

### 3. TrajectoryManager (`include/TrajectoryManager.h`, `src/TrajectoryManager.cpp`)
**Tanggung Jawab**: Manajemen file dataset lintasan gerak.
- Membaca file data posisi (`pos1..3.txt`), kecepatan (`velo1..3.txt`), gaya (`fc1..3.txt`), dan grafik (`grafik.txt`).
- Mengatur perpindahan aktif antar Trajektori 1, 2, dan 3.
- Menyediakan pointer array data ke modul kendali dan grafik.

### 4. SerialHandler (`include/SerialHandler.h`, `src/SerialHandler.cpp`)
**Tanggung Jawab**: Komunikasi serial USB dengan Arduino & data logging.
- Mengirim perintah teks ke Arduino (`S...`, `R...`, `ADMITTANCE_ON`, dll.).
- Membaca umpan balik telemetri Arduino (posisi aktual, error, load cell, yank, parameter admittance).
- Mencatat baris telemetri ke file `telemetry_YYYYMMDD_HHMMSS.csv` secara sinkron lengkap dengan tag eksperimen (`adm_mode`).
- Menangani *pause timeout fallback* untuk mencegah deadlock saat gangguan komunikasi serial.

### 5. CLIHandler (`include/CLIHandler.h`, `src/CLIHandler.cpp`)
**Tanggung Jawab**: Antarmuka terminal interaktif operator di mini PC.
- Berjalan di *background thread* membaca `stdin` tanpa memblokir siklus utama kendali.
- Mengantrekan perintah operator ke dalam antrean *thread-safe* (`std::mutex` + `std::queue`).
- Mem-parse perintah eksperimen: `admittance on/off/reset/status`, `set activation/k/b/gain`, `status`, `stop`, `reset`, `quit`.
- Melacak status aktif admittance untuk penandaan CSV (`ADM_ON` / `ADM_OFF`).

### 6. ModbusHandler (`include/ModbusHandler.h`, `src/ModbusHandler.cpp`)
**Tanggung Jawab**: Server Modbus TCP untuk HMI.
- Menginisialisasi soket server TCP pada port 5020.
- Melayani baca/tulis register kendali 16-bit dan data floating point (2 register).
- Menangani koneksi klien HMI secara *non-blocking* dengan timeout 10 ms.

### 7. GraphManager (`include/GraphManager.h`, `src/GraphManager.cpp`)
**Tanggung Jawab**: Manajemen visualisasi grafik pada HMI.
- Memuat kurva trajektori referensi ke Modbus Channel 0 (CH0).
- Memperbarui posisi aktual secara berkesinambungan ke Modbus Channel 1 (CH1).
- Mereset dan membersihkan buffer grafik saat pergantian siklus atau kalibrasi.

### 8. ControlHandler (`include/ControlHandler.h`, `src/ControlHandler.cpp`)
**Tanggung Jawab**: Logika otomasi rehabilitasi dan keselamatan.
- Menangani siklus *ramp-up* dari posisi awal menuju titik awal gait.
- Menghitung dan mengirim data titik target per jeda waktu (`JEDA_KONTROLER_MS`).
- Memproses urutan *Auto-Retreat* ke posisi nol (*go-to-zero*) saat terjadi lonjakan gaya (*yank spike*).
- Memantau kondisi *stall* mekanik saat kembali ke titik awal.

### 9. StateHandlers (`include/StateHandlers.h`, `src/StateHandlers.cpp`)
**Tanggung Jawab**: Implementasi transisi dan eksekusi state machine.
- Menangani perilaku pada state `IDLE`, `RESETTING`, `AUTO_RETREAT`, dan `POST_REHAB_DELAY`.

### 10. Main (`main.cpp`)
**Tanggung Jawab**: Titik masuk (*entry point*) dan koordinator siklus kendali utama.
- Menginisialisasi seluruh objek modul.
- Menjalankan loop utama (~100 Hz).
- Memproses antrean CLI, memeriksa Modbus TCP, membaca umpan balik Arduino, dan menggerakkan state machine.

---

## 🔄 Alur Eksekusi & Data

```
                  ┌──────────────────────┐
                  │ Operator Terminal    │
                  └──────────┬───────────┘
                             │ stdin (Thread CLI)
                             ▼
                  ┌──────────────────────┐
                  │ CLIHandler           │
                  └──────────┬───────────┘
                             │ Queue (Mutex Protected)
                             ▼
┌──────────────┐     ┌──────────────────────┐     ┌──────────────────────┐
│  HMI Display │◄───►│  Main Loop (main.cpp)│◄───►│  Arduino Leonardo    │
└──────────────┘     └──────────┬───────────┘     └──────────────────────┘
  Modbus TCP                    │                   Serial USB (/dev/ttyACM0)
  (Port 5020)                   ▼
                     ┌──────────────────────┐
                     │ CSV Telemetry Logger │
                     │ (ADM_ON / ADM_OFF)   │
                     └──────────────────────┘
```

---

## 🗂️ Ketergantungan Antar-Modul

```
main.cpp
 ├── Config
 ├── StateMachine
 ├── TrajectoryManager
 ├── SerialHandler ◄────► CLIHandler
 ├── ModbusHandler
 ├── GraphManager (tergantung ModbusHandler & TrajectoryManager)
 ├── ControlHandler (tergantung ModbusHandler, SerialHandler, TrajectoryManager, GraphManager)
 └── StateHandlers (tergantung ControlHandler, ModbusHandler, SerialHandler)
```
