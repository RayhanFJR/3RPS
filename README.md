# 3-RPS Parallel Robot - Rehabilitation Control System (Modular)

Sistem kontrol robot paralel 3-RPS untuk rehabilitasi pergelangan kaki (*ankle rehabilitation*), dilengkapi dengan kontrol posisi/kecepatan (CTC + PD), **Admittance Control** (model Hill-Zajac), antarmuka HMI via **Modbus TCP**, dan **CLI Experiment Controller** di mini PC untuk pengujian perbandingan (*with vs. without admittance*).

---

## 📁 Struktur Proyek

```
3RPS/
├── include/                      # Header files
│   ├── Config.h                  # Parameter sistem, timing, register Modbus
│   ├── StateMachine.h            # Enum state sistem
│   ├── TrajectoryManager.h       # Pengelola file trajectory
│   ├── SerialHandler.h           # Komunikasi serial Arduino & CSV logger
│   ├── ModbusHandler.h           # Komunikasi Modbus TCP ke HMI
│   ├── GraphManager.h            # Buffer data grafik real-time HMI
│   ├── ControlHandler.h          # Alur kontrol rehabilitasi & auto-retreat
│   ├── StateHandlers.h           # Transisi state machine
│   └── CLIHandler.h              # Command-Line Interface (stdin thread)
├── src/                          # Source files C++
│   ├── TrajectoryManager.cpp
│   ├── SerialHandler.cpp
│   ├── ModbusHandler.cpp
│   ├── GraphManager.cpp
│   ├── ControlHandler.cpp
│   ├── StateHandlers.cpp
│   └── CLIHandler.cpp
├── docs/                         # Dokumentasi teknis
│   ├── cli.md                    # Panduan lengkap CLI & analisis paper
│   ├── admittance.md             # Penurunan matematis K & B Hill-Zajac
│   ├── instalasi.md              # Detail dependensi & setup
│   ├── penggunaan.md             # Panduan operasional HMI & register
│   └── STRUCTURE.md              # Arsitektur modul
├── data/                         # Dataset lintasan gerak (pos, velo, fc, grafik)
│   ├── data_1/
│   ├── data_2/
│   └── data_3/
├── control.ino                   # Firmware Arduino Leonardo (CTC, Admittance, Motor Driver)
├── main.cpp                      # Program utama mini PC (Main loop)
├── Makefile                      # Build script Linux
└── README.md
```

---

## 🛠️ Persyaratan Sistem

### 1. Hardware
- **Mini PC / Komputer Kontrol**: OS Linux (Ubuntu/Debian) atau WSL2.
- **Microcontroller**: Arduino Leonardo (atau mikrokontroler berbasis ATmega32U4) terhubung via USB.
- **HMI / Display**: Terhubung satu jaringan lokal (LAN/Ethernet) via Modbus TCP.
- **Sensor**: Load Cell (HX711), Current Sensor (ACS712), Rotary Encoder.
- **Aktuator**: 3 unit DC Motor + Motor Driver (PWM).

### 2. Software Dependencies (Mini PC)
- **Kompiler**: GCC/G++ (dukungan C++11) & Make
- **Library**: `libmodbus`, `libboost-system`, `libboost-thread` (Boost C++), `pthread`

Instalasi dependensi di Ubuntu/Debian:
```bash
sudo apt update
sudo apt install build-essential g++ make libmodbus-dev libboost-all-dev
```

---

## 🚀 Prosedur Menjalankan Program

### Langkah 1: Flash Firmware Arduino
1. Buka file `control.ino` di Arduino IDE.
2. Pilih Board: **Arduino Leonardo** (atau board yang digunakan) dan pilih port serial yang sesuai.
3. Upload kode ke Arduino.
4. *Catatan:* Tidak perlu mengubah firmware Arduino saat beralih mode admittance, semua perintah dikendalikan langsung oleh mini PC via serial.

### Langkah 2: Hubungkan Hardware & Periksa Port Serial
1. Hubungkan Arduino ke mini PC via kabel USB.
2. Periksa nama port serial di terminal mini PC:
   ```bash
   ls /dev/ttyACM* /dev/ttyUSB*
   ```
   *(Default di `include/Config.h`: `/dev/ttyACM0`)*.
3. Berikan hak akses port serial untuk user:
   ```bash
   sudo usermod -a -G dialout $USER
   sudo chmod 666 /dev/ttyACM0
   ```

### Langkah 3: Kompilasi Program Mini PC
Masuk ke direktori root proyek dan jalankan `make`:
```bash
make clean
make
```
Setelah proses build selesai, file *executable* bernama **`mainhmi`** akan dibuat.

### Langkah 4: Jalankan Program
Jalankan program dengan hak akses yang memadai:
```bash
./mainhmi
```

Saat berhasil berjalan, terminal akan menampilkan banner sistem dan CLI prompt `>`:
```
===========================================
  SISTEM KONTROL REHABILITASI
  Multi-Trajectory + Cycle Counter
  + Admittance Control Support
  + Telemetri Arduino -> console mini PC
  Modular Version
===========================================

=========================================
  CLI EXPERIMENT CONTROLLER
  Ketik 'help' untuk daftar command
=========================================
> 
```

### Langkah 5: Operasi via HMI & CLI

1. **Hubungkan HMI**: Sambungkan HMI ke IP mini PC pada port `5020` (Modbus TCP).
2. **Kalibrasi**: Tekan tombol *Calibrate* di HMI (atau ketik `reset` di CLI terminal).
3. **Pilih Trajektori**: Pilih Trajektori 1, 2, atau 3 serta jumlah cycle di HMI.
4. **Mulai Latihan**: Tekan tombol *Start* pada HMI untuk memulai siklus rehabilitasi.

---

## 💻 Panduan CLI (Experiment Mode)

Untuk keperluan penelitian dan pembuatan paper, operator dapat mengubah mode kendali langsung dari terminal mini PC secara *real-time* tanpa menghentikan program:

### Daftar Perintah CLI

| Perintah | Fungsi | Serial Arduino |
|---|---|---|
| `admittance off` | **Mematikan** kontrol admittance (uji tanpa admittance) | `ADMITTANCE_OFF` |
| `admittance on` | **Mengaktifkan** kontrol admittance (uji dengan admittance) | `ADMITTANCE_ON` |
| `admittance reset` | Reset parameter admittance ke nilai standar | `ADMITTANCE_RESET` |
| `admittance status` | Menampilkan parameter aktif ($K, B, Z, \tau, F_{ext}$) | `ADMITTANCE_STATUS` |
| `set activation <val>` | Mengubah aktivasi otot $a \in [0.01, 1.0]$ | `ADM,A,<val>` |
| `set k <val>` | Override nilai kekakuan virtual $K$ (N/m) | `ADM,K,<val>` |
| `set b <val>` | Override nilai redaman virtual $B$ (N·s/m) | `ADM,B,<val>` |
| `set gain <val>` | Mengatur pengali gain admittance | `ADM,G,<val>` |
| `status` / `mode` | Melihat status mode pengujian saat ini | *(Lokal)* |
| `stop` | Menghentikan pergerakan motor | `0` |
| `emergency` | Mengaktifkan Emergency Stop | `E` |
| `reset` | Reset sistem ke posisi awal | `X` |
| `help` | Menampilkan daftar perintah yang tersedia | *(Lokal)* |
| `quit` / `exit` | Menghentikan motor dan keluar dari program | `0` + exit |

*Catatan: Semua perintah tidak sensitif huruf besar/kecil (case-insensitive).*

---

## 📊 Pengambilan Data & Logging untuk Paper

Program otomatis menyimpan data telemetri berkecepatan 10 Hz ke file CSV dengan format:
`telemetry_YYYYMMDD_HHMMSS.csv`

### Penandaan Mode Otomatis (`adm_mode`)
Kolom terakhir CSV (`adm_mode`) otomatis mencatat label kondisi eksperimen:
- **`ADM_ON`**: Data diambil saat sistem menggunakan kontrol admittance.
- **`ADM_OFF`**: Data diambil saat kontrol admittance dinonaktifkan.

### Alur Eksperimen Perbandingan
1. Jalankan `./mainhmi`.
2. Ketik `admittance off` di terminal mini PC.
3. Mulai latihan dari HMI $\rightarrow$ data tersimpan dengan tag `ADM_OFF`.
4. Setelah selesai, ketik `admittance on` di terminal mini PC.
5. Jalankan latihan kembali dengan profil gaya/beban yang sama $\rightarrow$ data tersimpan dengan tag `ADM_ON`.
6. Buka file CSV dan lakukan plotting perbandingan di Python/MATLAB:

```python
import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("telemetry_20261120_210000.csv")

# Filter data
data_with_adm    = df[df['adm_mode'] == 'ADM_ON']
data_without_adm = df[df['adm_mode'] == 'ADM_OFF']

# Plotting interaksi gaya dan tracking error
plt.figure()
plt.plot(data_without_adm['timestamp_ms'], data_without_adm['load'], label='Tanpa Admittance')
plt.plot(data_with_adm['timestamp_ms'], data_with_adm['load'], label='Dengan Admittance')
plt.xlabel('Waktu (ms)')
plt.ylabel('Gaya Interaksi (unit)')
plt.legend()
plt.title('Perbandingan Gaya Interaksi Pengguna')
plt.show()
```

---

## 📖 Dokumentasi Terkait
- [Dokumentasi Lengkap CLI](docs/cli.md)
- [Penurunan Matematis Model Admittance (Hill-Zajac)](docs/admittance.md)
- [Panduan Penggunaan Detail HMI & Register Modbus](docs/penggunaan.md)
- [Panduan Instalasi & Troubleshooting Dependensi](docs/instalasi.md)
