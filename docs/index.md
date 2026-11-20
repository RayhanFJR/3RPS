# Dokumentasi Sistem Kontrol Rehabilitasi 3RPS Modular

## Selamat Datang

Dokumentasi ini menjelaskan sistem kontrol rehabilitasi robot paralel 3-RPS versi modular. Sistem ini dirancang untuk mengontrol platform rehabilitasi pergelangan kaki (*ankle rehabilitation*) dengan komunikasi **Modbus TCP** untuk HMI, komunikasi **serial** dengan Arduino, integrasi **Admittance Control** (model Hill-Zajac), serta **CLI Experiment Controller** di mini PC untuk pengujian komparatif (*with vs. without admittance*).

---

## 📚 Daftar Isi

1. [Panduan Instalasi](instalasi.md) - Panduan instalasi dependensi dan kompilasi sistem
2. [Panduan Penggunaan](penggunaan.md) - Panduan penggunaan sistem via HMI dan alur rehabilitasi
3. [Panduan CLI & Eksperimen](cli.md) - Panduan pengoperasian terminal CLI dan pengujian untuk paper
4. [Penurunan Kontrol Admittance](admittance.md) - Penurunan matematis kekakuan ($K$) dan redaman ($B$) virtual Hill-Zajac
5. [Struktur Modul & Arsitektur](STRUCTURE.md) - Dokumentasi modul, kelas, dan diagram alur data

---

## 🎯 Overview Sistem

Sistem Kontrol Rehabilitasi 3RPS Modular adalah aplikasi C++ performa tinggi yang berjalan di mini PC dengan fitur-fitur utama:

### Fitur Utama

- **Multi-Trajectory Support**: Mendukung 3 trajektori latihan rehabilitasi berbeda yang dapat dipilih secara dinamis dari HMI.
- **Admittance Control (Hill-Zajac)**: Kontrol admittance orde pertama ($M=0$) dengan nilai kekakuan ($K$) dan redaman ($B$) berbasis model muskuloskeletal aktual.
- **CLI Experiment Controller**: Antarmuka terminal interaktif di mini PC untuk *toggle* admittance ON/OFF secara instan saat *runtime*.
- **Automatic CSV Telemetry Logging**: Pencatatan telemetri 10 Hz ke CSV lengkap dengan tag `ADM_ON` / `ADM_OFF` untuk mempermudah analisis data paper.
- **Modbus TCP Communication**: Server Modbus TCP untuk komunikasi dua arah dengan HMI (register kendali, grafik referensi, dan animasi aktual).
- **Serial Communication (USB CDC)**: Komunikasi serial stabil dengan Arduino Leonardo pada baud rate 115200.
- **Safety & Auto-Retreat**: Deteksi hentakan tiba-tiba (*yank* $dF/dt$), batas gaya, dan mekanisme *go-to-zero retreat* otomatis jika terjadi anomali beban.
- **Emergency Stop & Recovery**: Penanganan E-stop baik dari HMI maupun CLI terminal.

---

## 🧩 Komponen Sistem

1. **TrajectoryManager**: Mengelola pemuatan (*loading*) dan pemilihan trajektori dari file data (`pos`, `velo`, `fc`, `grafik`).
2. **SerialHandler**: Menangani komunikasi serial USB dengan Arduino, parsing data telemetri, dan pencatatan CSV.
3. **CLIHandler**: Mengelola input operator dari terminal mini PC dalam thread terpisah (*thread-safe command queue*).
4. **ModbusHandler**: Server Modbus TCP untuk pertukaran data dengan HMI.
5. **GraphManager**: Mengelola buffer data grafik referensi (CH0) dan animasi real-time (CH1) di HMI.
6. **ControlHandler**: Logika kontrol alur rehabilitasi, transisi siklus, dan urutan *retreat*.
7. **StateHandlers**: Logika eksekusi pada setiap state mesin (*IDLE*, *AUTO_REHAB*, *AUTO_RETREAT*, *POST_REHAB_DELAY*, *RESETTING*).

---

## ⚡ Quick Start

1. Hubungkan Arduino ke port USB mini PC (default: `/dev/ttyACM0`).
2. Buka terminal di folder proyek:
   ```bash
   make clean
   make
   ./mainhmi
   ```
3. Di terminal yang sama, gunakan perintah CLI seperti:
   - `admittance off` (untuk uji tanpa admittance)
   - `admittance on` (untuk uji dengan admittance)
   - `status` (melihat mode aktif)
4. Jalankan siklus rehabilitasi dari panel HMI.
