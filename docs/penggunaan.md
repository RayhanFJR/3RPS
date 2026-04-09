# Panduan Penggunaan

Dokumen ini menjelaskan cara menggunakan Sistem Kontrol Rehabilitasi 3RPS Modular.

## Persiapan

### 1. Pastikan Hardware Terhubung

- **Arduino**: Terhubung ke port serial yang dikonfigurasi
- **HMI**: Terhubung ke jaringan yang sama (untuk Modbus TCP)
- **Platform 3RPS**: Siap dan terkalibrasi

### 2. Verifikasi Konfigurasi

Pastikan konfigurasi di `include/Config.h` sesuai dengan setup Anda:
- Port serial Arduino
- Port Modbus TCP
- Path data trajectory

### 3. Jalankan Program

```bash
cd 3RPS_Modular
./mainhmi
```

Program akan menampilkan:

```
===========================================
  SISTEM KONTROL REHABILITASI
  Multi-Trajectory + Cycle Counter
  Modular Version
===========================================
```

## Operasi Dasar

### State Sistem

Sistem memiliki beberapa state:

1. **IDLE**: State awal, menunggu perintah
2. **RESETTING**: Reset sistem dari emergency stop
3. **AUTO_REHAB**: Rehabilitasi otomatis berjalan
4. **AUTO_RETREAT**: Retret otomatis
5. **POST_REHAB_DELAY**: Delay setelah rehabilitasi
6. **EMERGENCY_STOP**: Emergency stop aktif

### Kontrol melalui HMI

Sistem dikontrol melalui register Modbus yang dapat diakses dari HMI:

#### Register Kontrol

| Register | Address | Fungsi |
|----------|---------|--------|
| MANUAL_MAJU | 99 | Gerak maju manual |
| MANUAL_STOP | 100 | Stop manual |
| MANUAL_MUNDUR | 101 | Gerak mundur manual |
| CALIBRATE | 102 | Kalibrasi sistem |
| START | 103 | Mulai rehabilitasi |
| EMERGENCY | 104 | Emergency stop |
| RESET | 105 | Reset dari emergency |

#### Register Trajectory

| Register | Address | Fungsi |
|----------|---------|--------|
| TRAJEKTORI_1 | 106 | Pilih trajectory 1 |
| TRAJEKTORI_2 | 107 | Pilih trajectory 2 |
| TRAJEKTORI_3 | 108 | Pilih trajectory 3 |

#### Register Threshold

| Register | Address | Fungsi |
|----------|---------|--------|
| THRESHOLD_1 | 130 | Set threshold 1 |
| THRESHOLD_2 | 131 | Set threshold 2 |

#### Register Status

| Register | Address | Fungsi |
|----------|---------|--------|
| JUMLAH_CYCLE | 132 | Jumlah cycle rehabilitasi |
| DYNAMIC_THRESH_1 | 134-135 | Real-time nilai threshold 1 Dinamis (Float) |
| DYNAMIC_THRESH_2 | 136-137 | Real-time nilai threshold 2 Dinamis (Float) |
| RATE_OF_CHANGE | 138-139 | Real-time nilai laju perubahan gaya N/s (Float) |


## Operasi Manual

### 1. Kalibrasi Sistem

1. Pastikan platform dalam posisi awal
2. Set register `CALIBRATE` (102) ke `1` dari HMI
3. Sistem akan melakukan kalibrasi
4. Tunggu hingga kalibrasi selesai

### 2. Kontrol Manual

**Gerak Maju:**
- Set register `MANUAL_MAJU` (99) ke `1`
- Platform akan bergerak maju
- Set ke `0` untuk stop

**Gerak Mundur:**
- Set register `MANUAL_MUNDUR` (101) ke `1`
- Platform akan bergerak mundur
- Set ke `0` untuk stop

**Stop:**
- Set register `MANUAL_STOP` (100) ke `1`
- Platform akan berhenti

### 3. Pilih Trajectory

Sebelum memulai rehabilitasi, pilih trajectory:

- **Trajectory 1**: Set register `TRAJEKTORI_1` (106) ke `1`
- **Trajectory 2**: Set register `TRAJEKTORI_2` (107) ke `1`
- **Trajectory 3**: Set register `TRAJEKTORI_3` (108) ke `1`

Sistem akan otomatis switch ke trajectory yang dipilih.

## Operasi Rehabilitasi Otomatis

### 1. Setup Threshold

Sebelum memulai, set threshold untuk kontrol:

- Set `THRESHOLD_1` (130) dengan nilai threshold pertama
- Set `THRESHOLD_2` (131) dengan nilai threshold kedua

### 2. Mulai Rehabilitasi

1. Pastikan trajectory sudah dipilih
2. Pastikan threshold sudah diset
3. Set register `START` (103) ke `1` dari HMI
4. Sistem akan masuk ke state `AUTO_REHAB`

### 3. Alur Rehabilitasi

Saat rehabilitasi berjalan:

1. **AUTO_REHAB**: 
   - Sistem mengikuti trajectory yang dipilih
   - Data posisi, kecepatan, dan force dikirim ke Arduino
   - Grafik real-time diupdate di HMI
   - Sistem memantau feedback dari Arduino

2. **AUTO_RETREAT**:
   - Jika threshold tercapai, sistem masuk ke retreat
   - Platform bergerak mundur sesuai sequence

3. **POST_REHAB_DELAY**:
   - Setelah retreat, sistem delay 5 detik (default)
   - Delay dapat dikonfigurasi di `Config.h`

4. **Kembali ke AUTO_REHAB**:
   - Setelah delay, siklus berlanjut
   - Cycle counter bertambah

### 4. Monitor Cycle

Jumlah cycle dapat dibaca dari register `JUMLAH_CYCLE` (132).

## Grafik HMI

### Channel 0 (CH0) - Trajectory Reference

- Menampilkan data trajectory yang dipilih
- Data dimuat saat trajectory dipilih
- Start index dan end index sesuai konfigurasi trajectory

### Channel 1 (CH1) - Real-time Animation

- Menampilkan data real-time dari sistem
- Update setiap 100ms (default)
- Menampilkan posisi aktual platform

### Load Cell Real-time

- Register `REALTIME_LOAD_CELL` (126) menampilkan data load cell real-time

## Emergency Stop

### Mengaktifkan Emergency Stop

1. Set register `EMERGENCY` (104) ke `1` dari HMI
2. Sistem akan langsung masuk ke state `EMERGENCY_STOP`
3. Platform akan berhenti segera
4. Command "E" dikirim ke Arduino

### Reset dari Emergency Stop

1. Pastikan kondisi aman
2. Set register `RESET` (105) ke `1`
3. Sistem akan masuk ke state `RESETTING`
4. Command "R" dikirim ke Arduino
5. Sistem kembali ke state `IDLE`

## Monitoring dan Debugging

### Console Output

Program menampilkan informasi di console:

- State transitions
- Emergency stop activation
- Error messages
- Arduino feedback

### Modbus Register Monitoring

Monitor register berikut untuk status sistem:

- State register (jika ada)
- Cycle counter
- Load cell real-time
- Error flags (jika ada)

## Troubleshooting

### Platform Tidak Bergerak

1. Cek koneksi serial Arduino
2. Cek apakah Arduino menerima command
3. Cek log console untuk error
4. Verifikasi data trajectory ter-load

### HMI Tidak Terhubung

1. Cek koneksi jaringan
2. Cek port Modbus (default: 5020)
3. Cek firewall settings
4. Verifikasi IP address HMI

### Trajectory Tidak Berubah

1. Pastikan register trajectory di-set ke `1`
2. Cek apakah data trajectory ter-load
3. Verifikasi path data trajectory

### Grafik Tidak Update

1. Cek koneksi Modbus
2. Verifikasi register grafik
3. Cek timing update (default: 100ms)
4. Pastikan data trajectory valid

## Best Practices

1. **Selalu kalibrasi** sebelum menggunakan sistem
2. **Pilih trajectory** sebelum memulai rehabilitasi
3. **Set threshold** sesuai kebutuhan
4. **Monitor cycle counter** untuk tracking
5. **Gunakan emergency stop** jika diperlukan
6. **Verifikasi hardware** sebelum operasi
7. **Backup data trajectory** secara berkala

## Konfigurasi Lanjutan

### Timing Configuration

Edit `include/Config.h` untuk mengubah timing:

```cpp
const int JEDA_KONTROLER_MS = 100;  // Interval kontroler (ms)
const int JEDA_GRAFIK_MS = 100;     // Interval update grafik (ms)
const int POST_REHAB_DELAY_SEC = 5; // Delay setelah rehab (detik)
```

### Trajectory Configuration

Setiap trajectory memiliki konfigurasi index untuk grafik:

- `GRAFIK_START_INDEX` / `GRAFIK_END_INDEX`: Range data untuk grafik
- `GAIT_START_INDEX` / `GAIT_END_INDEX`: Range data untuk gait

Edit di `include/Config.h` sesuai kebutuhan.

## Next Steps

- [Struktur Modul](STRUCTURE.md) - Memahami detail implementasi
- [Index](index.md) - Kembali ke halaman utama

