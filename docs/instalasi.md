# Panduan Instalasi

Dokumen ini menjelaskan langkah-langkah instalasi Sistem Kontrol Rehabilitasi 3RPS Modular.

## Persyaratan Sistem

### Sistem Operasi
- Linux (disarankan Ubuntu 18.04+ atau Debian 10+)
- Windows dengan WSL2 (untuk development)

### Kompiler
- GCC 7.0 atau lebih baru dengan dukungan C++11
- Make utility

### Dependencies

1. **libmodbus** - Library untuk komunikasi Modbus
2. **Boost Libraries** - Minimal boost::asio dan boost::system
3. **pthread** - Threading library (biasanya sudah terinstall)

## Instalasi Dependencies

### Ubuntu/Debian

```bash
# Update package list
sudo apt update

# Install build essentials
sudo apt install build-essential g++ make

# Install libmodbus
sudo apt install libmodbus-dev

# Install Boost libraries
sudo apt install libboost-all-dev
```

### Fedora/CentOS/RHEL

```bash
# Install build tools
sudo dnf install gcc-c++ make

# Install libmodbus
sudo dnf install libmodbus-devel

# Install Boost
sudo dnf install boost-devel
```

### Compile dari Source (Alternatif)

Jika paket tidak tersedia di repository, Anda dapat compile dari source:

#### libmodbus

```bash
# Download libmodbus
wget https://github.com/stephane/libmodbus/archive/v3.1.6.tar.gz
tar -xzf v3.1.6.tar.gz
cd libmodbus-3.1.6

# Configure, compile, dan install
./autogen.sh
./configure
make
sudo make install
sudo ldconfig
```

#### Boost

```bash
# Download Boost
wget https://sourceforge.net/projects/boost/files/boost/1.75.0/boost_1_75_0.tar.gz
tar -xzf boost_1_75_0.tar.gz
cd boost_1_75_0

# Bootstrap dan compile
./bootstrap.sh
./b2
sudo ./b2 install
```

## Setup Proyek

### 1. Clone atau Download Proyek

```bash
cd /path/to/workspace
# Pastikan struktur folder sudah benar
```

### 2. Verifikasi Struktur Folder

Pastikan struktur folder berikut ada:

```
3RPS_Modular/
├── include/
├── src/
├── data/
│   ├── data_1/
│   ├── data_2/
│   └── data_3/
├── main.cpp
└── Makefile
```

### 3. Verifikasi Data Trajectory

Pastikan folder `data/` berisi data trajectory yang diperlukan:

```bash
# Cek struktur data
ls -R data/
```

Setiap folder `data_X/` harus berisi:
- `pos1.txt`, `pos2.txt`, `pos3.txt` - Data posisi
- `velo1.txt`, `velo2.txt`, `velo3.txt` - Data kecepatan
- `fc1.txt`, `fc2.txt`, `fc3.txt` - Data force
- `grafik.txt` - Data grafik

### 4. Konfigurasi Path Data

Path data default diset ke `"data"` di `main.cpp`:

```cpp
if (!trajectoryManager.loadAllTrajectoryData("data")) {
    // Ubah path sesuai lokasi data Anda jika berbeda
}
```

## Kompilasi

### Build Program

```bash
cd 3RPS_Modular
make
```

Jika berhasil, akan menghasilkan executable `mainhmi`.

### Clean Build

Untuk membersihkan file build:

```bash
make clean
```

### Rebuild

```bash
make clean
make
```

## Konfigurasi Sistem

### 1. Konfigurasi Serial Port

Edit `include/Config.h` untuk mengatur port serial:

```cpp
// Line 73
const char* DEFAULT_SERIAL_PORT = "/dev/ttyACM0";  // Ubah sesuai port Arduino
const int SERIAL_BAUD_RATE = 115200;
```

**Catatan**: Pastikan user memiliki permission untuk mengakses serial port:

```bash
# Tambahkan user ke group dialout
sudo usermod -a -G dialout $USER
# Logout dan login kembali
```

### 2. Konfigurasi Modbus

Edit `include/Config.h` untuk mengatur Modbus:

```cpp
// Line 77-79
const char* MODBUS_HOST = "0.0.0.0";  // 0.0.0.0 untuk listen semua interface
const int MODBUS_PORT = 5020;         // Port Modbus TCP
const int MODBUS_SLAVE_ID = 1;
```

### 3. Firewall (jika diperlukan)

Jika menggunakan firewall, buka port Modbus:

```bash
# Ubuntu/Debian (ufw)
sudo ufw allow 5020/tcp

# CentOS/RHEL (firewalld)
sudo firewall-cmd --add-port=5020/tcp --permanent
sudo firewall-cmd --reload
```

## Verifikasi Instalasi

### 1. Cek Dependencies

```bash
# Cek libmodbus
pkg-config --modversion libmodbus

# Cek Boost
pkg-config --modversion boost
```

### 2. Test Kompilasi

```bash
make clean
make
```

Jika tidak ada error, instalasi berhasil.

### 3. Test Run (tanpa hardware)

Program dapat dijalankan untuk test, namun akan error jika:
- Port serial tidak tersedia
- Data trajectory tidak ditemukan

## Troubleshooting

### Error: libmodbus not found

```bash
# Install libmodbus-dev
sudo apt install libmodbus-dev

# Atau set LD_LIBRARY_PATH jika install dari source
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
```

### Error: boost/asio.hpp not found

```bash
# Install boost development libraries
sudo apt install libboost-all-dev
```

### Error: Permission denied (serial port)

```bash
# Tambahkan user ke group dialout
sudo usermod -a -G dialout $USER
# Logout dan login kembali, atau:
newgrp dialout
```

### Error: Cannot load trajectory data

- Pastikan path ke folder `data/` benar
- Pastikan semua file trajectory ada
- Cek permission folder dan file

### Error: Modbus bind failed

- Pastikan port 5020 tidak digunakan aplikasi lain
- Cek permission (port < 1024 memerlukan root)
- Cek firewall settings

## Next Steps

Setelah instalasi selesai, lanjutkan ke:
- [Penggunaan](penggunaan.md) - Panduan penggunaan sistem
- [Struktur Modul](STRUCTURE.md) - Memahami arsitektur sistem

