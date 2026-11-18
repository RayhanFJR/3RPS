# CLI Experiment Controller

Modul CLI memungkinkan operator mengirim perintah langsung dari terminal mini PC saat program `mainhmi` sedang berjalan. Fitur utamanya adalah toggling admittance control ON/OFF untuk keperluan eksperimen perbandingan pada paper.

## Cara Kerja

CLI berjalan sebagai **thread terpisah** yang membaca input dari `stdin`. Command yang diketik operator masuk ke antrian (thread-safe queue), kemudian diproses oleh main loop dan diteruskan ke Arduino via serial.

```
Terminal (stdin)  →  CLIHandler (queue)  →  Main Loop  →  Serial  →  Arduino
```

Main loop tetap berjalan normal (~100Hz) tanpa terganggu oleh blocking stdin.

## Daftar Command

### Experiment Control

| Command | Deskripsi | Serial ke Arduino |
|---|---|---|
| `admittance on` | Aktifkan admittance control | `ADMITTANCE_ON` |
| `admittance off` | Matikan admittance control | `ADMITTANCE_OFF` |
| `admittance reset` | Reset K, B, activation ke default (a=0.3) | `ADMITTANCE_RESET` |
| `admittance status` | Print detail status admittance (K, B, Z, tau, dll.) | `ADMITTANCE_STATUS` |

### Parameter Tuning

| Command | Deskripsi | Serial ke Arduino |
|---|---|---|
| `set activation <val>` | Set aktivasi otot (0.01–1.0). Mengubah K dan B secara proporsional. | `ADM,A,<val>` |
| `set k <val>` | Override stiffness K secara langsung (N/m) | `ADM,K,<val>` |
| `set b <val>` | Override damping B secara langsung (N·s/m) | `ADM,B,<val>` |
| `set gain <val>` | Set admittance gain multiplier | `ADM,G,<val>` |

### System Control

| Command | Deskripsi | Serial ke Arduino |
|---|---|---|
| `status` / `mode` | Print mode eksperimen saat ini (lokal) | — |
| `reset` | System reset | `X` |
| `stop` | Stop semua motor | `0` |
| `emergency` / `estop` | Emergency stop | `E` |
| `help` | Tampilkan daftar command (lokal) | — |
| `quit` / `exit` | Stop motor + keluar program | `0` + exit |

> Semua command **case-insensitive**.

## CSV Mode Tagging

Setiap baris telemetri yang dicatat ke file CSV (`telemetry_YYYYMMDD_HHMMSS.csv`) memiliki kolom terakhir `adm_mode` yang berisi:

- `ADM_ON` — data diambil saat admittance control aktif
- `ADM_OFF` — data diambil saat admittance control nonaktif

Tag ini otomatis berubah saat operator mengetik `admittance on` atau `admittance off`.

### Contoh CSV Output

```csv
timestamp_ms,state,mode,load,yank,...,K,B,Z,tpause,adm_mode
1200,run,fwd,12.50,0.30,...,15556.00,7272.00,0.0012,0,ADM_ON
1300,run,fwd,13.10,0.80,...,15556.00,7272.00,0.0015,0,ADM_ON
...
45000,run,fwd,11.20,0.10,...,0.00,0.00,0.0000,0,ADM_OFF
45100,run,fwd,12.00,0.50,...,0.00,0.00,0.0000,0,ADM_OFF
```

### Filtering di Python

```python
import pandas as pd

df = pd.read_csv("telemetry_20261116_143012.csv")

df_on  = df[df["adm_mode"] == "ADM_ON"]
df_off = df[df["adm_mode"] == "ADM_OFF"]

# Plot perbandingan
import matplotlib.pyplot as plt

fig, ax = plt.subplots()
ax.plot(df_on["timestamp_ms"], df_on["load"], label="With Admittance")
ax.plot(df_off["timestamp_ms"], df_off["load"], label="Without Admittance")
ax.set_xlabel("Time (ms)")
ax.set_ylabel("Load")
ax.legend()
plt.show()
```

## Alur Eksperimen

1. Jalankan program: `./mainhmi`
2. Ketik `admittance off` di terminal → mode tanpa admittance
3. Start rehab dari HMI (N cycle) → data ter-tag `ADM_OFF`
4. Tunggu selesai
5. Ketik `admittance on` di terminal → mode dengan admittance
6. Start rehab dari HMI (N cycle) → data ter-tag `ADM_ON`
7. Buka CSV, filter berdasarkan kolom `adm_mode`, plot perbandingan

## Contoh Sesi Terminal

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
> admittance off
[CLI] Admittance OFF — mode: WITHOUT admittance

> status

--- Experiment Status ---
  Admittance mode : OFF (WITHOUT admittance)
  CSV tag         : ADM_OFF
  (Ketik 'admittance status' untuk detail K/B/Z dari Arduino)
-------------------------

> admittance on
[CLI] Admittance ON — mode: WITH admittance

> set activation 0.5
[CLI] Sent to Arduino: ADM,A,0.5

> admittance status
  === Admittance Status ===
  Enabled    : YES
  Activation : 0.500
  K_adm      : 25926.65 N/m
  B_adm      : 12120.00 N.s/m
  tau (B/K)  : 0.4674 s
  Z_adm      : 0.0000 mm
  ...
  =========================

> quit
[CLI] Shutting down...
[CLI] Motor stopped. Program exiting.
```

## Catatan Teknis

- CLI thread menggunakan `std::getline()` yang blocking — zero CPU cost saat idle.
- Command di-queue dengan `std::mutex` dan diproses di main loop thread, sehingga tidak ada race condition dengan pengiriman serial yang sudah ada.
- `quit` / `exit` mengirim command stop (`0`) ke Arduino sebelum memanggil `exit(0)`.
- Default saat program start: admittance **ON** (sesuai default Arduino `admittanceEnabled = true`).
