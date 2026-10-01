🇮🇩 Bahasa Indonesia | [🇬🇧 English](README.en.md)

# BME280 STM32 HAL Library

Driver C ringan dan project contoh untuk menghubungkan sensor lingkungan Bosch BME280 dengan mikrokontroler STM32 melalui API I2C STM32 HAL.

Project yang disertakan menargetkan **STM32F401CCU6** dan mendemonstrasikan pembacaan suhu, tekanan barometrik, kelembapan relatif, dan estimasi ketinggian. Hasil pengukuran tersedia lewat USART2 dan keluaran debug SWV/ITM.

[![Language](https://img.shields.io/badge/language-C-blue.svg)](https://github.com/mjmokhtar/bme280-lib-hal)
[![Platform](https://img.shields.io/badge/platform-STM32-orange.svg)](https://www.st.com/stm32)
[![License](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)

## Fitur

- Komunikasi I2C berbasis STM32 HAL
- Alamat I2C BME280 `0x76` dan `0x77`
- Verifikasi chip ID dan software reset
- Pemuatan data kalibrasi pabrik
- Pembacaan suhu, tekanan, dan kelembapan terkompensasi
- Estimasi ketinggian menggunakan tekanan permukaan laut
- Oversampling, filter IIR, standby time, dan power mode yang dapat dikonfigurasi
- Pengecekan koneksi dan status pengukuran
- Contoh output lewat USART2 dan SWV/ITM
- Scan I2C otomatis saat inisialisasi gagal

## Konfigurasi Hardware

| Komponen | Konfigurasi |
| --- | --- |
| Mikrokontroler | STM32F401CCU6 |
| Sensor | Bosch BME280 |
| Antarmuka | I2C1 pada 100 kHz |
| I2C SCL | PB6 |
| I2C SDA | PB7 |
| Output debug | USART2 pada 115200 baud |
| USART2 TX / RX | PA2 / PA3 |
| System clock | 84 MHz |

## Wiring

| BME280 | STM32F401CCU6 | Deskripsi |
| --- | --- | --- |
| VIN / VCC | 3.3 V | Daya sensor |
| GND | GND | Ground |
| SCL / SCK | PB6 | Clock I2C1 |
| SDA / SDI | PB7 | Data I2C1 |
| CSB | 3.3 V | Memilih mode I2C bila diperlukan |
| SDO | GND atau 3.3 V | Memilih alamat `0x76` atau `0x77` |

> Gunakan logika 3.3 V. Banyak modul BME280 sudah menyertakan resistor pull-up I2C; sensor polos memerlukan pull-up yang sesuai pada SDA dan SCL.

## Struktur Repositori

```text
.
├── Core
│   ├── Inc
│   │   └── BME280_STM32.h       # API driver dan definisi
│   └── Src
│       ├── BME280_STM32.c       # Implementasi driver
│       └── main.c               # Aplikasi contoh
├── Drivers                      # STM32F4 HAL dan CMSIS
├── i2c_bme280_test.ioc          # Konfigurasi STM32CubeMX
├── STM32F401CCUX_FLASH.ld       # Linker script
└── LICENSE
```

## Memulai

### 1. Clone repositori

```bash
git clone https://github.com/mjmokhtar/bme280-lib-hal.git
cd bme280-lib-hal
```

### 2. Buka project

Import repositori sebagai existing project di **STM32CubeIDE**. Konfigurasi aslinya memakai STM32CubeMX 6.9.2 dan STM32Cube FW_F4 V1.27.1.

Jika perlu, buka `i2c_bme280_test.ioc` lalu regenerate kode inisialisasi sesuai firmware package STM32F4 yang terpasang.

### 3. Hubungkan dan konfigurasi sensor

Hubungkan BME280 ke PB6 dan PB7. Contoh memakai alamat `0x76`:

```c
BME280_ADDRESS_PRIMARY
```

Gunakan `BME280_ADDRESS_SECONDARY` untuk modul yang dikonfigurasi sebagai `0x77`.

### 4. Build, flash, dan monitor

Build dan flash firmware dengan STM32CubeIDE dan programmer ST-Link. Buka terminal serial pada **115200 baud, 8-N-1**. Pengukuran diperbarui kira-kira setiap dua detik.

Contoh output:

```text
BME280 initialization successful!
BME280 Chip ID: 0x60

=== Loop #0 ===
Temperature: 27.42 C
Pressure: 100742.31 Pa (1007.42 hPa)
Humidity: 68.15%
Altitude: 49.08 m
```

## Memakai Driver di Project Lain

Salin file-file ini ke project STM32 kamu:

```text
Core/Inc/BME280_STM32.h
Core/Src/BME280_STM32.c
```

Inisialisasi I2C sebelum memanggil driver:

```c
#include "BME280_STM32.h"

BME280_HandleTypeDef bme280;

if (BME280_Init(&bme280, &hi2c1, BME280_ADDRESS_PRIMARY)) {
    float temperature = BME280_ReadTemperature(&bme280);          // °C
    float pressure    = BME280_ReadPressure(&bme280);             // Pa
    float humidity    = BME280_ReadHumidity(&bme280);             // %RH
    float altitude    = BME280_ReadAltitude(&bme280, 1013.25f);   // m
}
```

Berikan alamat 7-bit normal ke `BME280_Init()`. Driver menggesernya sendiri karena STM32 HAL mengharapkan alamat dalam bentuk 8-bit.

Untuk keluarga STM32 lain, ganti include ini di `BME280_STM32.h`:

```c
#include "stm32f4xx_hal.h"
```

dengan header HAL yang sesuai, seperti `stm32f1xx_hal.h` atau `stm32l4xx_hal.h`.

## Konfigurasi Default

`BME280_Init()` menerapkan:

| Pengaturan | Default |
| --- | --- |
| Oversampling suhu | ×16 |
| Oversampling tekanan | ×16 |
| Oversampling kelembapan | ×16 |
| Filter IIR | Koefisien 16 |
| Standby time | 0.5 ms |
| Power mode | Normal |

Konfigurasi dapat diubah setelah inisialisasi:

```c
BME280_SetMode(&bme280, BME280_MODE_NORMAL);
BME280_SetOversamplingTemperature(&bme280, BME280_OVERSAMP_4X);
BME280_SetOversamplingPressure(&bme280, BME280_OVERSAMP_4X);
BME280_SetOversamplingHumidity(&bme280, BME280_OVERSAMP_2X);
BME280_SetFilter(&bme280, BME280_FILTER_4);
BME280_SetStandbyTime(&bme280, BME280_STANDBY_1000);
```

## API Utama

### Inisialisasi dan status

```c
bool BME280_Init(BME280_HandleTypeDef *bme, I2C_HandleTypeDef *hi2c, uint8_t address);
bool BME280_IsConnected(BME280_HandleTypeDef *bme);
bool BME280_IsMeasuring(BME280_HandleTypeDef *bme);
uint8_t BME280_GetChipID(BME280_HandleTypeDef *bme);
void BME280_Reset(BME280_HandleTypeDef *bme);
```

### Pengukuran

```c
float BME280_ReadTemperature(BME280_HandleTypeDef *bme);
float BME280_ReadPressure(BME280_HandleTypeDef *bme);
float BME280_ReadHumidity(BME280_HandleTypeDef *bme);
float BME280_ReadAltitude(BME280_HandleTypeDef *bme, float seaLevelPressure);
```

## Akurasi Ketinggian

Ketinggian diperkirakan dari tekanan atmosfer. Contoh memakai `1013.25 hPa`, yaitu tekanan rata-rata standar permukaan laut:

```c
BME280_ReadAltitude(&bme280, 1013.25f);
```

Untuk akurasi lokal yang lebih baik, gunakan tekanan permukaan laut terkini dari stasiun cuaca terdekat. Ketinggian BME280 hanyalah estimasi, bukan pengganti GNSS atau elevasi hasil survei.

## Pemecahan Masalah

### Inisialisasi gagal

- Pastikan modul diberi daya dengan logika 3.3 V yang kompatibel.
- Periksa sambungan SDA, SCL, dan GND.
- Coba kedua alamat, `0x76` dan `0x77`.
- Pastikan SDA dan SCL memiliki resistor pull-up.
- Pastikan I2C1 sudah diinisialisasi sebelum `BME280_Init()`.
- Periksa output serial; contoh akan men-scan bus setelah terjadi kegagalan.

### Tekanan berfungsi tapi kelembapan salah

Pastikan perangkat adalah **BME280**, bukan BMP280. BMP280 tidak memiliki sensor kelembapan. Chip ID BME280 yang diharapkan adalah `0x60`.

### Tidak ada output UART

- Gunakan baud rate 115200.
- Pastikan USART2 TX tersedia di PA2.
- Pastikan board dan adaptor serial berbagi ground yang sama.

## Catatan

- Driver memakai panggilan I2C STM32 HAL yang blocking.
- Pembacaan tekanan dan kelembapan memperbarui nilai kompensasi suhu secara internal.
- Implementasi saat ini mendukung I2C; SPI belum diimplementasikan.
- Header menargetkan STM32F4 HAL tetapi dapat diadaptasi ke keluarga STM32 HAL lain.

## Kontribusi

Issue dan pull request sangat diterima. Sertakan target STM32, alamat sensor, wiring, dan langkah reproduksi saat melaporkan masalah.

## Lisensi

Didistribusikan di bawah [MIT License](LICENSE).

Copyright © 2025 Muhammad Jumi'at Mokhtar.
