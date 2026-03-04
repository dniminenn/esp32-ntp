## esp32-ntp

High-precision NTP server for ESP32 using GPS PPS discipline, with support for WIZnet W5500 Ethernet and WiFi STA, an optional MAX7219 LED matrix display, and an HTTP stats endpoint. PPS edges are captured in hardware via the MCPWM capture peripheral (12.5 ns resolution, zero ISR jitter). Hardware-assisted transmit timestamp correction is implemented for the W5500 path; the same NTP server and metrics are available over either interface.

### Features

- **NTP server** on UDP port 123, serving time disciplined by a GPS receiver with PPS.
- **MCPWM hardware PPS capture** — the ESP32's MCPWM capture peripheral latches a counter at the exact GPIO edge in silicon, providing 12.5 ns (80 MHz APB clock) resolution with zero ISR-latency jitter. The PI servo, jitter estimator, and outlier rejection all operate on hardware-captured tick deltas.
- **Hardware timestamping logic** that compensates for PPS frequency error and ARP/Tx latency when using the W5500 Ethernet controller.
- **HTTP stats endpoint** on TCP port 8080 exposing Prometheus-style metrics for lock state, offsets, jitter, frequency, uptime, and request counts (served over whichever interface is selected).
- **Optional LED display** (MAX7219 8×8 matrices on SPI) showing current time and a top-row centisecond uptime indicator.
- Runs directly on **ESP32 + W5500** with no Wi‑Fi or SNTP dependency once GPS is locked.

### Hardware overview

- **MCU**: ESP32 (IDF target `esp32`).
- **Ethernet**: Optional WIZnet W5500 on a dedicated SPI bus (HSPI by default). When selected as the network interface, this path uses the W5500 hardware-friendly transmit timing and timestamping logic.
- **GPS**: UART GPS module (e.g. NEO‑6M) plus PPS input GPIO. PPS is captured via the MCPWM peripheral — any GPIO works (including input-only pins 34–39).
- **Display (optional)**: Up to 4× MAX7219 8×8 matrices on a separate SPI bus.

Default pins are defined in `config.cpp` and can be overridden via `menuconfig` (see below).

### Build and flash

Prerequisites:
- ESP-IDF installed (tested with a v6.0-dev toolchain).
- `IDF_PATH` pointing at your ESP-IDF checkout.

You can use the provided `Makefile` helpers:

```bash
make build          # idf.py set-target && idf.py build
make flash          # flash firmware to ESP32
make monitor        # open serial console
make flash-monitor  # flash then monitor
make menuconfig     # open project configuration
```

Or call `idf.py` directly if you prefer.

### Configuration

Project-level options live under **`esp32-ntp configuration`** in `menuconfig` (`make menuconfig`):

- **Network**
  - **Network interface**: **WIZnet W5500 Ethernet** or **WiFi STA**. IP configuration works the same either way (DHCP or static). Hardware timestamping is only used on the W5500 path; WiFi uses the same GPS/PPS discipline without the W5500-specific transmit timing trick.
  - **IP config**: DHCP by default, or static IP, gateway, and netmask when **Use static IP** is enabled. Applies to whichever interface is selected.
  - With **WiFi STA**: set **WiFi SSID** and **WiFi password**.

- **NTP / timezone**
  - `APP_NTP_SERVER`, `APP_NTP_PORT`: reserved (not currently used; device acts as a server).
  - `APP_TZ`: POSIX timezone string. Examples:
    - Eastern US: `EST5EDT,M3.2.0,M11.1.0`
    - Central Europe: `CET-1CEST,M3.5.0,M10.5.0/3`
    - UK: `GMT0BST,M3.5.0/1,M10.5.0/2`
    - Atlantic Canada: `AST4ADT,M3.2.0,M11.1.0`

- **SPI / Display**
  - `APP_USE_DISPLAY`: enable or disable the MAX7219 LED display.
  - `APP_SPI_HOST`, `APP_SPI_MOSI_PIN`, `APP_SPI_SCLK_PIN`, `APP_CS_PIN`, `APP_MAX_DEVICES`, `APP_SPI_CLOCK_HZ`: SPI host and pinout for the display.

Ethernet and GPS pin assignments are in `Config::getW5500*` and `Config::getGps*` in `components/config/config.cpp`. When using WiFi, only SSID/password from menuconfig apply.

### Runtime behavior

- On boot, `app_main`:
  - Initializes NVS, time baseline, and the ESP-IDF networking stack.
  - Brings up either the W5500 via `W5500Eth` or WiFi STA via `WifiSta` according to **Network interface** in menuconfig, and configures IP using DHCP by default or the static IP settings when enabled.
  - Applies the configured timezone.
  - Optionally initializes the LED display and spawns a display task if `APP_USE_DISPLAY` is enabled.
  - Starts GPS/PPS disciplining (`GpsDiscipline`), the NTP server (`NtpServer`), and the stats HTTP server (`NtpStats`).

- **NTP server**:
  - Listens on UDP port 123 on whichever interface is selected (W5500 Ethernet or WiFi STA).
  - Uses GPS PPS data and a monotonic timer to synthesize NTP timestamps in the 1900 epoch.
  - On the W5500 path, uses a non-blocking send and may resend with a corrected transmit timestamp after ARP completes; on the WiFi path it uses a single send via the ESP-IDF lwIP stack.

- **Stats HTTP server**:
  - Listens on TCP port 8080 on whichever interface is selected (W5500 or WiFi).
  - Serves a Prometheus text exposition format with metrics such as:
    - `ntp_clock_offset_seconds`
    - `ntp_rms_offset_seconds`
    - `ntp_frequency_ppm`
    - `ntp_pps_jitter_seconds`
    - `ntp_root_dispersion_seconds`
    - `ntp_gps_lock`
    - `ntp_stratum`
    - `ntp_uptime_seconds`
    - `ntp_requests_total`
    - `ntp_pps_count`

### Code structure

- `main/app_main.cpp` – system bring-up, component wiring, main loop.
- `components/config` – pinout and configuration accessors (`Config::…`).
- `components/gps` – GPS NMEA parsing and PPS disciplining logic. Uses MCPWM hardware capture for jitter-free PPS interval measurement and a PI servo for clock steering.
- `components/ntp_server` – NTP server implementation over W5500 UDP or WiFi UDP (lwIP), depending on the selected interface.
- `components/ntp_stats` – HTTP stats server over W5500 TCP or WiFi TCP (lwIP), depending on the selected interface.
- `components/w5500_eth` – W5500 SPI driver, DHCP and static IP handling, and PHY setup.
- `components/wifi_sta` – WiFi STA with DHCP or static IP and `getIpAddr()` for stats server.
- `components/w5k` – thin UDP/TCP wrapper API over the WIZnet ioLibrary sockets.
- `components/display` – MAX7219 display driver and rendering helpers.
- `ioLibrary_Driver` – WIZnet W5x00 reference driver (git submodule; licensed separately by WIZnet).

### License

All original code in this repository is licensed under **MIT No Attribution (MIT-0)** – see `LICENSE` for details.  
The `ioLibrary_Driver` submodule and any other bundled third-party components retain their own upstream licenses.

