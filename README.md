# Redesigned from the original project by [StaRky33](https://github.com/StaRky33/ZigbeeMacropad)

A huge thank you for the work you put into this project and for generously sharing it as open and free software. 

# 🧠 Zigbee Macropad (ESP32-C6)

A compact **16-key Zigbee macropad** with a **Rotary Encoder**, powered by the **Seeed Studio XIAO ESP32-C6**.  
Designed for integration with **Home Assistant (ZIGBEE2MQTT)**, it operates via USB power and provides tactile mechanical key input, rotary control, and excellent wireless range thanks to an external antenna.

It gives 3 types of button inputs (single click, double click, long press) and rotary encoder events (left, right, press) to add into any automation to control devices remotely.

---
## ⚙️ Hardware

| Component | Description |
|------------|-------------|
| **MCU** | Seeed Studio XIAO ESP32C6 |
| **Switches** | 16 x Cherry MX Red (linear) |
| **Encoder** | 1 x EC11 Rotary Encoder |
| **Antenna** | 1 x 2.4GHz External Antenna |
| **Diodes**| 16 x 1N4148 Small Signal Fast Switching Diodes |
| **Threaded inserts** | 4 x M2.5 heat-set brass inserts |
| **Screws** | 4 x M2.5 × 5 mm machine screws |
| **Case** | 3D-printed PLA enclosure |
| **Keycaps**| 16 x Bought Cherry MX Keycaps |
| **Knob** | 1 x 3D-printed or standard Encode Knob |

## 💻 Code

Built with **VS Code and ESP-IDF v5.3.4**.

### 🧠 Software Stack
- **Framework:** ESP-IDF v5.3.4  
- **Zigbee SDK:** [Espressif ESP-Zigbee-SDK](https://github.com/espressif/esp-zigbee-sdk)  
- **Zigbee Role:** Router / End Device  
- **Endpoint Type:** Custom Endpoint and clusters

### 🔧 Functionality
- **16 Keys**: Detects **single**, **double**, and **long press**.
- **Rotary Encoder**: Detects **Rotation CW (Right)**, **Rotation CCW (Left)**.
- **Encoder Button**:
    - **Single Press**: Acts as a 17th button.
    - **Ultra Long Press (>6s)**: Triggers **Factory Reset / Pairing Mode**.
- **USB Powered**: No battery, permanently powered via USB-C.
- **Power Saving**: Enters "Fake Sleep" (light sleep) after 30 seconds of inactivity, and Deep Sleep after 6 hours.
- **External Antenna**: Configured for external antenna usage for improved range.
- **Debounce**: ISR-driven button logic for reliability.

---

## 🗺️ Circuit Wiring

### Key Matrix
wired in a 4x4 specific matrix.
| Rows (Outputs) | Columns (Inputs) |
|---|---|
| GPIO 0 (D0) | GPIO 17 (D7) |
| GPIO 1 (D1) | GPIO 19 (D9) |
| GPIO 2 (D2) | GPIO 20 (D8) |
| GPIO 4 (D3) | GPIO 18 (D10) |

*Connect diodes between the switch and the column line (Cathode/Black bar towards Column).*

### Rotary Encoder
| Encoder Pin | ESP32-C6 Pin |
|---|---|
| **A (Clk)** | GPIO 23 (D5) |
| **B (DT)** | GPIO 16 (D6) |
| **Switch** | GPIO 22 |
| **GND** | GND |

### Antenna
Connect the external antenna to the IPEX/U.FL connector on the XIAO ESP32C6.
---

## 💰 Price Breakdown
*Estimated costs.*

| Item | Qty |
|------|-----|
| XIAO Esp32C6 | 1 |
| Mechanical Switches | 16 |
| Rotary Encoder | 1 |
| External Antenna | 1 |
| Diodes 1N4148 | 16 |
| Inserts & Screws | Set |
| 3D Print Filament | ~120g |

**Total Cost per Macropad: ~15-18 €** (Removed battery/charger costs).

---

## ⚠️ Note about the ESP32-C6 board

Using the Seeed Studio XIAO ESP32C6 is a breeze. 
The documentation is excellent.
This project uses the USB-C port for both power and programming.

---

## 🧰 Build Instructions

### 🪛 1. Print and Prepare the Case
- Print all parts.
- Verify that switches and PCB fit snugly before final assembly.
- Insert **M2.5 heat-set inserts** into the designated mounting points using a soldering iron at ~200 °C.

### ⚡ 2. Mount the Components
- **Switches**: Press-fit into the 16-slot plate and solder in a grid matrix.
- **Rotary Encoder**: Mount in the designated hole, secure with nut.
- **Antenna**: Fix the antenna connector to the case hole and connect to the XIAO.
- **XIAO ESP32C6**: Clip in place. Connect USB-C for power.

### 🔋 3. Wiring Connection
Refer to the **Circuit Wiring** section above for detailed pinouts.
Ensure all diodes are correctly oriented (cathode to column).
Double check the Encoder wiring (A, B, Switch, GND).

### 🔧 4. Flash the Firmware
1. Install **ESP-IDF v5.3.4** (or newer).  
2. Clone or copy the project to your workspace.  
3. Build and flash:
   ```bash
   idf.py set-target esp32c6
   idf.py build
   idf.py flash monitor
   ```
   *Note: Ensure the external antenna is connected before powering up to avoid RF damage.*

4. **Pairing**: Long press the **Encoder Button** for >6 seconds to enter pairing mode.

---

## 🔗 Pair with Home Assistant

Add `macropad.mjs` to `config/zigbee2mqtt/external_converters` (create folder if needed).
Restart Zigbee2MQTT.
In Home Assistant, enable **Permit join**.
Long press the Encoder Button (>6s) to pair.

### Troubleshooting
If the device joins but exposes don't appear:
1. Check Zigbee2MQTT logs.
2. Verify the Cluster ID. The converter uses `manuSpecificAssaDoorLock` (0xFC00).
3. If your setup assigns a different name, update `macropad.mjs`.

---

## ✨ Features Summary

🔘 16 mechanical switches with multiple click detection
🎛️ Rotary Encoder with Push Button and rotation detection
📡 External Antenna for extended range
🔌 USB Powered (Battery-free)
💤 Smart Power Management (Light & Deep Sleep)
🔄 Encoder Button triggers factory reset (Long Press >6s)
🧱 Modular, 3D-printed enclosure

---

## 🧾 License

This project is released under the MIT License.
Remixed 3D models remain under their respective creator licenses (see linked Printables pages).

---
