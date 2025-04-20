Bathroom Emergency Detection System

A real-time **emergency alert system** designed to detect incidents and notify staff discreetly.

Team 21 – Damien Center Collaboration
Stephanie Close

Jenovia Mistry

Dimitris Kontos

Oliver Boettger

---

This system is composed of integrated embedded subsystems, each for the purpose of detection and response:

- **🚪 Door Sensor**  
  Monitors door activity to detect entry and occupancy status based on open/close events.

- **🎯 Motion Sensor**  
  Detects movement within the room to assess whether someone is present or inactive for an extended time.

- **📟 Pager Unit**  
  Wirelessly alerts staff ensuring discreet and immediate notification.

- **🧠 Main MCU Controller**  
  Coordinates communication between sensors and the pager system and manages system states.

## Emergency Logic Flow

1. **Normal State**: Motion + door sensor data monitored continuously.
2. **Potential Emergency Detected**:  
   - Door is closed  
   - No motion for threshold duration  
   - Triggers buzzer + LED for 15 seconds
3. **User Response Window**:  
   - If reset button is pressed → system resets  
   - If not → pager receives alert
4. **Manual Emergency**:  
   - Emergency button bypasses logic, sends alert instantly
5. **Pager Response**:  
   - Staff press acknowledge button  
   - Confirmation sent back via LoRa
---

### Build Instructions

1. Install [PlatformIO IDE](https://platformio.org/install)
2. Clone this repository:
   ```bash
   git clone https://github.com/your-username/Bathroom-Emergency-Detection.git
   cd Bathroom-Emergency-Detection
> To build and flash code, **navigate into the specific subsystem directory** (`doorSensor/`, `pager/`, or `main_mcu/`).  
> **PlatformIO does not work from the top-level repo directory!**

---

### Bare-Metal

- Full register-level control via STM32 CMSIS
- SPI (RFM9x), I2C (LCD, fuel gauge), GPIO (sensor/buttons)
- Precise timing and ultra-low-power sleep modes
- Software debouncing for buttons and door sensing
- Interrupt-driven motion detection + emergency logic
- UART logging for debugging + oscilloscope-tested timings
- Endorsed by Niraj

### HAL Abstraction Layer (Deprecated)
