# Schematic

### 1. Power Supply and Management

- **Power Input (P1, P2, P3, P4):**

  - Headers for various power inputs (5V, 12V) and ground connections.
  - Voltage regulators such as U20 (RT9193-1.8PB) to provide 1.8V and 3.3V outputs.
- **Capacitors:**

  - Various capacitors (C90, C91, C92, etc.) for filtering and stabilizing the power supply.

### 2. Microcontroller Unit (MCU)

- **RP2040 (U19):**
  - The main microcontroller with GPIO pins for various I/O operations.
  - Connected to an external flash memory (U17 - W25Qxx) for additional storage.

### 3. Communication Interfaces

- **I2C Bus:**

  - Multiple I2C devices connected to SDA and SCL lines.
  - Includes a header (P5) for connecting external I2C devices.
- **SPI Bus:**

  - Used for high-speed communication with devices such as the MPU9250/ICM20948 (U21) sensor.
  - Flash memory (U17) also communicates over SPI.

### 4. Sensors and Peripherals

- **INA219 (INA1):**

  - Current sensor for monitoring power consumption.
  - Connected to I2C bus with address 0x41.
- **MPU9250/ICM20948 (U21):**

  - 9-axis IMU sensor for motion tracking, connected via I2C and SPI.
- **SSD1306 (U15):**

  - OLED display driver for a 128x32 pixel screen.
  - Connected via I2C for displaying information.

### 5. Motor Drivers

- **DRV8870 (DRV1, DRV2):**
  - Dual motor driver circuits for controlling the motors of the JetBot.
  - Controlled via GPIO pins for direction and speed control.
  - Includes current sensing for motor feedback.

### 6. Connectors and Headers

- **GPIO Headers (H1, H2, P6):**
  - Headers for accessing GPIO pins for additional peripherals and debugging.
  - Pins for SWD (Serial Wire Debug) for programming and debugging the MCU.

### 7. LEDs and Indicators

- **LED1:**
  - Status LED controlled via GPIO.

### 8. Miscellaneous Components

- **Resistors and Capacitors:**
  - Various resistors (R43, R44, R45, etc.) and capacitors (C78, C79, etc.) used for pull-up/down, filtering, and stabilization.
  - Resistors in series with LEDs for current limiting.
