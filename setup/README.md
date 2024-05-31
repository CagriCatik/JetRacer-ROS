# Instructions for Setting Up Jetson Nano with JetRacer

## Step 1: Program Jetson Nano Image

1. **Prepare an SD card** (minimum 64GB) and insert it into your computer using a card reader.
2. **Format the SD card** using SDFormatter. **Note:** Ensure other storage devices are disconnected to prevent accidental formatting.
3. **Download Win32DiskImager** to program the image to the SD card.
4. **Program the image** onto the SD card.
5. **Eject the SD card** after programming is complete.

## Step 2: Connect Jetson Nano to WiFi (with Display)

**Tip:** Connect peripherals such as monitors here for easy WiFi setup. If you lack a monitor, refer to Step 3.

1. Insert the SD card into the Jetson Nano (SD card slot on the back of the module).
2. Connect a mouse, keyboard, and monitor to the Jetson Nano board.
3. Turn on the power switch to start the Jetson Nano and log in with:
   - **Username:** jetson
   - **Password:** jetson
4. Click WiFi to connect. **Note:** Ensure the Jetson Nano and PC are on the same network for successful communication.
5. After connecting to WiFi, the OLED screen on the JetRacer will display the IP address.
6. Unplug the mouse, keyboard, and other peripherals for SSH remote login.

## Step 3: Connect Jetson Nano to WiFi (without Display)

1. Connect the network cable to the Jetson Nano. The JetRacer will display the network cable's IP address. **Note:** Ensure your PC and Jetson Nano are connected to the same network.
2. **Download MobaXterm** software for SSH remote login.
3. Open MobaXterm, click "Session -> SSH", enter the IP address displayed by the car in the host field, and press Enter to open the terminal. Login with:
   - **Username:** jetson
   - **Password:** jetson

4. Start connecting to WiFi:
   - Turn on WiFi:

     ```bash
     sudo nmcli r wifi on
     ```

   - Scan for WiFi:

     ```bash
     sudo nmcli dev wifi
     ```

   - Connect to WiFi:

     ```bash
     nmcli dev wifi connect wifi_name password wifi_password
     ```

   - A successful connection message will appear. Reboot and unplug the network cable. It will connect to the specified WiFi after booting.

## Step 4: SSH Remote Login

After connecting the Jetson Nano and PC to the same WiFi, the IP address should remain the same upon reboot.

1. The OLED screen on the JetRacer will display the IP address of the connected WiFi or the wired network port if WiFi is not connected.

2. **Download MobaXterm** for SSH Remote Login.
3. Open MobaXterm, click "Session -> SSH", enter the IP address displayed by the car, and press Enter. Login with:
   - **Username:** jetson
   - **Password:** jetson

### MobaXterm File Transmission

- To transfer files from Windows to Jetson Nano, drag the files to the left directory of MobaXterm.
- To transfer files from Jetson Nano to Windows, drag the files from the left directory of MobaXterm to Windows.

### NoMachine Login

NoMachine is a free remote desktop software that covers all major operating systems, including Windows, Mac, Linux, iOS, Android, and Raspberry Pi.

#### Install on Jetson Nano

1. Download and unzip NoMachine.
2. Copy the `.deb` file to the Jetson Nano using a USB drive or MobaXterm File Transmission.
3. Install NoMachine with:

   ```bash
   sudo dpkg -i nomachine_7.10.1_1_arm64.deb
   ```

#### Install on Windows Computer

1. Download and install NoMachine on your Windows computer. Restart the computer after installation.

#### Connect to Jetson Nano

1. Open NoMachine and enter the IP address of the Jetson Nano (e.g., 192.168.15.100). **Note:** Refer to the OLED display for the IP address.
2. Click Connect to the new host, enter the Jetson Nano's username and password (jetson), and log in.

## Step 5: Get Virtual IP and Hostname

1. Open the robot terminal and enter the following commands to check the IP and hostname of the Jetson Nano:

   ```bash
   ifconfig  # Get the IP address of the virtual machine
   hostname  # Get the hostname of the virtual machine
   ```

2. To check the WiFi name that Jetson Nano is connected to, and ensure your PC is connected to the same WiFi:

   ```bash
   iwconfig wlan0  # View WiFi name
   ```