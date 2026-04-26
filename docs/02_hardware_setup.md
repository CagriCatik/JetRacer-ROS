# Hardware Setup

This guide details the hardware setup for the Waveshare JetRacer ROS AI Kit.

## Platform Target

* **Robot**: Waveshare JetRacer ROS AI Kit
* **Controller**: NVIDIA Jetson Nano Developer Kit B01 (4GB)

> [!IMPORTANT]
> The Jetson Nano should be running **JetPack 4.4**. This is an Ubuntu 18.04 LTS (Bionic) environment, which explicitly targets **ROS 1 Melodic Morenia**. Do not flash JetPack 5+ unless you plan to do a manual, unsupported ROS migration.

## Assembly

Please follow the assembly instructions provided by Waveshare. The kit typically requires mounting the Jetson Nano onto the expansion board and connecting the motors, camera, and lidar (if applicable).

- [Waveshare Product Page](http://www.waveshare.com/JetRacer-ROS-AI-Kit.htm)

## Power Supply

- Ensure you use a high-quality power supply (typically 5V/4A via the barrel jack or as provided by the battery pack).
- The Jetson Nano can draw significant power during ML inference or motor startup, which may cause sudden reboots if underpowered.

## Peripherals

- **Camera**: Connect the CSI camera securely. Ensure the ribbon cable is seated correctly.
- **Wireless**: Use a compatible Wi-Fi module for the Jetson Nano to enable SSH and remote ROS communication.
- **Game Controller**: If using the included or a third-party gamepad, ensure it is paired and recognized by Ubuntu (`/dev/input/js0`).

> [!NOTE]
> TODO: Add specific I2C addresses or motor controller pinouts if deviating from the default Waveshare configuration.
