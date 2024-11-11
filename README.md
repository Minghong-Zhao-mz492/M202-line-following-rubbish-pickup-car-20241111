# M202-line-following-rubbish-pickup-car-20241111

# Line-Following Rubbish-Pickup Car

This project is an Arduino-based robotic car designed to follow a line and pick up rubbish, placing it in designated areas marked by green and red zones.

## Features
- Line following using sensors
- Detection of rubbish
- Sorting and placing rubbish in designated zones

## Requirements
- **Arduino UNO**: The microcontroller board used to control the car and sensors.
- **Light Sensors**: Sensors for line-following functionality, detecting the path on the track.
- **Motors and Motor Driver**: Motors for movement and a motor driver to control them via the Arduino.
- **Chassis**: A car chassis that houses all components, including sensors, motors, and the Arduino.
- **Claws**: Mechanism for picking up rubbish.
- **Ultrasonic Sensor**: Used for detecting obstacles in the car's path.
- **Magnetic Hall Sensor**: Sensor to help identify and sort magnetic rubbish.
- **Additional Sensors**: Any other sensors required to enhance functionality.
- **Arduino IDE**: Software for writing, uploading, and debugging code on the Arduino.

## Installation and Setup
1. Connect the sensors and motors according to the wiring diagram (to be added).
2. Install necessary libraries in the Arduino IDE.
3. Upload the code to the Arduino.

## Code Structure
- `src/` - Contains the main Arduino code for the car.
- `include/` - Header files for various functionalities (some files are currently under development).
- `docs/` - Documentation and design notes.

## Current Status
This project is currently under development. Some features and files, like header files and sensor code, are not yet complete.

## Usage
1. Place the car on a track with lines and colored zones.
2. Power on the Arduino and observe the line-following and sorting behavior.

## Contributing
Contributions are welcome! Please open an issue to discuss any major changes.

## License
This project is currently not licensed.

## Contact
For questions, contact mz492@cam.ac.uk.
