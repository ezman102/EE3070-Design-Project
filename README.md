# SmartLocker Campus System

## Overview
SmartLocker Campus System is an innovative solution designed to enhance the security and convenience of locker systems in academic environments. This system integrates advanced technologies like RFID, keypad entry, and online management to provide a seamless user experience.

## Features
- **User Authentication**: Secure login through a unique combination of RFID, keypad entry, and fingerprint scanning.
- **Remote Monitoring**: Real-time locker status monitoring including humidity, temperature, and light conditions, accessible via a web interface.
- **Emergency Access**: Special provisions for emergency situations allowing authorized personnel to unlock lockers remotely.
- **QR Code Functionality**: Easy access to locker information and unlocking via QR codes.
- **Live Camera Feed**: Monitor locker areas in real-time for enhanced security.

## Installation

### Prerequisites
- Node.js and npm (Node Package Manager)
- MongoDB
- Arduino IDE for ESP8266/ESP32 module programming

### Setting Up the Server
1. Clone the repository to your local machine.
2. Navigate to the project directory and install dependencies:
   ```bash
   npm install
   ```
3. Start the MongoDB service on your system.
4. Run the server:
   ```bash
   npm start
   ```

### Configuring the Hardware
1. Setup ESP8266/ESP32 modules with Arduino IDE.
2. Load the provided Arduino scripts to the respective modules.
3. Connect the hardware components as per the schematic provided in the `hardware_schematics` directory.

## Usage
After setting up both the server and the hardware components:
1. Open a web browser and navigate to `http://localhost:3000`.
2. Log in using your credentials or register as a new user.
3. Access various functionalities like locker status, live camera feed, etc., from the dashboard.

## Contributing
We welcome contributions to the SmartLocker Campus System. Please read `CONTRIBUTING.md` for details on our code of conduct, and the process for submitting pull requests to us.

## License
This project is licensed under the MIT License - see the `LICENSE.md` file for details.

## Contact
For any queries or suggestions, feel free to contact us at `info@smartlockercampus.com`.

## Acknowledgments
- Special thanks to the contributors who have helped in building and refining this system.
- Thankful for the open-source community for the various tools and libraries used in this project.
