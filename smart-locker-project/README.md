# SmartLocker Campus System

## Overview
The SmartLocker Campus System is an innovative solution designed to manage and secure locker systems within educational institutions. It provides a user-friendly interface for locker management, integrating features like a virtual keypad, live camera feeds, locker location mapping, and QR code functionality. The system emphasizes user convenience and security, making locker management efficient and secure.

## Features

### User Interface
- **Sidebar Navigation:** Easy-to-use sidebar with links to all main features.
- **Responsive Design:** Optimized for various devices and screen sizes.
- **Interactive Main Content Area:** Displays relevant pages and information dynamically.

### Frontend Features
- **Live Camera Feed:** Monitor locker surroundings in real-time.
- **Locker Location Mapping:** Google Maps integration to locate lockers.
- **QR Code Functionality:** Generate and scan QR codes for locker information and access.
- **Suspicious Activity Alerts:** Monitoring and notification of unusual locker access.

### Backend and Security
- **Virtual Keypad:** Interact with lockers using a virtual keypad.
- **Data Visualization:** Charts for displaying locker status like humidity, temperature, etc.
- **Emergency Locker Unlock:** Special access for emergencies with logging.
- **Email Integration:** Automated notifications for critical alerts.
- **User Authentication and Management:** Enhanced security for user data.

### Additional Tools and Technologies
- **Chart.js:** For dynamic data representation.
- **QRCode.js:** To generate and read QR codes.
- **Instascan.js:** For real-time camera scanning of QR codes.
- **Node.js and Express.js:** Backend server development.
- **MongoDB:** Database for storing user and locker data.
- **Nodemailer:** For sending automated emails.
- **Bcrypt:** For secure password hashing.

## Installation and Setup

1. **Clone the Repository:**
   ```
   git clone [repository-url]
   ```

2. **Navigate to the Project Directory:**
   ```
   cd smart-locker-campus-system
   ```

3. **Install Dependencies:**
   ```
   npm install
   ```

4. **Setup MongoDB:**
   - Ensure MongoDB is installed and running on your system.
   - Configure the connection string in the application settings.

5. **Start the Server:**
   ```
   npm start
   ```

6. **Access the Application:**
   - Open a web browser and navigate to `https://localhost:3000`.

## Usage

- **Login:** Use your credentials to access the system.
- **Locker Interaction:** Access the virtual keypad, view locker status, and use QR codes for locker operations.
- **Live Camera Feed:** Check the live feed for real-time surveillance.
- **User Settings:** Manage your account settings and password.

## Contributing

Contributions to the SmartLocker Campus System are welcome. Please follow these steps to contribute:

1. Fork the repository.
2. Create a new branch (`git checkout -b feature-branch`).
3. Make your changes and commit (`git commit -am 'Add some feature'`).
4. Push to the branch (`git push origin feature-branch`).
5. Create a new Pull Request.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

**Note:** Replace `[repository-url]` with the actual URL of the GitHub repository.