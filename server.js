const express = require('express');
const app = express();
const port = 3000; // Change the port number if needed

app.use(express.json());

// Handle POST requests from the web interface
app.post('/control', (req, res) => {
    const buttonPressed = req.body.button; // Get the button value from the request
    // Send the buttonPressed value to your Arduino using a suitable communication protocol (e.g., Serial communication)
    // You'll need to implement this part based on your Arduino code.
    // Example: serialPort.write(buttonPressed);
    res.status(200).send('Command sent to Arduino');
});

app.listen(port, () => {
    console.log(`Server is running on port ${port}`);
});
