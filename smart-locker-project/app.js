const express = require('express');
const app = express();
const path = require('path');
const fs = require('fs');
const https = require('https');
const bcrypt = require('bcrypt');
const { MongoClient } = require('mongodb');
var nodemailer = require('nodemailer');

const port = 3000;

  // MongoDB setup
  const mongoUrl = 'mongodb://127.0.0.1:27017';
  const client = new MongoClient(mongoUrl);
  const dbName = 'smartLockerDB';
  let db;

  app.set('view engine', 'ejs');
  app.set('views', path.join(__dirname, 'views'));

  app.use(express.static(path.join(__dirname, 'public')));
  app.use(express.urlencoded({ extended: true }));

  // Connect to MongoDB
  async function connectToDB() {
    try {
      await client.connect();
      console.log('Connected successfully to MongoDB server');
      db = client.db(dbName); 
    } catch (error) {
      console.error('Failed to connect to MongoDB:', error);
      process.exit(1);
    }
  }

  async function startServer() {
    await connectToDB(); // Make sure DB is connected before starting the server
  
    const httpsServer = https.createServer(credentials, app);
    httpsServer.listen(port, () => {
      console.log(`HTTPS Server running at https://localhost:${port}/`);
    });
  }
  
  startServer();

  app.use(express.json());

  app.get('/', (req, res) => {
    res.render('login');
  });

  const privateKey = fs.readFileSync('localhost.key', 'utf8');
  const certificate = fs.readFileSync('localhost.cert', 'utf8');
  
  const credentials = { key: privateKey, cert: certificate };

  app.post('/login', async (req, res) => {
    console.log(req.body); 
    const userInput = req.body.username; // Username or email
    const password = req.body.password;

    try {
      const user = await db.collection('users').findOne({
        $or: [{ username: userInput }, { email: userInput }]
      });

      if (!user) {
        return res.send('User not found');
      }

      // Using bcrypt to compare the password
      const isMatch = await bcrypt.compare(password, user.password);
      if (!isMatch) {
        return res.send('Invalid password');
      }

      res.render('home', { displayName: user.username, lockerNumber: user.lockerNumber, userRole: user.role, password: user.password });
    } catch (error) {
      console.error('Login error:', error);
      res.status(500).send('Internal Server Error');
    }
  });

  app.get('/home', (req, res) => {
    const userRole = req.session.userRole;
    res.render('home', { userRole: userRole });
  });

  const saltRounds = 10;

  async function createUser(username, email, rawPassword, role, lockerNumber) {
    try {
      const hashedPassword = await bcrypt.hash(rawPassword, saltRounds);

      const newUser = {
        username,
        email,
        password: hashedPassword,
        role,
        lockerNumber
      };

      await db.collection('users').insertOne(newUser);
      console.log('User created successfully');
    } catch (error) {
      console.error('Error creating user:', error);
      throw error;
    }
  }

  app.post('/emergencyUnlock', async (req, res) => {
    const { lockerNumber, username, password } = req.body;

    // Authenticate the guard

    const apiKey = "B66AQC1B5H7758EU";
    const fieldToUpdate = 5; // Adjust based on your setup
    const updateURL = `https://api.thingspeak.com/update?api_key=${apiKey}&field${fieldToUpdate}=1`;

    try {
        const response = await fetch(updateURL);
        const data = await response.text();
        if (data > 0) {
            res.send(`Locker ${lockerNumber} unlocked successfully.`);
        } else {
            throw new Error('Failed to unlock locker');
        }
    } catch (error) {
        console.error('Error:', error);
        res.status(500).send('Error unlocking locker');
    }
  });

  app.post('/sendEmail', async (req, res) => {
    const { lockerNumber, username } = req.body;

    try {
        // Fetch user's email from the database using the inputted username
        const user = await db.collection('users').findOne({ username: username });
        if (!user) {
            return res.status(404).send('User not found.');
        }

        const userEmail = user.email; // Extract the email from the user document

        var transporter = nodemailer.createTransport({
            service: 'gmail',
            auth: {
                user: 'smartlockercampus@gmail.com',
                pass: 'ovqfqqvtgsaukdvs'
            }
        });

        // Email HTML content
        const emailHtml = `
    <!DOCTYPE html>
    <html>
    <head>
        <style>
            body { font-family: Arial, sans-serif;}
            .container { width: 100%; max-width:auto; margin: auto; }
            .header { background-color: #004a7c; color: white; padding: 10px; text-align: center; font-size: 15px;}
            .content { padding: 20px; font-size: 13px; }
            .footer { background-color: #f2f2f2; padding: 10px; text-align: center; }
        </style>
    </head>
    <body>
        <div class="container">
            <div class="header">
                <h2>SmartLocker Campus System</h2>
            </div>
            <div class="content">
                <p>Dear ${username},</p>
                <p>An emergency unlock has been performed on <strong>Locker Number: ${lockerNumber}</strong> by Secuity Guard.</p>
                <p>If you have any questions or did not request this action, please contact our support team immediately.</p>
            </div>
            <div class="footer">
                <p>Contact us at support@smartlockercampus.com</p>
            </div>
        </div>
    </body>
    </html>
    `;

        var mailOptions = {
            from: 'smartlockercampus@gmail.com',
            to: userEmail, // Send to the user's email retrieved from the database
            subject: 'Emergency Locker Unlock Alert',
            html: emailHtml
        };

        transporter.sendMail(mailOptions, function(error, info){
            if (error) {
                console.log(error);
                return res.status(500).send('Error sending email.');
            } else {
                console.log('Email sent: ' + info.response);
                res.send('Email sent successfully.');
            }
        });

    } catch (error) {
        console.error('Error:', error);
        res.status(500).send('Error retrieving user information.');
    }
});



  app.post('/updatePassword', async (req, res) => {
    console.log(req.body); // Log the request body to debug
    const { userInput, newPassword } = req.body;
    if (!userInput || !newPassword) {
        return res.status(400).send('Username/email and password are required.');
    }

    try {
        // Find user by username or email
        const user = await db.collection('users').findOne({
            $or: [{ username: userInput }, { email: userInput }]
        });

        if (!user) {
            return res.status(404).send('User not found.');
        }

        // Hash the new password
        const saltRounds = 10;
        const hashedPassword = await bcrypt.hash(newPassword, saltRounds);

        // Update password in the database
        await db.collection('users').updateOne({ _id: user._id }, { $set: { password: hashedPassword } });

        res.send('Password updated successfully.');
    } catch (error) {
        console.error('Error updating password:', error);
        res.status(500).send('Internal Server Error');
    }
  });


