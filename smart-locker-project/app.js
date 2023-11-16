const express = require('express');
const app = express();
const path = require('path');
const fs = require('fs');
const https = require('https');
const bcrypt = require('bcrypt');
const { MongoClient } = require('mongodb');

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
      db = client.db(dbName); // Set the db variable
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
  
  connectToDB();

  startServer();

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
    const userRole = req.session.userRole; // Adjust according to your implementation
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

  async function updateUserSettings(username, email, password, notifications) {
    try {
      // Validate and sanitize the inputs (implement according to your needs)
      if (!username || !email) throw new Error('Username and email are required.');

      let updateObject = {
        email: email,
        notifications: notifications
      };

      // Only hash the password if it's provided and needs to be changed
      if (password) {
        const saltRounds = 10;
        const hashedPassword = await bcrypt.hash(password, saltRounds);
        updateObject.password = hashedPassword;
      }

      // Update user settings in MongoDB
      await db.collection('users').updateOne({ username: username }, { $set: updateObject });

      console.log('User settings updated successfully.');
    } catch (error) {
      console.error('Error updating user settings:', error);
      throw error;
    }
  }

  app.post('/emergencyUnlock', async (req, res) => {
    const { lockerNumber, username, password } = req.body;

    // Authenticate the guard

    // Update the ThingSpeak channel
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


