const express = require('express');
const path = require('path');

const app = express();
const port = 3000;

app.set('view engine', 'ejs');
app.set('views', path.join(__dirname, 'views'));

app.use(express.static(path.join(__dirname, 'public')));
app.use(express.urlencoded({ extended: true }));

app.get('/home', (req, res) => {
    // Ensure the 'title' property is being passed
    res.render('home', { 
        title: 'Home',
        userType: 'YourUserType' // Make sure you replace 'YourUserType' with the actual user type.
    });
});



const users = {
  student: '11111111',
  professor: '22222222',
  security_guard: '33333333'
};

app.post('/login', (req, res) => {
    const userType = req.body.username;  // Assuming 'username' is one of 'student', 'professor', 'security_guard'
    const password = req.body.password;
  
    if (users[userType] && users[userType] === password) {
      // Include the title in the render method
      res.render('home', { 
        title: 'Home',  // Add a suitable title
        userType: userType 
      });
    } else {
      res.send('Invalid username or password');
    }
  });
  

app.listen(port, () => {
  console.log(`Server running at http://localhost:${port}/`);
});
