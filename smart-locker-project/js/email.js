var nodemailer = require('nodemailer');

var transporter = nodemailer.createTransport({
  service: 'gmail',
  auth: {
    user: 'smartlockercampus@gmail.com',
    pass: 'ovqfqqvtgsaukdvs'
  }
});

var mailOptions = {
  from: 'smartlockercampus@gmail.com',
  to: 'sinyukyuen@gmail.com',
  subject: 'Emergency Locker Unlock Alert', // Subject line
  text: `An emergency unlock has been performed on Locker Number: ${lockerNumber} by ${username}.`, // plain text body
};

transporter.sendMail(mailOptions, function(error, info){
  if (error) {
    console.log(error);
  } else {
    console.log('Email sent: ' + info.response);
  }
});