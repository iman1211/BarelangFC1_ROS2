require('Dynamixel');

twait = 0.010;

Dynamixel.open();
Dynamixel.read_data(200,32,12);
