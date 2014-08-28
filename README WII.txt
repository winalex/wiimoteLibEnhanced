//IDG 600
//FAST MODE
//+/- 2000 °/sec 0.5mV/deg   at ZERO = 1.5V

//SLOW MODE
//+/- 440 deg/s  2.27mV/deg
//+/- 550 deg/s  2mV/deg
mV/°/sec
at stationary Y=8196units
X=8220
Z=8233

8192 half range and 16.384 units in total 2^14  (14 bits) 2Bytes used

example
(N-512)/1024 * 3.3V * (1 deg/s)/.002V * (Pi radians)/(180 deg)
or
rate = (N-512)/1024 * (double) 28.783;


Analog Devices ADXL330 3 Axis Accelerometer
   //could go +/-3g
   //(N-512)/1024 * 3.0V * (9.8 m/s^2)/(0.300 V/g)