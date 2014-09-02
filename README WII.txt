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
    at 3g there are 512units and 0.9V/g
	
	
	//const double fastModeFactor = 2000.0 / 440.0; //According to wiibrew
            const double fastModeFactor = 20.0 / 4.0; //According to wiic
            var gyro = calibration.NormalizeMotionplus(DateTime.Now, motionplus.yaw_down_speed,
                                                                     motionplus.pitch_left_speed,
                                                                     motionplus.roll_left_speed);

            return new CalibratedValue<Gyro>(gyro.DidCalibrate, new Gyro((motionplus.slow_modes & 0x1) == 0x1 ? gyro.Value.x : gyro.Value.x * fastModeFactor,
                                                                         (motionplus.slow_modes & 0x4) == 0x4 ? gyro.Value.y : gyro.Value.y * fastModeFactor,
                                                                         (motionplus.slow_modes & 0x2) == 0x2 ? gyro.Value.z : gyro.Value.z * fastModeFactor));
