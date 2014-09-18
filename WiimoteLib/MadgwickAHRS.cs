using System;
using System.Collections.Generic;

using System.Text;
using System.Diagnostics;

namespace WiimoteLib
{
    /// <summary>
    /// MadgwickAHRS class. Implementation of Madgwick's IMU and AHRS algorithms.
    /// </summary>
    /// <remarks>
    /// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
    /// </remarks>
    public class MadgwickAHRS
    {
        /// <summary>
        /// Gets or sets the sample period.
        /// </summary>
        public float SamplePeriod { get; set; }

        /// <summary>
        /// Gets or sets the algorithm gain beta.
        /// </summary>
        public float Beta { get; set; }

        /// <summary>
        /// Gets or sets the Quaternion output.
        /// </summary>
        public float[] Quaternion { get; set; }

        /// <summary>
        /// Initializes a new instance of the <see cref="MadgwickAHRS"/> class.
        /// </summary>
        /// <param name="samplePeriod">
        /// Sample period.
        /// </param>
        public MadgwickAHRS(float samplePeriod)
            : this(samplePeriod, 1f)
        {
        }

        /// <summary>
        /// Initializes a new instance of the <see cref="MadgwickAHRS"/> class.
        /// </summary>
        /// <param name="samplePeriod">
        /// Sample period.
        /// </param>
        /// <param name="beta">
        /// Algorithm gain beta.
        /// </param>
        public MadgwickAHRS(float samplePeriod, float beta)
        {
            SamplePeriod = samplePeriod;
            Beta = beta;
            Quaternion = new float[] { 1f, 0f, 0f, 0f };
        }

        /// <summary>
        /// Algorithm AHRS update method. Requires only gyroscope and accelerometer data.
        /// </summary>
        /// <param name="gx">
        /// Gyroscope x axis measurement in radians/s.
        /// </param>
        /// <param name="gy">
        /// Gyroscope y axis measurement in radians/s.
        /// </param>
        /// <param name="gz">
        /// Gyroscope z axis measurement in radians/s.
        /// </param>
        /// <param name="ax">
        /// Accelerometer x axis measurement in any calibrated units.
        /// </param>
        /// <param name="ay">
        /// Accelerometer y axis measurement in any calibrated units.
        /// </param>
        /// <param name="az">
        /// Accelerometer z axis measurement in any calibrated units.
        /// </param>
        /// <param name="mx">
        /// Magnetometer x axis measurement in any calibrated units.
        /// </param>
        /// <param name="my">
        /// Magnetometer y axis measurement in any calibrated units.
        /// </param>
        /// <param name="mz">
        /// Magnetometer z axis measurement in any calibrated units.
        /// </param>
        /// <remarks>
        /// Optimised for minimal arithmetic.
        /// Total ±: 160
        /// Total *: 172
        /// Total /: 5
        /// Total sqrt: 5
        /// </remarks> 
        public void Update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
        {
            float q1 = Quaternion[0], q2 = Quaternion[1], q3 = Quaternion[2], q4 = Quaternion[3];   // short name local variable for readability
            float norm;
            float hx, hy, _2bx, _2bz;
            float s1, s2, s3, s4;
            float qDot1, qDot2, qDot3, qDot4;

            // Auxiliary variables to avoid repeated arithmetic
            float _2q1mx;
            float _2q1my;
            float _2q1mz;
            float _2q2mx;
            float _4bx;
            float _4bz;
            float _2q1 = 2f * q1;
            float _2q2 = 2f * q2;
            float _2q3 = 2f * q3;
            float _2q4 = 2f * q4;
            float _2q1q3 = 2f * q1 * q3;
            float _2q3q4 = 2f * q3 * q4;
            float q1q1 = q1 * q1;
            float q1q2 = q1 * q2;
            float q1q3 = q1 * q3;
            float q1q4 = q1 * q4;
            float q2q2 = q2 * q2;
            float q2q3 = q2 * q3;
            float q2q4 = q2 * q4;
            float q3q3 = q3 * q3;
            float q3q4 = q3 * q4;
            float q4q4 = q4 * q4;

            // Normalise accelerometer measurement
            norm = (float)Math.Sqrt(ax * ax + ay * ay + az * az);
            if (norm == 0f) return; // handle NaN
            norm = 1 / norm;        // use reciprocal for division
            ax *= norm;
            ay *= norm;
            az *= norm;

            // Normalise magnetometer measurement
            norm = (float)Math.Sqrt(mx * mx + my * my + mz * mz);
            if (norm == 0f) return; // handle NaN
            norm = 1 / norm;        // use reciprocal for division
            mx *= norm;
            my *= norm;
            mz *= norm;

            // Reference direction of Earth's magnetic field
            _2q1mx = 2f * q1 * mx;
            _2q1my = 2f * q1 * my;
            _2q1mz = 2f * q1 * mz;
            _2q2mx = 2f * q2 * mx;
            hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
            hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
            _2bx = (float)Math.Sqrt(hx * hx + hy * hy);
            _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
            _4bx = 2f * _2bx;
            _4bz = 2f * _2bz;

            // Gradient decent algorithm corrective step
            s1 = -_2q3 * (2f * q2q4 - _2q1q3 - ax) + _2q2 * (2f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            s2 = _2q4 * (2f * q2q4 - _2q1q3 - ax) + _2q1 * (2f * q1q2 + _2q3q4 - ay) - 4f * q2 * (1 - 2f * q2q2 - 2f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            s3 = -_2q1 * (2f * q2q4 - _2q1q3 - ax) + _2q4 * (2f * q1q2 + _2q3q4 - ay) - 4f * q3 * (1 - 2f * q2q2 - 2f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            s4 = _2q2 * (2f * q2q4 - _2q1q3 - ax) + _2q3 * (2f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            norm = 1f / (float)Math.Sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
            s1 *= norm;
            s2 *= norm;
            s3 *= norm;
            s4 *= norm;

            // Compute rate of change of quaternion
            qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - Beta * s1;
            qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - Beta * s2;
            qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - Beta * s3;
            qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - Beta * s4;

            // Integrate to yield quaternion
            q1 += qDot1 * SamplePeriod;
            q2 += qDot2 * SamplePeriod;
            q3 += qDot3 * SamplePeriod;
            q4 += qDot4 * SamplePeriod;
            norm = 1f / (float)Math.Sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
            Quaternion[0] = q1 * norm;
            Quaternion[1] = q2 * norm;
            Quaternion[2] = q3 * norm;
            Quaternion[3] = q4 * norm;
        }

        /// <summary>
        /// Algorithm IMU update method. Requires only gyroscope and accelerometer data.
        /// </summary>
        /// <param name="gx">
        /// Gyroscope x axis measurement in radians/s.
        /// </param>
        /// <param name="gy">
        /// Gyroscope y axis measurement in radians/s.
        /// </param>
        /// <param name="gz">
        /// Gyroscope z axis measurement in radians/s.
        /// </param>
        /// <param name="ax">
        /// Accelerometer x axis measurement in any calibrated units.
        /// </param>
        /// <param name="ay">
        /// Accelerometer y axis measurement in any calibrated units.
        /// </param>
        /// <param name="az">
        /// Accelerometer z axis measurement in any calibrated units.
        /// </param>
        /// <remarks>
        /// Optimised for minimal arithmetic.
        /// Total ±: 45
        /// Total *: 85
        /// Total /: 3
        /// Total sqrt: 3
        /// </remarks>
        public void Update(float gx, float gy, float gz, float ax, float ay, float az)
        {
            float q1 = Quaternion[0], q2 = Quaternion[1], q3 = Quaternion[2], q4 = Quaternion[3];   // short name local variable for readability
            float norm;
            float s1, s2, s3, s4;
            float qDot1, qDot2, qDot3, qDot4;

            // Auxiliary variables to avoid repeated arithmetic
            float _2q1 = 2f * q1;
            float _2q2 = 2f * q2;
            float _2q3 = 2f * q3;
            float _2q4 = 2f * q4;
            float _4q1 = 4f * q1;
            float _4q2 = 4f * q2;
            float _4q3 = 4f * q3;
            float _8q2 = 8f * q2;
            float _8q3 = 8f * q3;
            float q1q1 = q1 * q1;
            float q2q2 = q2 * q2;
            float q3q3 = q3 * q3;
            float q4q4 = q4 * q4;

            // Normalise accelerometer measurement
            norm = (float)Math.Sqrt(ax * ax + ay * ay + az * az);
            if (norm == 0f) return; // handle NaN
            norm = 1 / norm;        // use reciprocal for division
            ax *= norm;
            ay *= norm;
            az *= norm;

            // Gradient decent algorithm corrective step
            s1 = _4q1 * q3q3 + _2q3 * ax + _4q1 * q2q2 - _2q2 * ay;
            s2 = _4q2 * q4q4 - _2q4 * ax + 4f * q1q1 * q2 - _2q1 * ay - _4q2 + _8q2 * q2q2 + _8q2 * q3q3 + _4q2 * az;
            s3 = 4f * q1q1 * q3 + _2q1 * ax + _4q3 * q4q4 - _2q4 * ay - _4q3 + _8q3 * q2q2 + _8q3 * q3q3 + _4q3 * az;
            s4 = 4f * q2q2 * q4 - _2q2 * ax + 4f * q3q3 * q4 - _2q3 * ay;
            norm = 1f / (float)Math.Sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
            s1 *= norm;
            s2 *= norm;
            s3 *= norm;
            s4 *= norm;

            //// compute angular estimated direction of the gyroscope error
            //w_err_x = twoSEq_1 * SEqHatDot_2 - twoSEq_2 * SEqHatDot_1 - twoSEq_3 * SEqHatDot_4 + twoSEq_4 * SEqHatDot_3;
            //w_err_y = twoSEq_1 * SEqHatDot_3 + twoSEq_2 * SEqHatDot_4 - twoSEq_3 * SEqHatDot_1 - twoSEq_4 * SEqHatDot_2;
            //w_err_z = twoSEq_1 * SEqHatDot_4 - twoSEq_2 * SEqHatDot_3 + twoSEq_3 * SEqHatDot_2 - twoSEq_4 * SEqHatDot_1;

            //// compute and remove the gyroscope baises
            //w_bx += w_err_x * deltat * zeta;
            //w_by += w_err_y * deltat * zeta;
            //w_bz += w_err_z * deltat * zeta;
            //w_x -= w_bx;
            //w_y -= w_by;
            //w_z -= w_bz;



            // Compute rate of change of quaternion
            qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - Beta * s1;
            qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - Beta * s2;
            qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - Beta * s3;
            qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - Beta * s4;

            // Integrate to yield quaternion
            q1 += qDot1 * SamplePeriod;
            q2 += qDot2 * SamplePeriod;
            q3 += qDot3 * SamplePeriod;
            q4 += qDot4 * SamplePeriod;
            norm = 1f / (float)Math.Sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
            Quaternion[0] = q1 * norm;
            Quaternion[1] = q2 * norm;
            Quaternion[2] = q3 * norm;
            Quaternion[3] = q4 * norm;
        }
    }

    //double dt = (newTime - mTime) * 0.000001; // delta time in seconds

    //void IMUupdateMadgwick(double w_x, double w_y, double w_z, double a_x, double a_y, double a_z, double dt)
    //{
    //    // Grab the quaternion values
    //    float SEq_1 = mQuatMadgwick.W;
    //    float SEq_2 = mQuatMadgwick.X;
    //    float SEq_3 = mQuatMadgwick.Y;
    //    float SEq_4 = mQuatMadgwick.Z;

    //    // Local system variables
    //    float norm;																// vector norm
    //    float SEqDot_omega_1, SEqDot_omega_2, SEqDot_omega_3, SEqDot_omega_4;	// quaternion derrivative from gyroscopes elements
    //    float f_1, f_2, f_3;													// objective function elements
    //    float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33;				// objective function Jacobian elements
    //    float SEqHatDot_1, SEqHatDot_2, SEqHatDot_3, SEqHatDot_4;				// estimated direction of the gyroscope error

    //    // Axulirary variables to avoid reapeated calcualtions
    //    float halfSEq_1 = 0.5f * SEq_1;
    //    float halfSEq_2 = 0.5f * SEq_2;
    //    float halfSEq_3 = 0.5f * SEq_3;
    //    float halfSEq_4 = 0.5f * SEq_4;
    //    float twoSEq_1 = 2.0f * SEq_1;
    //    float twoSEq_2 = 2.0f * SEq_2;
    //    float twoSEq_3 = 2.0f * SEq_3;

    //    // Normalise the accelerometer measurement
    //    norm = sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
    //    a_x /= norm;
    //    a_y /= norm;
    //    a_z /= norm;

    //    // Compute the objective function and Jacobian
    //    f_1 = twoSEq_2 * SEq_4 - twoSEq_1 * SEq_3 - a_x;
    //    f_2 = twoSEq_1 * SEq_2 + twoSEq_3 * SEq_4 - a_y;
    //    f_3 = 1.0f - twoSEq_2 * SEq_2 - twoSEq_3 * SEq_3 - a_z;
    //    J_11or24 = twoSEq_3;
    //    J_12or23 = 2.0f * SEq_4;
    //    J_13or22 = twoSEq_1;
    //    J_14or21 = twoSEq_2;
    //    J_32 = 2.0f * J_14or21;
    //    J_33 = 2.0f * J_11or24;

    //    // Compute the gradient (matrix multiplication)
    //    SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1;
    //    SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3;
    //    SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1;
    //    SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2;

    //    // Normalise the gradient
    //    norm = sqrt(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
    //    SEqHatDot_1 /= norm;
    //    SEqHatDot_2 /= norm;
    //    SEqHatDot_3 /= norm;
    //    SEqHatDot_4 /= norm;

    //    // Compute the quaternion derrivative measured by gyroscopes
    //    SEqDot_omega_1 = -halfSEq_2 * w_x - halfSEq_3 * w_y - halfSEq_4 * w_z;
    //    SEqDot_omega_2 = halfSEq_1 * w_x + halfSEq_3 * w_z - halfSEq_4 * w_y;
    //    SEqDot_omega_3 = halfSEq_1 * w_y - halfSEq_2 * w_z + halfSEq_4 * w_x;
    //    SEqDot_omega_4 = halfSEq_1 * w_z + halfSEq_2 * w_y - halfSEq_3 * w_x;

    //    // Compute then integrate the estimated quaternion
    //    SEq_1 += (SEqDot_omega_1 - (beta * SEqHatDot_1)) * dt;
    //    SEq_2 += (SEqDot_omega_2 - (beta * SEqHatDot_2)) * dt;
    //    SEq_3 += (SEqDot_omega_3 - (beta * SEqHatDot_3)) * dt;
    //    SEq_4 += (SEqDot_omega_4 - (beta * SEqHatDot_4)) * dt;

    //    // Normalise quaternion
    //    norm = sqrt(SEq_1 * SEq_1 + SEq_2 * SEq_2 + SEq_3 * SEq_3 + SEq_4 * SEq_4);
    //    SEq_1 /= norm;
    //    SEq_2 /= norm;
    //    SEq_3 /= norm;
    //    SEq_4 /= norm;

    //    // Store the new orientation
    //    mQuatMadgwick.W = SEq_1;
    //    mQuatMadgwick.X = SEq_2;
    //    mQuatMadgwick.Y = SEq_3;
    //    mQuatMadgwick.Z = SEq_4;
    //}




    public class MagdwickFuser : IFuser
    {
        private readonly SamplePeriodCounter motionPlusPeriodCounter;
        private MadgwickAHRS madgwickAHRS;
        private Euler Angles;
        public static double RAD_TO_DEG = 180 / Math.PI;
        public static double DEG_TO_RAD = Math.PI / 180;
      // Constants for computing Madgwick
       // private const gyroMeasError 3.14159265358979f * (5.0f / 180.0f) // gyroscope measurement error in rad/s (shown as 5 deg/s)
        private const double gyroMeasError= 3.14159265358979f * (5.0f / 180.0f); // gyroscope measurement error in rad/s (shown as 5 deg/s)
        private double beta= Math.Sqrt(3.0f / 4.0f) * gyroMeasError; // compute beta
        private const double gyroMeasDrift= 3.14159265358979 * (0.2f / 180.0f) ;// gyroscope measurement error in rad/s/s (shown as 0.2f deg/s/s)
        private double zeta= Math.Sqrt(3.0f / 4.0f) * gyroMeasDrift; // compute zeta
      


        private Stopwatch watcher;
        long lastTime=-1;
        GyroFilter gyroFilter;
        private Kalman kalman1;


        public MagdwickFuser()
        {
           // motionPlusPeriodCounter = new SamplePeriodCounter(1);
            madgwickAHRS = new MadgwickAHRS(0.05f, (float)beta);
            Angles = new Euler();
            watcher = new Stopwatch();
            gyroFilter = new GyroFilter(200);
            kalman1 = new Kalman();
        }




//        #define ACCELEROMETER_SENSITIVITY 8192.0
//#define GYROSCOPE_SENSITIVITY 65.536
 
//#define M_PI 3.14159265359	    
 
//#define dt 0.01	
        
//        //
//        // 10 ms sample rate!    
 
//void madgwickAHRSFilter(short accData[3], short gyrData[3], float *pitch, float *roll)
//{
//    float pitchAcc, rollAcc;               
 
//    // Integrate the gyroscope data -> int(angularSpeed) = angle
//    *pitch += ((float)gyrData[0] / GYROSCOPE_SENSITIVITY) * dt; // Angle around the X-axis
//    *roll -= ((float)gyrData[1] / GYROSCOPE_SENSITIVITY) * dt;    // Angle around the Y-axis
 
//    // Compensate for drift with accelerometer data if !bullshit
//    // Sensitivity = -2 to 2 G at 16Bit -> 2G = 32768 && 0.5G = 8192
//    int forceMagnitudeApprox = abs(accData[0]) + abs(accData[1]) + abs(accData[2]);
//    if (forceMagnitudeApprox > 8192 && forceMagnitudeApprox < 32768)
//    {
//    // Turning around the X axis results in a vector on the Y-axis
//        pitchAcc = atan2f((float)accData[1], (float)accData[2]) * 180 / M_PI;
//        *pitch = *pitch * 0.98 + pitchAcc * 0.02;
 
//    // Turning around the Y axis results in a vector on the X-axis
//        rollAcc = atan2f((float)accData[0], (float)accData[2]) * 180 / M_PI;
//        *roll = *roll * 0.98 + rollAcc * 0.02;
//    }
//} 


        /// <summary>
        /// Handle gyro and accel data by applying calculation if sample period have passed
        /// </summary>
        /// <param name="yawDown"></param>
        /// <param name="pitchLeft"></param>
        /// <param name="rollLeft"></param>
        /// <param name="accX"></param>
        /// <param name="accY"></param>
        /// <param name="accZ"></param>
        public void HandleIMUData(double yawDown, double pitchLeft, double rollLeft, double accX, double accY, double accZ)
        {
            //if (motionPlusPeriodCounter.Update())
            //{

            //    //// Switch from RH Z-up Wiimote to LH Z-up UDK coordinate system
            //    //mPitchRate = -(new_state.MotionPlus.Speed.Pitch - mBias.x) * 3.14159265359 / 180.0;	// flip pitch
            //    //mYawRate = (new_state.MotionPlus.Speed.Yaw - mBias.y) * 3.14159265359 / 180.0;
            //    //mRollRate = -(new_state.MotionPlus.Speed.Roll - mBias.z) * 3.14159265359 / 180.0;		// flip roll

            //    //double wx = mRollRate;	// angular speed about x-axis
            //    //double wy = mPitchRate;	// angular speed about y-axis
            //    //double wz = mYawRate;	// angular speed about z-axis
            //    //double ax = -new_state.Acceleration.Y;	// switch and flip X and Y
            //    //double ay = -new_state.Acceleration.X;	// switch and flip X and Y
            //    //double az = new_state.Acceleration.Z;

            //    //if (mTime != 0)
            //    //{
            //    //    IMUupdateMadgwick(wx, wy, wz, ax, ay, az, dt);
            //    //}




            //   // Debug.WriteLine(rollLeft);
            //    madgwickAHRS.SamplePeriod = motionPlusPeriodCounter.SamplePeriod;
            //  //  madgwickAHRS.Update((float)( pitchLeft* DEG_TO_RAD), (float)(0 * DEG_TO_RAD), (float)(yawDown * DEG_TO_RAD), (float)accY, (float)accX, (float)accZ);
            //  //  madgwickAHRS.Update((float)(rollLeft * DEG_TO_RAD), 0f, 0f, (float)accX, (float)accY, (float)accZ);
            //  //  madgwickAHRS.Update(0f, 0f, 0f, (float)accY, (float)accX, (float)accZ);
            //   // madgwickAHRS.Update(0f, 0f, 0f, (float)accY, -(float)accX, (float)accZ);

            //   // madgwickAHRS.Update((float)(rollLeft * DEG_TO_RAD), (float)(pitchLeft * DEG_TO_RAD), (float)(yawDown * DEG_TO_RAD), -(float)accY, -(float)accX, (float)accZ);
             
            //}

            if (!watcher.IsRunning)
            {
                watcher.Start();
                
            }
            else
            {
                if (lastTime > -1)
                {
                    //faster samplePeriod slower reaching the goal angle
                    madgwickAHRS.SamplePeriod = 0.1F;// (watcher.ElapsedMilliseconds - lastTime) * 0.001f;

                   // Debug.Write(madgwickAHRS.SamplePeriod);
                   // gyroFilter.removeOffset(ref rollLeft, ref pitchLeft, ref yawDown);

                    //double P = Angles.Pitch;

                     double xSquared = accX * accX;
                    double ySquared = accY * accY;
                    double zSquared = accZ * accZ;



                    double inv_len = 1 / Math.Sqrt(xSquared + ySquared + zSquared);
                    double x = accX * inv_len;
                    double y = accY * inv_len;
                    double z = accZ * inv_len;


                  //  Angles.Yaw = kalman1.getAngle(Angles.Yaw, yawDown, 0.05);

                  //  Debug.WriteLine(yawDown);
                  //  Angles.Pitch

                    if (yawDown < 0.5 && yawDown > -0.5)
                        yawDown = 0;

                    Angles.Yaw += yawDown / 360 * 1.5;

                    if (pitchLeft < 0.5 && pitchLeft > -0.5)
                        pitchLeft = 0;

                    Angles.Pitch += pitchLeft / 360 * 1.45;

                    if (rollLeft < 0.5 && rollLeft > -0.5)
                        rollLeft = 0;

                    Angles.Roll += rollLeft / 360 * 1.5;

                    

                  //  Angles.Pitch += pitchLeft / 360 * 1.5;

                    //if (accY < 0.15 && accY > -0.15)
                    //    accY = 0;

                    //if (accX < 0.02 && accX > -0.02)
                    //    accX = 0;

                    //if (accZ < 0.15 && accZ > -0.15)
                    //    accZ = 0;

                 //   Angles.Yaw += yawDown/360 *1.5;


                   // madgwickAHRS.Update((float)(pitchLeft * DEG_TO_RAD), (float)(rollLeft * DEG_TO_RAD), (float)(yawDown * DEG_TO_RAD), -(float)accY, -(float)accX, (float)accZ);

                   // madgwickAHRS.Update((float)(0f * DEG_TO_RAD), (float)(rollLeft * DEG_TO_RAD), (float)(yawDown * DEG_TO_RAD), (float)accY, (float)accX , (float)accZ);


                   // madgwickAHRS.Update((float)(pitchLeft * DEG_TO_RAD), (float)(rollLeft * DEG_TO_RAD), (float)(yawDown * DEG_TO_RAD), (float)accY, - (float)accZ,(float)accX);

                    
                }


                lastTime = watcher.ElapsedMilliseconds;
           }

        }


        public static Point3F ToEulerAngles(float[] q)
        {
            // Store the Euler angles in radians
            Point3F pitchYawRoll = new Point3F();

            double sqw = q[0] * q[0];
            double sqx = q[1] * q[1];
            double sqy = q[2] * q[2];
            double sqz = q[3] * q[3];

            // If quaternion is normalised the unit is one, otherwise it is the correction factor
            double unit = sqx + sqy + sqz + sqw;
            double test = q[1] * q[2] + q[3] * q[0];

            if (test > 0.4999f * unit)                              // 0.4999f OR 0.5f - EPSILON
            {
                // Singularity at north pole
                pitchYawRoll.Y = 2f * (float)Math.Atan2(q[1], q[0]);  // Yaw
                pitchYawRoll.X = (float)Math.PI * 0.5f;                         // Pitch
                pitchYawRoll.Z = 0f;                                // Roll
                return pitchYawRoll;
            }
            else if (test < -0.4999f * unit)                        // -0.4999f OR -0.5f + EPSILON
            {
                // Singularity at south pole
                pitchYawRoll.Y = -2f * (float)Math.Atan2(q[1], q[0]); // Yaw
                pitchYawRoll.X = -(float)Math.PI * 0.5f;                        // Pitch
                pitchYawRoll.Z = 0f;                                // Roll
                return pitchYawRoll;
            }
            else
            {
                pitchYawRoll.Y = (float)Math.Atan2(2f * q[2] * q[0] - 2f * q[1] * q[3], sqx - sqy - sqz + sqw);       // Yaw
                pitchYawRoll.X = (float)Math.Asin(2f * test / unit);                                             // Pitch
                pitchYawRoll.Z = (float)Math.Atan2(2f * q[1] * q[0] - 2f * q[2] * q[3], -sqx + sqy - sqz + sqw);      // Roll
            }



            pitchYawRoll.X *= (float)RAD_TO_DEG;
            pitchYawRoll.Y *= (float)RAD_TO_DEG;
            pitchYawRoll.Z *= (float)RAD_TO_DEG;

            return pitchYawRoll;
        }


        //FOR UNITY
       // public static Quaternion ToQ(Vector3 v)
        //{
        //    return ToQ(v.y, v.x, v.z);
        //}

        //public static Quaternion ToQ(float yaw, float pitch, float roll)
        //{
        //    yaw *= Mathf.Deg2Rad;
        //    pitch *= Mathf.Deg2Rad;
        //    roll *= Mathf.Deg2Rad;
        //    float rollOver2 = roll * 0.5f;
        //    float sinRollOver2 = (float)Math.Sin((double)rollOver2);
        //    float cosRollOver2 = (float)Math.Cos((double)rollOver2);
        //    float pitchOver2 = pitch * 0.5f;
        //    float sinPitchOver2 = (float)Math.Sin((double)pitchOver2);
        //    float cosPitchOver2 = (float)Math.Cos((double)pitchOver2);
        //    float yawOver2 = yaw * 0.5f;
        //    float sinYawOver2 = (float)Math.Sin((double)yawOver2);
        //    float cosYawOver2 = (float)Math.Cos((double)yawOver2);
        //    Quaternion result;
        //    result.w = cosYawOver2 * cosPitchOver2 * cosRollOver2 + sinYawOver2 * sinPitchOver2 * sinRollOver2;
        //    result.x = cosYawOver2 * sinPitchOver2 * cosRollOver2 + sinYawOver2 * cosPitchOver2 * sinRollOver2;
        //    result.y = sinYawOver2 * cosPitchOver2 * cosRollOver2 - cosYawOver2 * sinPitchOver2 * sinRollOver2;
        //    result.z = cosYawOver2 * cosPitchOver2 * sinRollOver2 - sinYawOver2 * sinPitchOver2 * cosRollOver2;

        //    return result;
        //}

        //public static Vector3 FromQ2(Quaternion q1)
        //{
        //    float sqw = q1.w * q1.w;
        //    float sqx = q1.x * q1.x;
        //    float sqy = q1.y * q1.y;
        //    float sqz = q1.z * q1.z;
        //    float unit = sqx + sqy + sqz + sqw; // if normalised is one, otherwise is correction factor
        //    float test = q1.x * q1.w - q1.y * q1.z;
        //    Vector3 v;

        //    if (test > 0.4995f * unit)
        //    { // singularity at north pole
        //        v.y = 2f * Mathf.Atan2(q1.y, q1.x);
        //        v.x = Mathf.PI / 2;
        //        v.z = 0;
        //        return NormalizeAngles(v * Mathf.Rad2Deg);
        //    }
        //    if (test < -0.4995f * unit)
        //    { // singularity at south pole
        //        v.y = -2f * Mathf.Atan2(q1.y, q1.x);
        //        v.x = -Mathf.PI / 2;
        //        v.z = 0;
        //        return NormalizeAngles(v * Mathf.Rad2Deg);
        //    }
        //    Quaternion q = new Quaternion(q1.w, q1.z, q1.x, q1.y);
        //    v.y = (float)Math.Atan2(2f * q.x * q.w + 2f * q.y * q.z, 1 - 2f * (q.z * q.z + q.w * q.w));     // Yaw
        //    v.x = (float)Math.Asin(2f * (q.x * q.z - q.w * q.y));                             // Pitch
        //    v.z = (float)Math.Atan2(2f * q.x * q.y + 2f * q.z * q.w, 1 - 2f * (q.y * q.y + q.z * q.z));      // Roll
        //    return NormalizeAngles(v * Mathf.Rad2Deg);
        //}

        //static Vector3 NormalizeAngles(Vector3 angles)
        //{
        //    angles.x = NormalizeAngle(angles.x);
        //    angles.y = NormalizeAngle(angles.y);
        //    angles.z = NormalizeAngle(angles.z);
        //    return angles;
        //}

        //static float NormalizeAngle(float angle)
        //{
        //    while (angle > 360)
        //        angle -= 360;
        //    while (angle < 0)
        //        angle += 360;
        //    return angle;
        //}



        public Euler FusedValues
        {
            get
            {


                var calculator = new Quaternion();
                calculator.Update(madgwickAHRS.Quaternion[0], madgwickAHRS.Quaternion[1], madgwickAHRS.Quaternion[2],
                                 madgwickAHRS.Quaternion[3]);

                //(float)Math.Atan2(2f * q.X * q.W + 2f * q.Y * q.Z, 1 - 2f * (sqz + sqw));  

              // Angles.Yaw= MadgwickMotionPlusFuser.ToEulerAngles(madgwickAHRS.Quaternion).X;

               // Debug.WriteLine(MadgwickMotionPlusFuser.ToEulerAngles(madgwickAHRS.Quaternion).ToString());

                //Angles.Yaw = calculator.Yaw * RAD_TO_DEG;



                //Angles.Roll = calculator.Roll * RAD_TO_DEG;
                //Angles.Pitch = calculator.Pitch * RAD_TO_DEG;

                return Angles;
            }
        }


    }
}