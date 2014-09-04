using System;
using System.Collections.Generic;
using System.Text;
using System.Diagnostics;

namespace WiimoteLib
{

    /// <summary>
    /// MahonyAHRS class. Madgwick's implementation of Mayhony's AHRS algorithm.
    /// </summary>
    /// <remarks>
    /// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
    /// </remarks>
    public class MahonyAHRS
    {
        /// <summary>
        /// Gets or sets the sample period.
        /// </summary>
        public float SamplePeriod { get; set; }

        /// <summary>
        /// Gets or sets the algorithm proportional gain.
        /// </summary>
        public float Kp { get; set; }

        /// <summary>
        /// Gets or sets the algorithm integral gain.
        /// </summary>
        public float Ki { get; set; }

        /// <summary>
        /// Gets or sets the Quaternion output.
        /// </summary>
        public float[] Quaternion { get; set; }

        /// <summary>
        /// Gets or sets the integral error.
        /// </summary>
        private float[] eInt { get; set; }

        /// <summary>
        /// Initializes a new instance of the <see cref="MadgwickAHRS"/> class.
        /// </summary>
        /// <param name="samplePeriod">
        /// Sample period.
        /// </param>
        public MahonyAHRS(float samplePeriod)
            : this(samplePeriod, 1f, 0f)
        {
        }

        /// <summary>
        /// Initializes a new instance of the <see cref="MadgwickAHRS"/> class.
        /// </summary>
        /// <param name="samplePeriod">
        /// Sample period.
        /// </param>
        /// <param name="kp">
        /// Algorithm proportional gain.
        /// </param> 
        public MahonyAHRS(float samplePeriod, float kp)
            : this(samplePeriod, kp, 0f)
        {
        }

        /// <summary>
        /// Initializes a new instance of the <see cref="MadgwickAHRS"/> class.
        /// </summary>
        /// <param name="samplePeriod">
        /// Sample period.
        /// </param>
        /// <param name="kp">
        /// Algorithm proportional gain.
        /// </param>
        /// <param name="ki">
        /// Algorithm integral gain.
        /// </param>
        public MahonyAHRS(float samplePeriod, float kp, float ki)
        {
            SamplePeriod = samplePeriod;
            Kp = kp;
            Ki = ki;
            Quaternion = new float[] { 1f, 0f, 0f, 0f };
            eInt = new float[] { 0f, 0f, 0f };
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
        /// </remarks> 
        public void Update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
        {
            float q1 = Quaternion[0], q2 = Quaternion[1], q3 = Quaternion[2], q4 = Quaternion[3];   // short name local variable for readability
            float norm;
            float hx, hy, bx, bz;
            float vx, vy, vz, wx, wy, wz;
            float ex, ey, ez;
            float pa, pb, pc;

            // Auxiliary variables to avoid repeated arithmetic
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
            hx = 2f * mx * (0.5f - q3q3 - q4q4) + 2f * my * (q2q3 - q1q4) + 2f * mz * (q2q4 + q1q3);
            hy = 2f * mx * (q2q3 + q1q4) + 2f * my * (0.5f - q2q2 - q4q4) + 2f * mz * (q3q4 - q1q2);
            bx = (float)Math.Sqrt((hx * hx) + (hy * hy));
            bz = 2f * mx * (q2q4 - q1q3) + 2f * my * (q3q4 + q1q2) + 2f * mz * (0.5f - q2q2 - q3q3);

            // Estimated direction of gravity and magnetic field
            vx = 2f * (q2q4 - q1q3);
            vy = 2f * (q1q2 + q3q4);
            vz = q1q1 - q2q2 - q3q3 + q4q4;
            wx = 2f * bx * (0.5f - q3q3 - q4q4) + 2f * bz * (q2q4 - q1q3);
            wy = 2f * bx * (q2q3 - q1q4) + 2f * bz * (q1q2 + q3q4);
            wz = 2f * bx * (q1q3 + q2q4) + 2f * bz * (0.5f - q2q2 - q3q3);

            // Error is cross product between estimated direction and measured direction of gravity
            ex = (ay * vz - az * vy) + (my * wz - mz * wy);
            ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
            ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
            if (Ki > 0f)
            {
                eInt[0] += ex;      // accumulate integral error
                eInt[1] += ey;
                eInt[2] += ez;
            }
            else
            {
                eInt[0] = 0.0f;     // prevent integral wind up
                eInt[1] = 0.0f;
                eInt[2] = 0.0f;
            }

            // Apply feedback terms
            gx = gx + Kp * ex + Ki * eInt[0];
            gy = gy + Kp * ey + Ki * eInt[1];
            gz = gz + Kp * ez + Ki * eInt[2];

            // Integrate rate of change of quaternion
            pa = q2;
            pb = q3;
            pc = q4;
            q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * SamplePeriod);
            q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * SamplePeriod);
            q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * SamplePeriod);
            q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * SamplePeriod);

            // Normalise quaternion
            norm = (float)Math.Sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
            norm = 1.0f / norm;
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
        public void Update(float gx, float gy, float gz, float ax, float ay, float az)
        {
            float q1 = Quaternion[0], q2 = Quaternion[1], q3 = Quaternion[2], q4 = Quaternion[3];   // short name local variable for readability
            float norm;
            float vx, vy, vz;
            float ex, ey, ez;
            float pa, pb, pc;

            // Normalise accelerometer measurement
            norm = (float)Math.Sqrt(ax * ax + ay * ay + az * az);
            if (norm == 0f) return; // handle NaN
            norm = 1 / norm;        // use reciprocal for division
            ax *= norm;
            ay *= norm;
            az *= norm;

            // Estimated direction of gravity
            vx = 2.0f * (q2 * q4 - q1 * q3);
            vy = 2.0f * (q1 * q2 + q3 * q4);
            vz = q1 * q1 - q2 * q2 - q3 * q3 + q4 * q4;

            // Error is cross product between estimated direction and measured direction of gravity
            ex = (ay * vz - az * vy);
            ey = (az * vx - ax * vz);
            ez = (ax * vy - ay * vx);
            if (Ki > 0f)
            {
                eInt[0] += ex;      // accumulate integral error
                eInt[1] += ey;
                eInt[2] += ez;
            }
            else
            {
                eInt[0] = 0.0f;     // prevent integral wind up
                eInt[1] = 0.0f;
                eInt[2] = 0.0f;
            }


            // Apply feedback terms
            gx = gx + Kp * ex + Ki * eInt[0];
            gy = gy + Kp * ey + Ki * eInt[1];
            gz = gz + Kp * ez + Ki * eInt[2];

            // Integrate rate of change of quaternion
            pa = q2;
            pb = q3;
            pc = q4;
            q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * SamplePeriod);
            q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * SamplePeriod);
            q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * SamplePeriod);
            q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * SamplePeriod);

            // Normalise quaternion
            norm = (float)Math.Sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
            norm = 1.0f / norm;
            Quaternion[0] = q1 * norm;
            Quaternion[1] = q2 * norm;
            Quaternion[2] = q3 * norm;
            Quaternion[3] = q4 * norm;
        }
    }



    public class Quaternion
    {
        public double Yaw { get; private set; }
        public double Roll { get; private set; }
        public double Pitch { get; private set; }

        public void Update(double w, double x, double y, double z, bool conjugate)
        {
            // normalize the vector
            double len = Math.Sqrt((w * w) + (x * x) + (y * y) + (z * z));
            w /= len;
            x /= len;
            y /= len;
            z /= len;

            // The Freespace quaternion gives the rotation in terms of
            // rotating the world around the object. We take the conjugate to
            // get the rotation in the object's reference frame.
            if (conjugate)
            {
                w = w;
                x = -x;
                y = -y;
                z = -z;
            }

            // Convert to angles in radians
            double m11 = (2.0f * w * w) + (2.0f * x * x) - 1.0f;
            double m12 = (2.0f * x * y) + (2.0f * w * z);
            double m13 = (2.0f * x * z) - (2.0f * w * y);
            double m23 = (2.0f * y * z) + (2.0f * w * x);
            double m33 = (2.0f * w * w) + (2.0f * z * z) - 1.0f;

            Roll = Math.Atan2(m23, m33);
            Pitch = Math.Asin(-m13);
            Yaw = Math.Atan2(m12, m11);

            if (Double.IsNaN(Roll))
                Roll = 0d;
            if (Double.IsNaN(Pitch))
                Pitch = 0d;
            if (Double.IsNaN(Yaw))
                Yaw = 0d;
        }

        public void Update(double w, double x, double y, double z)
        {
            Update(w, x, y, z, true);
        }
    }



    public class SamplePeriodCounter
    {
        private readonly Stopwatch stopwatch;
        private uint samples;
        
        protected uint _period;

        public SamplePeriodCounter(uint period)
        {
            _period = period;
            stopwatch = new Stopwatch();
        }

        public bool Update()
        {
            if (!stopwatch.IsRunning)
            {
                //stopwatch.Restart();
                //stopwatch.Reset();
                stopwatch.Start();
                //stopwatch.Re
                return false;
            }

            samples++;

            if (stopwatch.ElapsedMilliseconds > _period)
            {
                stopwatch.Stop();
                SamplePeriod = 1 / (float)(samples / stopwatch.Elapsed.TotalSeconds);
                return true;
            }

            return false;
        }

        public void Stop()
        {
            stopwatch.Stop();
        }

        public float SamplePeriod { get; private set; }
    }


    public class MahonyMotionPlusFuser:IFuser
    {
        private readonly SamplePeriodCounter motionPlusPeriodCounter;
        private MahonyAHRS mahonyAHRS;
        private Euler Angles;
        public static double RAD_TO_DEG = 180 / Math.PI;
        public static double DEG_TO_RAD = Math.PI / 180;
      


        public MahonyMotionPlusFuser()
        {
            motionPlusPeriodCounter = new SamplePeriodCounter(10);
            mahonyAHRS = new MahonyAHRS(0.01f);
            Angles = new Euler();
            
        }


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
            if (motionPlusPeriodCounter.Update())
            {
               // mahonyAHRS.Kp = 0;
              //  mahonyAHRS.SamplePeriod = motionPlusPeriodCounter.SamplePeriod;
              //  mahonyAHRS.Update((float)(rollLeft * DEG_TO_RAD), (float)(pitchLeft * DEG_TO_RAD), (float)(yawDown * DEG_TO_RAD), (float)accX, (float)accY, (float)accZ);

                mahonyAHRS.Update((float)(rollLeft * DEG_TO_RAD), (float)(pitchLeft * DEG_TO_RAD), (float)(yawDown * DEG_TO_RAD), (float)accY, -(float)accX, (float)accZ);
              
            }
            
       }

        public Euler FusedValues
        {
            get
            {
               

                var calculator = new Quaternion();
                calculator.Update(mahonyAHRS.Quaternion[0], mahonyAHRS.Quaternion[1], mahonyAHRS.Quaternion[2],
                                 mahonyAHRS.Quaternion[3]);

                Angles.Yaw = calculator.Yaw * RAD_TO_DEG;
               // Angles.Yaw = (Angles.Yaw * 0.35 + calculator.Yaw * 0.65) * RAD_TO_DEG;


                Angles.Roll = calculator.Roll * RAD_TO_DEG;
                Angles.Pitch = calculator.Pitch * RAD_TO_DEG;
                return Angles;
            }
        }

       
    }
}
