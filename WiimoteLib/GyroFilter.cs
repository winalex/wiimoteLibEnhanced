using System;
using System.Collections.Generic;

using System.Text;

namespace KalmanFilter
{
    /// <summary>
    /// Implementation of Gyros filter
    /// </summary>
    public class GyroFilter
    {
        int n_samples;
        int k;
        double offset_x, offset_y, offset_z;

        /// <summary>
        /// Constructor for Gyro Filter.
        /// n is number of sampless used to average offset value.
        /// </summary>
        public GyroFilter(int n)
        {
            n_samples = n;
            Reset();
        }

        /// <summary>
        /// Reset filter. The next n samples will be used to calculate offset value
        /// </summary>
        public void Reset()
        {
            k = 1;
            offset_x = 0;
            offset_y = 0;
            offset_z = 0;
        }

        /// <summary>
        /// Calculate mean offset of n samles (using recursive math) and remove if from signal.
        /// gx, gy, gz are gyro input to be filtered. 
        /// </summary>
        public void removeOffset(ref double gx, ref double gy, ref double gz)
        {
            if (k <= n_samples)
            {
                double alpha = (k - 1) / k;
                offset_x = alpha * offset_x + (1 - alpha) * gx;
                offset_y = alpha * offset_y + (1 - alpha) * gy;
                offset_z = alpha * offset_z + (1 - alpha) * gz;
                k++;
            }
            gx -= offset_x;
            gy -= offset_y;
            gz -= offset_z;
        }
    }

}
