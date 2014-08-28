//------------------------------------------------------------------------
//
// Author      : Jairo Rotava
// Email        :jairo.rotava@gmail.com
// Date        : 12/2011
// Version     : 1
// Description : Euler Kalmam Filter for Wiimote+motionplus position
//              The state variable (xhat) are roll, pitch and yaw.
//
//              Parameters needed are gyro data and external position reference 
//              (usually data derive from accel)
//------------------------------------------------------------------------
using System;
using System.Collections.Generic;
using System.Text;
using MatrixLibrary;

namespace KalmanFilter
{
    /// <summary>
    /// Euler Extended Kalman Filter (EKF) Library for Wiimote + MotionPlus
    /// </summary>
    public class EulerEKF
    {
        private Matrix K,A, z, H, Q, R, xhat, P, Pp, xp;
        private double Qvalue, Rvalue, Pvalue;

        /// <summary>
        /// Kalman filter parameter Q
        /// </summary>
        public double Qparam
        {
            get
            {
                return Qvalue;
            }
            set
            {
                Qvalue = value;
            }
        }


        /// <summary>
        /// Kalman filter parameter R
        /// </summary>
        public double Rparam
        {
            get
            {
                return Rvalue;
            }
            set
            {
                Rvalue = value;
            }
        }

        /// <summary>
        /// Kalman filter parameter P
        /// </summary>
        public double Pparam
        {
            get
            {
                return Pvalue;
            }
            set
            {
                Pvalue = value;
            }
        }

        /// <summary>
        /// Constructor.
        /// Q and R are how much you trust on data. High Q and low R trust more on reference data,
        /// low Q and high R trust more on gyros.
        /// P is initial covariance error. Should be big (~10).
        /// Q = 0.01, R=6, P = 10 are good starting values.
        /// </summary>
        public EulerEKF()
        {
            Qvalue = 0.01;
            Rvalue = 6;
            Pvalue = 10;
            Reset();
        }

        /// <summary>
        /// Reset filter
        /// </summary>
        public void Reset()
        {
            A = new Matrix(3, 3);
            xp = new Matrix(3, 1);
            xhat = new Matrix(3, 1);
            //z contem phi e theta
            z = new Matrix(3, 1);
            //H é a matriz que relaciona o estado com as medidas
            H = new Matrix(3, 3);
            H[0, 0] = 1; H[0, 1] = 0; H[0, 2] = 0;
            H[1, 0] = 0; H[1, 1] = 1; H[1, 2] = 0;
            H[2, 0] = 0; H[2, 1] = 0; H[2, 2] = 1;

            //Q é a matriz de covariancia do ruido wk - do sistema
            Q = new Matrix(3, 3);
            //double Qvalue = 1;
            Q[0, 0] = Qvalue; Q[0, 1] = 0; Q[0, 2] = 0;
            Q[1, 0] = 0; Q[1, 1] = Qvalue; Q[1, 2] = 0;
            Q[2, 0] = 0; Q[2, 1] = 0; Q[2, 2] = Qvalue;

            //R é a matriz de covariancia do ruido vk - da medidas
            R = new Matrix(3, 3);
            //Double Rvalue = 1;
            R[0, 0] = Rvalue; R[0, 1] = 0; R[0, 2] = 0;
            R[1, 0] = 0; R[1, 1] = Rvalue; R[1, 2] = 0;
            R[2, 0] = 0; R[2, 1] = 0; R[2, 2] = Rvalue;

            //x contem os valores inicias dos estados do sistema
            //No caso o roll,pitch e yaw são iguais a zero
            xhat = new Matrix(3, 1);
            xhat[0, 0] = 0; xhat[1, 0] = 0; xhat[2, 0] = 0;

            //P é a matriz de covariancia de erro da previsao.
            P = new Matrix(3, 3);
            //float Pvalue = 10;
            P[0, 0] = Pvalue; P[0, 1] = 0; P[0, 2] = 0;
            P[1, 0] = 0; P[1, 1] = Pvalue; P[1, 2] = 0;
            P[2, 0] = 0; P[2, 1] = 0; P[2, 2] = Pvalue;
        }

        
        /// <summary>
        /// Predict new state based on gyro inputs [rad/s] and last state.
        /// Calculate predictec covariance
        /// dt is elapsed time since lsat call in seconds.
        /// </summary>
        public void Predict(double p, double q, double r, double dt)
        {
            //Relacao:
            // p = phi = roll = gx
            // q = theta = pitch = gy
            // r = psi = yaw = gz
            //Referencia NED: North(x) East(y) Down(z)
            //Regra da mao direita para rotação positiva
            //Isso é um pouco diferente do hardwar do wii e mp tomar cuidado na hora de alimentar os dados na rotina

            //Coloca um nome mais facil
            double phi = xhat[0, 0];
            double theta = xhat[1, 0];

            //Coloca um nome mais bacana e converte para radianos
            //double p =  gy;
            //double q = gx;
            //double p = gx;
            //double q = gy;
            //double r = gz;

            //Calcula valores que vao ser utilizados mais a frente
            double cos_phi = Math.Cos(phi);
            double sin_phi = Math.Sin(phi);

            double cos_theta = Math.Cos(theta);
            double sec_theta = 1/Math.Cos(theta);
            double tan_theta = Math.Tan(theta);

            //	 [ T*q*cos(phi)*tan(theta) - T*r*sin(phi)*tan(theta) + 1,               T*r*cos(phi)*(tan(theta)^2 + 1) + T*q*sin(phi)*(tan(theta)^2 + 1), 0]
            //	 [                         - T*q*sin(phi) - T*r*cos(phi),                                                                               1, 0]
            //	 [ (T*q*cos(phi))/cos(theta) - (T*r*sin(phi))/cos(theta), (T*r*cos(phi)*sin(theta))/cos(theta)^2 + (T*q*sin(phi)*sin(theta))/cos(theta)^2, 1]


            //Jacobiana de A na forma discreta
            A[0, 0] = dt * (q * cos_phi * tan_theta - r * sin_phi * tan_theta) + 1;
            A[0, 1] = dt * (q * sin_phi * sec_theta * sec_theta + r * cos_phi * sec_theta * sec_theta);
            A[0, 2] = 0;
            A[1, 0] = dt * (-q * sin_phi - r * cos_phi);
            A[1, 1] = 1;
            A[1, 2] = 0;
            A[2, 0] = dt * (q * cos_phi * sec_theta - r * sin_phi * sec_theta);
            A[2, 1] = dt * (q * sin_phi * sec_theta * tan_theta + r * cos_phi * sec_theta * tan_theta);
            A[2, 2] = 1;


            /*
            phi_dot = p + r*cos(phi)*tan(theta) + q*sin(phi)*tan(theta)
            theta_dot = q*cos(phi) - r*sin(phi)
            psi_dot = (r*cos(phi))/cos(theta) + (q*sin(phi))/cos(theta)
            */

            //Calcula estado da predicao xp
            xp[0, 0] = dt * (p + q * sin_phi * tan_theta + r * cos_phi * tan_theta) + xhat[0, 0];
            xp[1, 0] = dt * (q * cos_phi - r * sin_phi) + xhat[1, 0];
            //xp[1, 0] = dt * (q * cos_phi + r * sin_phi) + xhat[1, 0];
            xp[2, 0] = dt * (q * sin_phi * sec_theta + r * cos_phi * sec_theta) + xhat[2, 0];
            //xp[2, 0] = dt * (- q * sin_phi * sec_theta + r * cos_phi * sec_theta) + xhat[2, 0];

            // Calcula Covariacia da predicao
            Pp = A * P * Matrix.Transpose(A) + Q;

            //Calcula Ganho Kalman
            K = Pp * Matrix.Transpose(H) * Matrix.Inverse(H * Pp * Matrix.Transpose(H) + R);
        }

        //Estimate states and error
        /// <summary>
        /// Estimate new state based on external reference and prediction.
        /// Calculate Estimate covariance
        /// </summary>
        public void Estimate(double phi, double theta, double psi)
        {
            //Z é uma matriz com phi e theta calculados a partir dos acelerometros
            z[0, 0] = phi;
            z[1, 0] = theta;
            z[2, 0] = psi;
            xhat = xp + K * (z - H * xp);
            P = Pp - K * H * Pp;
        }

        /// <summary>
        /// Get state variables roll, pitch and yaw for state matrix.
        /// </summary>
        public void GetState(out double roll, out double pitch, out double yaw)
        {
            roll = xhat[0,0];
            pitch = xhat[1,0];
            yaw = xhat[2,0];
        }
    }
}
