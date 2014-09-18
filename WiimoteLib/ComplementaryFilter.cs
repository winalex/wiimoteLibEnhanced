using System;
using System.Collections.Generic;
using System.Text;
using System.Diagnostics;

namespace WiimoteLib
{
   
  


public class ComplementaryFilter {
    // Gyro weight/smooting factor.

    
    // Marks first sample.
    bool firstSample;



    /// <summary>
    ///  Angles X,Y,Z in radians
    /// </summary>
    Point3F _Angles = new Point3F();

    Point3F _AnglesIntegrated = new Point3F();

    public float w1;
    public float w2;

    public Point3F Angles
    {
        get { return _Angles; }
       
    }




    public ComplementaryFilter(float weight1 = 0.98f, float weight2= 0.02f)
    {
        w1 = weight1;
        w2 = weight2;
       firstSample = true;
    }





    public void Update(double accX, double accY, double accZ, double gx, double gy, double gz, double dt)
    {
       

        // Integrate the gyroscope data -> int(angularSpeed) = angle
        //*pitch += ((float)gyrData[0] / GYROSCOPE_SENSITIVITY) * dt; // Angle around the X-axis
        //*roll -= ((float)gyrData[1] / GYROSCOPE_SENSITIVITY) * dt;    // Angle around the Y-axis

        double xSquared = accX * accX;
        double ySquared = accY * accY;
        double zSquared = accZ * accZ;


        double magnitude=Math.Sqrt(xSquared + ySquared + zSquared);
        double inv_len = 1 / magnitude;
        double x = accX * inv_len;
        double y = accY * inv_len;
        double z = accZ * inv_len;

        if (firstSample)
        {
            _AnglesIntegrated.Y = (float)Math.Atan2(x, z);
            _AnglesIntegrated.X = -(float)Math.Atan2(y, z);// -(float)Math.Asin(y);

            _Angles.X += (float)Math.Atan2(x, z);
            _Angles.Y += -(float)Math.Atan2(y, z); //-(float)Math.Asin(y);

           
            

            firstSample = false;
        }else{
             _AnglesIntegrated.X += (float)(gx * dt);
          //   _AnglesIntegrated.X /= (float)(2 * Math.PI);

             if (_AnglesIntegrated.X > Math.PI)
                 _AnglesIntegrated.X += -(float)(2 * Math.PI);
             else if (_AnglesIntegrated.X < -Math.PI)
                 _AnglesIntegrated.X += (float)(2 * Math.PI);

        
             _AnglesIntegrated.Y += (float)(gy * dt);


             if (_AnglesIntegrated.Y > Math.PI)
                 _AnglesIntegrated.Y += -(float)(2 * Math.PI);
             else if (_AnglesIntegrated.Y < -Math.PI)
                 _AnglesIntegrated.Y += (float)(2 * Math.PI);

             //_Angles.X += (float)(gx * dt);
             //_Angles.Y += (float)(gy * dt);


             //Roll
             _Angles.Y = _AnglesIntegrated.Y * w1 + w2 * (float)Math.Atan2(x, z);

             //Pitch
             _Angles.X = _AnglesIntegrated.X * w1 - w2 * (float)Math.Atan2(y, z); 
        }


        if (Double.IsNaN(_Angles.X))
            _Angles.X = 0f;
        if (Double.IsNaN(_Angles.Y))
            _Angles.Y = 0f;

       
        _Angles.Z += (float)(gz * dt);
        //_Angles.Z /= (float)(2 * Math.PI);
       

       // Debug.WriteLine("Roll gyro:" + (_AnglesIntegrated.Y * ComplementaryFilterFuser.RAD_TO_DEG) + " Roll acc:" + ((float)Math.Atan2(x, z)*ComplementaryFilterFuser.RAD_TO_DEG));
      //  Debug.WriteLine("Pitch gyro:" + (_AnglesIntegrated.X * ComplementaryFilterFuser.RAD_TO_DEG) + " Pitch acc:" + (-(float)Math.Asin(y) * ComplementaryFilterFuser.RAD_TO_DEG));
       // Debug.WriteLine("Pitch gyro:" + (_AnglesIntegrated.X * ComplementaryFilterFuser.RAD_TO_DEG) + " Pitch acc:" + (-(float)Math.Atan2(y, z) * ComplementaryFilterFuser.RAD_TO_DEG));
        
        // Debug.WriteLine("Pitch gyro:" + -_AnglesIntegrated.X   + " Pitch acc:" + (-(float)Math.Asin(y)));



        //if (magnitude < 0.15)
        //{ }
          
        

       
    }

    
}


    /// <summary>
    /// 
    /// </summary>
public class ComplementaryFilterFuser : IFuser
{
   
    private ComplementaryFilter complementary;
    private Euler Angles;
    public static double RAD_TO_DEG = 180 / Math.PI;
    public static double DEG_TO_RAD = Math.PI / 180;
  

    private Stopwatch watcher;
    long lastTime = -1;
  

    public ComplementaryFilterFuser()
    {
        // motionPlusPeriodCounter = new SamplePeriodCounter(1);
        complementary = new ComplementaryFilter();
        Angles = new Euler();
        watcher = new Stopwatch();
      
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
       
        if (!watcher.IsRunning)
        {
            watcher.Start();

        }
        else
        {
            if (lastTime > -1)
            {
              

                //  Angles.Yaw = kalman1.getAngle(Angles.Yaw, yawDown, 0.05);

                //  Debug.WriteLine(yawDown);
                //  Angles.Pitch

                if (yawDown < 0.5 && yawDown > -0.5)
                    yawDown = 0;

               
                if (pitchLeft < 0.5 && pitchLeft > -0.5)
                    pitchLeft = 0;

              

                if (rollLeft < 0.5 && rollLeft > -0.5)
                    rollLeft = 0;


                complementary.Update(accX, accY, accZ, pitchLeft * DEG_TO_RAD, rollLeft * DEG_TO_RAD, yawDown * DEG_TO_RAD, (watcher.ElapsedMilliseconds - lastTime) * 0.001);

                //Angles.Pitch = (float)Math.Atan2(-result[1], result[2]);// (Math.Atan2(result[0], result[2]) * RAD_TO_DEG);
               // Angles.Yaw = result[1] * RAD_TO_DEG;

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


  




    public Euler FusedValues
    {
        get
        {
            Angles.Yaw =  complementary.Angles.Z * RAD_TO_DEG;
            Angles.Pitch = complementary.Angles.X * RAD_TO_DEG;
            Angles.Roll = complementary.Angles.Y * RAD_TO_DEG;

            return Angles;
        }
    }


}

}
