using System;
using System.Collections.Generic;
using System.Text;

namespace WiimoteLib
{
    public interface IFuser
    {
        Euler FusedValues {get;}
        void HandleIMUData(double yawDown, double pitchLeft, double rollLeft, double accX, double accY, double accZ);
    }
}
