//////////////////////////////////////////////////////////////////////////////////
//	DataTypes.cs
//	Managed Wiimote Library
//	Written by Brian Peek (http://www.brianpeek.com/)
//	for MSDN's Coding4Fun (http://msdn.microsoft.com/coding4fun/)
//	Visit http://blogs.msdn.com/coding4fun/archive/2007/03/14/1879033.aspx
//  and http://www.codeplex.com/WiimoteLib
//	for more information
//////////////////////////////////////////////////////////////////////////////////

using System;

// if we're building the MSRS version, we need to bring in the MSRS Attributes
// if we're not doing the MSRS build then define some fake attribute classes for DataMember/DataContract
#if MSRS
	using Microsoft.Dss.Core.Attributes;
#else
	sealed class DataContract : Attribute
	{
	}

	sealed class DataMember: Attribute
	{
	}
#endif

namespace WiimoteLib
{
#if MSRS
    [DataContract]
    public struct RumbleRequest
    {
        [DataMember]
        public bool Rumble;
    }
#endif

	/// <summary>
	/// Point structure for floating point 2D positions (X, Y)
	/// </summary>
	[Serializable]
	[DataContract]
	public struct PointF
	{
		/// <summary>
		/// X, Y coordinates of this point
		/// </summary>
		[DataMember]
		public float X, Y;

		/// <summary>
		/// Convert to human-readable string
		/// </summary>
		/// <returns>A string that represents the point</returns>
		public override string ToString()
		{
			return string.Format("{{X={0}, Y={1}}}", X, Y);
		}
		
	}

	/// <summary>
	/// Point structure for int 2D positions (X, Y)
	/// </summary>
	[Serializable]	
	[DataContract]
	public struct Point
	{
		/// <summary>
		/// X, Y coordinates of this point
		/// </summary>
		[DataMember]
		public int X, Y;

		/// <summary>
		/// Convert to human-readable string
		/// </summary>
		/// <returns>A string that represents the point.</returns>
		public override string ToString()
		{
			return string.Format("{{X={0}, Y={1}}}", X, Y);
		}
	}

	/// <summary>
	/// Point structure for floating point 3D positions (X, Y, Z)
	/// </summary>
	[Serializable]	
	[DataContract]
	public struct Point3F
	{
		/// <summary>
		/// X, Y, Z coordinates of this point
		/// </summary>
		[DataMember]
		public float X, Y, Z;


        public Point3F(float x = 0f, float y = 0f, float z = 0f)
        {
            this.X = x;
            this.Y = y;
            this.Z = z;
        }

		/// <summary>
		/// Convert to human-readable string
		/// </summary>
		/// <returns>A string that represents the point</returns>
		public override string ToString()
		{
			return string.Format("{{X={0:0.00}, Y={1:0.00}, Z={2:0.00}}}", X, Y, Z);
		}
		
	}

	/// <summary>
	/// Point structure for int 3D positions (X, Y, Z)
	/// </summary>
	[Serializable]
	[DataContract]
	public struct Point3
	{
		/// <summary>
		/// X, Y, Z coordinates of this point
		/// </summary>
		[DataMember]
		public int X, Y, Z;

		/// <summary>
		/// Convert to human-readable string
		/// </summary>
		/// <returns>A string that represents the point.</returns>
		public override string ToString()
		{
			return string.Format("{{X={0}, Y={1}, Z={2}}}", X, Y, Z);
		}
	}



    /// <summary>
    /// Euler structure for floating point 3D positions (roll, pitch, yaw)
    /// </summary>
    [Serializable]
    [DataContract]
    public struct Euler
    {
        /// <summary>
        /// roll, pitch, yaw coordinates of this point
        /// </summary>
        [DataMember]
        public double Roll, Pitch, Yaw;

        public Euler(double roll = 0f, double pitch = 0f, double yaw = 0)
        {
            this.Roll = roll;
            this.Pitch = pitch;
            this.Yaw = yaw;

        }

        /// <summary>
        /// Convert to human-readable string
        /// </summary>
        /// <returns>A string that represents the point</returns>
        public override string ToString()
        {
            return string.Format("{{Pitch={0:0.00}, Roll={1:0.00}, Yaw={2:0.00}}}", Pitch, Roll, Yaw);
        }

    }



	/// <summary>
	/// Current overall state of the Wiimote and all attachments
	/// </summary>
	[Serializable]
	[DataContract]
	public class WiimoteState
	{
        /// <summary>
        /// Current Position - Euler Angles
        /// </summary>
        [DataMember]
        public Euler Position;

		/// <summary>
		/// Current calibration information
		/// </summary>
		[DataMember]
		public AccelCalibrationInfo AccelCalibrationInfo;
		/// <summary>
		/// Current state of accelerometers
		/// </summary>
		[DataMember]
		public AccelState AccelState;
		/// <summary>
		/// Current state of buttons
		/// </summary>
		[DataMember]
		public ButtonState ButtonState;
		/// <summary>
		/// Current state of IR sensors
		/// </summary>
		[DataMember]
		public IRState IRState;
		/// <summary>
		/// Raw byte value of current battery level
		/// </summary>
		[DataMember]
		public byte BatteryRaw;
		/// <summary>
		/// Calculated current battery level
		/// </summary>
		[DataMember]
		public float Battery;
		/// <summary>
		/// Current state of rumble
		/// </summary>
		[DataMember]
		public bool Rumble;
		/// <summary>
		/// Is an extension controller inserted?
		/// </summary>
		[DataMember]
		public byte Extension;
		
		/// <summary>
		/// Current state of Nunchuk extension
		/// </summary>
		[DataMember]
		public NunchukState NunchukState;
		/// <summary>
		/// Current state of Classic Controller extension
		/// </summary>
		[DataMember]
		public ClassicControllerState ClassicControllerState;
		/// <summary>
		/// Current state of Guitar extension
		/// </summary>
		[DataMember]
		public GuitarState GuitarState;
		/// <summary>
		/// Current state of Drums extension
		/// </summary>
		[DataMember]
		public DrumsState DrumsState;
		/// <summary>
		/// Current state of the Wii Fit Balance Board
		/// </summary>
		public BalanceBoardState BalanceBoardState;
		/// <summary>
		/// Current state of the Taiko TaTaCon drum controller
		/// </summary>
		public TaikoDrumState TaikoDrumState;
		/// <summary>
		/// Current state of the MotionPlus controller
		/// </summary>
		public MotionPlusState MotionPlusState;
		/// <summary>
		/// Current state of LEDs
		/// </summary>
		[DataMember]
		public LEDState LEDState;
		/// <summary>
		/// Constructor for WiimoteState class
		/// </summary>
		public WiimoteState()
		{
			IRState.IRSensors = new IRSensor[4];
		}
	}

	/// <summary>
	/// Current state of LEDs
	/// </summary>
	[Serializable]
    [DataContract]
    public struct LEDState
    {
		/// <summary>
		/// LED on the Wiimote
		/// </summary>
        [DataMember]
        public bool LED1, LED2, LED3, LED4;
    }

	/// <summary>
	/// Calibration information stored on the Nunchuk
	/// </summary>
	[Serializable]
	[DataContract]
	public struct NunchukCalibrationInfo
	{
		/// <summary>
		/// Accelerometer calibration data
		/// </summary>
		public AccelCalibrationInfo AccelCalibration;
		/// <summary>
		/// Joystick X-axis calibration
		/// </summary>
		[DataMember]
		public byte MinX, MidX, MaxX;
		/// <summary>
		/// Joystick Y-axis calibration
		/// </summary>
		[DataMember]
		public byte MinY, MidY, MaxY;
	}

	/// <summary>
	/// Calibration information stored on the Classic Controller
	/// </summary>
	[Serializable]
	[DataContract]	
	public struct ClassicControllerCalibrationInfo
	{
		/// <summary>
		/// Left joystick X-axis 
		/// </summary>
		[DataMember]
		public byte MinXL, MidXL, MaxXL;
		/// <summary>
		/// Left joystick Y-axis
		/// </summary>
		[DataMember]
		public byte MinYL, MidYL, MaxYL;
		/// <summary>
		/// Right joystick X-axis
		/// </summary>
		[DataMember]
		public byte MinXR, MidXR, MaxXR;
		/// <summary>
		/// Right joystick Y-axis
		/// </summary>
		[DataMember]
		public byte MinYR, MidYR, MaxYR;
		/// <summary>
		/// Left analog trigger
		/// </summary>
		[DataMember]
		public byte MinTriggerL, MaxTriggerL;
		/// <summary>
		/// Right analog trigger
		/// </summary>
		[DataMember]
		public byte MinTriggerR, MaxTriggerR;
	}

	/// <summary>
	/// Current state of the Nunchuk extension
	/// </summary>
	[Serializable]
	[DataContract]	
	public struct NunchukState
	{
		/// <summary>
		/// Calibration data for Nunchuk extension
		/// </summary>
		[DataMember]
		public NunchukCalibrationInfo CalibrationInfo;
		/// <summary>
		/// State of accelerometers
		/// </summary>
		[DataMember]
		public AccelState AccelState;
		/// <summary>
		/// Raw joystick position before normalization.  Values range between 0 and 255.
		/// </summary>
		[DataMember]
		public Point RawJoystick;
		/// <summary>
		/// Normalized joystick position.  Values range between -0.5 and 0.5
		/// </summary>
		[DataMember]
		public PointF Joystick;
		/// <summary>
		/// Digital button on Nunchuk extension
		/// </summary>
		[DataMember]
		public bool C, Z;
	}

	/// <summary>
	/// Curernt button state of the Classic Controller
	/// </summary>
	[Serializable]
	[DataContract]
	public struct ClassicControllerButtonState
	{
		/// <summary>
		/// Digital button on the Classic Controller extension
		/// </summary>
		[DataMember]
		public bool A, B, Plus, Home, Minus, Up, Down, Left, Right, X, Y, ZL, ZR;
		/// <summary>
		/// Analog trigger - false if released, true for any pressure applied
		/// </summary>
		[DataMember]
		public bool TriggerL, TriggerR;
	}

	/// <summary>
	/// Current state of the Classic Controller
	/// </summary>
	[Serializable]
	[DataContract]
	public struct ClassicControllerState
	{
		/// <summary>
		/// Calibration data for Classic Controller extension
		/// </summary>
		[DataMember]
		public ClassicControllerCalibrationInfo CalibrationInfo;
		/// <summary>
		/// Current button state
		/// </summary>
		[DataMember]
		public ClassicControllerButtonState ButtonState;
		/// <summary>
		/// Raw value of left joystick.  Values range between 0 - 255.
		/// </summary>
		[DataMember]
		public Point RawJoystickL;
		/// <summary>
		/// Raw value of right joystick.  Values range between 0 - 255.
		/// </summary>
		[DataMember]
		public Point RawJoystickR;
		/// <summary>
		/// Normalized value of left joystick.  Values range between -0.5 - 0.5
		/// </summary>
		[DataMember]
		public PointF JoystickL;
		/// <summary>
		/// Normalized value of right joystick.  Values range between -0.5 - 0.5
		/// </summary>
		[DataMember]
		public PointF JoystickR;
		/// <summary>
		/// Raw value of analog trigger.  Values range between 0 - 255.
		/// </summary>
		[DataMember]
		public byte RawTriggerL, RawTriggerR;
		/// <summary>
		/// Normalized value of analog trigger.  Values range between 0.0 - 1.0.
		/// </summary>
		[DataMember]
		public float TriggerL, TriggerR;
	}

	/// <summary>
	/// Current state of the Guitar controller
	/// </summary>
	[Serializable]
	[DataContract]
	public struct GuitarState
	{
		/// <summary>
		/// Guitar type
		/// </summary>
		[DataMember]
		public GuitarType GuitarType;
		/// <summary>
		/// Current button state of the Guitar
		/// </summary>
		[DataMember]
		public GuitarButtonState ButtonState;
		/// <summary>
		/// Current fret button state of the Guitar
		/// </summary>
		[DataMember]
		public GuitarFretButtonState FretButtonState;
		/// <summary>
		/// Current touchbar state of the Guitar
		/// </summary>
		[DataMember]
		public GuitarFretButtonState TouchbarState;
		/// <summary>
		/// Raw joystick position.  Values range between 0 - 63.
		/// </summary>
		[DataMember]
		public Point RawJoystick;
		/// <summary>
		/// Normalized value of joystick position.  Values range between 0.0 - 1.0.
		/// </summary>
		[DataMember]
		public PointF Joystick;
		/// <summary>
		/// Raw whammy bar position.  Values range between 0 - 10.
		/// </summary>
		[DataMember]
		public byte RawWhammyBar;
		/// <summary>
		/// Normalized value of whammy bar position.  Values range between 0.0 - 1.0.
		/// </summary>
		[DataMember]
		public float WhammyBar;
	}

	/// <summary>
	/// Current fret button state of the Guitar controller
	/// </summary>
	[Serializable]
	[DataContract]
	public struct GuitarFretButtonState
	{
		/// <summary>
		/// Fret buttons
		/// </summary>
		[DataMember]
		public bool Green, Red, Yellow, Blue, Orange;
	}

	
	/// <summary>
	/// Current button state of the Guitar controller
	/// </summary>
	[Serializable]
	[DataContract]
	public struct GuitarButtonState
	{
		/// <summary>
		/// Strum bar
		/// </summary>
		[DataMember]
		public bool StrumUp, StrumDown;
		/// <summary>
		/// Other buttons
		/// </summary>
		[DataMember]
		public bool Minus, Plus;
	}

	/// <summary>
	/// Current state of the Drums controller
	/// </summary>
	[Serializable]
	[DataContract]
	public struct DrumsState
	{
		/// <summary>
		/// Drum pads
		/// </summary>
		public bool Red, Green, Blue, Orange, Yellow, Pedal;
		/// <summary>
		/// Speed at which the pad is hit.  Values range from 0 (very hard) to 6 (very soft)
		/// </summary>
		public int RedVelocity, GreenVelocity, BlueVelocity, OrangeVelocity, YellowVelocity, PedalVelocity;
		/// <summary>
		/// Other buttons
		/// </summary>
		public bool Plus, Minus;
		/// <summary>
		/// Raw value of analong joystick.  Values range from 0 - 15
		/// </summary>
		public Point RawJoystick;
		/// <summary>
		/// Normalized value of analog joystick.  Values range from 0.0 - 1.0
		/// </summary>
		public PointF Joystick;
	}

	/// <summary>
	/// Current state of the Wii Fit Balance Board controller
	/// </summary>
	[Serializable]
	[DataContract]
	public struct BalanceBoardState
	{
		/// <summary>
		/// Calibration information for the Balance Board
		/// </summary>
		[DataMember]
		public BalanceBoardCalibrationInfo CalibrationInfo;
		/// <summary>
		/// Raw values of each sensor
		/// </summary>
		[DataMember]
		public BalanceBoardSensors SensorValuesRaw;
		/// <summary>
		/// Kilograms per sensor
		/// </summary>
		[DataMember]
		public BalanceBoardSensorsF SensorValuesKg;
		/// <summary>
		/// Pounds per sensor
		/// </summary>
		[DataMember]
		public BalanceBoardSensorsF SensorValuesLb;
		/// <summary>
		/// Total kilograms on the Balance Board
		/// </summary>
		[DataMember]
		public float WeightKg;
		/// <summary>
		/// Total pounds on the Balance Board
		/// </summary>
		[DataMember]
		public float WeightLb;
		/// <summary>
		/// Center of gravity of Balance Board user
		/// </summary>
		[DataMember]
		public PointF CenterOfGravity;
	}

	/// <summary>
	/// Current state of the Taiko Drum (TaTaCon) controller
	/// </summary>
	[Serializable]
	[DataContract]
	public struct TaikoDrumState
	{
		/// <summary>
		/// Drum hit location
		/// </summary>
		[DataMember]
		public bool InnerLeft, InnerRight, OuterLeft, OuterRight;
	}


    /// <summary>
    /// Calibration information stored on the Montion Plus
    /// </summary>
    [Serializable]
    [DataContract]
    public struct MontionPlusCalibrationInfo
    {
        /// <summary>
        /// Gyros calibration data
        /// </summary>
        public GyroCalibrationInfo GyroCalibration;
        /// <summary>
        /// Gyros calibration data
        /// </summary>
        public GyroCalibrationInfo GyroFastCalibration;


    }


    /// <summary>
    /// Gyros calibration information
    /// </summary>
    [Serializable]
    [DataContract]
    public struct GyroCalibrationInfo
    {
        /// <summary>
        /// Offset of gyros
        /// </summary>
        [DataMember]
        public int X0, Y0, Z0;
        /// <summary>
        /// Gain of gyros
        /// </summary>
        [DataMember]
        public float XG, YG, ZG;
    }


	/// <summary>
	/// Current state of the MotionPlus controller
	/// </summary>
	[Serializable]
	[DataContract]
	public struct MotionPlusState
	{
        /// <summary>
        /// Calibration data for MontionPlus
        /// </summary>
        [DataMember]
        public MontionPlusCalibrationInfo CalibrationInfo;

        /// <summary>
        /// GYro values after filter
        /// <remarks>Values range between 0 - ?</remarks>
        /// </summary>
        [DataMember]
        public Point3F FilteredValues;

      

		/// <summary>
		/// Raw speed data
		/// <remarks>Values range between 0 - 16384</remarks>
		/// </summary>
		[DataMember]
		public Point3 RawValues;

      

		/// <summary>
		/// Normalized speed data
		/// <remarks>Values range between 0 - ?</remarks>
		/// </summary>
		[DataMember]
		public Point3F Values;


        /// <summary>
        /// Normalized speed data
        /// <remarks>Values range between 0 - ?</remarks>
        /// </summary>
        [DataMember]
        public Point3F Values2;

		/// <summary>
		/// Yaw/Pitch/Roll rotating "quickly" (no definition for "quickly" yet...)
		/// </summary>
		[DataMember]
		public bool YawFast, PitchFast, RollFast;


        /// <summary>
        ///  Pitch and Roll and Yaw
        /// </summary>
        [DataMember]
        public Euler Angles;

     
	}


	/// <summary>
	/// Calibration information
	/// </summary>
	[Serializable]
	[DataContract]
	public struct BalanceBoardCalibrationInfo
	{
		/// <summary>
		/// Calibration information at 0kg
		/// </summary>
		[DataMember]
		public BalanceBoardSensors Kg0;
		/// <summary>
		/// Calibration information at 17kg
		/// </summary>
		[DataMember]
		public BalanceBoardSensors Kg17;
		/// <summary>
		/// Calibration information at 34kg
		/// </summary>
		[DataMember]
		public BalanceBoardSensors Kg34;
	}

	/// <summary>
	/// The 4 sensors on the Balance Board (short values)
	/// </summary>
	[Serializable]
	[DataContract]
	public struct BalanceBoardSensors
	{
		/// <summary>
		/// Sensor at top right
		/// </summary>
		[DataMember]
		public short TopRight;
		/// <summary>
		/// Sensor at top left
		/// </summary>
		[DataMember]
		public short TopLeft;
		/// <summary>
		/// Sensor at bottom right
		/// </summary>
		[DataMember]
		public short BottomRight;
		/// <summary>
		/// Sensor at bottom left
		/// </summary>
		[DataMember]
		public short BottomLeft;
	}

	/// <summary>
	/// The 4 sensors on the Balance Board (float values)
	/// </summary>
	[Serializable]
	[DataContract]
	public struct BalanceBoardSensorsF
	{
		/// <summary>
		/// Sensor at top right
		/// </summary>
		[DataMember]
		public float TopRight;
		/// <summary>
		/// Sensor at top left
		/// </summary>
		[DataMember]
		public float TopLeft;
		/// <summary>
		/// Sensor at bottom right
		/// </summary>
		[DataMember]
		public float BottomRight;
		/// <summary>
		/// Sensor at bottom left
		/// </summary>
		[DataMember]
		public float BottomLeft;
	}

	/// <summary>
	/// Current state of a single IR sensor
	/// </summary>
	[Serializable]
	[DataContract]
	public struct IRSensor
	{
		/// <summary>
		/// Raw values of individual sensor.  Values range between 0 - 1023 on the X axis and 0 - 767 on the Y axis.
		/// </summary>
		[DataMember]
		public Point RawPosition;
		/// <summary>
		/// Normalized values of the sensor position.  Values range between 0.0 - 1.0.
		/// </summary>
		[DataMember]
		public PointF Position;
		/// <summary>
		/// Size of IR Sensor.  Values range from 0 - 15
		/// </summary>
		[DataMember]
		public int Size;
		/// <summary>
		/// IR sensor seen
		/// </summary>
		[DataMember]
		public bool Found;
		/// <summary>
		/// Convert to human-readable string
		/// </summary>
		/// <returns>A string that represents the point.</returns>
		public override string ToString()
		{
			return string.Format("{{{0}, Size={1}, Found={2}}}", Position, Size, Found);
		}
	}

	/// <summary>
	/// Current state of the IR camera
	/// </summary>
	[Serializable]
	[DataContract]
	public struct IRState
	{
		/// <summary>
		/// Current mode of IR sensor data
		/// </summary>
		[DataMember]
		public IRMode Mode;
		/// <summary>
		/// Current state of IR sensors
		/// </summary>
		[DataMember]
		public IRSensor[] IRSensors;
		/// <summary>
		/// Raw midpoint of IR sensors 1 and 2 only.  Values range between 0 - 1023, 0 - 767
		/// </summary>
		[DataMember]
		public Point RawMidpoint;
		/// <summary>
		/// Normalized midpoint of IR sensors 1 and 2 only.  Values range between 0.0 - 1.0
		/// </summary>
		[DataMember]
		public PointF Midpoint;
	}

	/// <summary>
	/// Current state of the accelerometers
	/// </summary>
	[Serializable]
	[DataContract]
	public struct AccelState
	{
		/// <summary>
		/// Raw accelerometer data.
		/// <remarks>Values range between 0 - 255</remarks>
		/// </summary>
		[DataMember]
		public Point3 RawValues6b;


        /// <summary>
        /// Raw accelerometer data.
        /// <remarks>Values range between 0 - 1024</remarks>
        /// </summary>
        [DataMember]
        public Point3 RawValues8b;


		/// <summary>
		/// Normalized accelerometer data.
		/// <remarks>Values range between 0 - 3g, but values > 3g and &lt; -3g are inaccurate.(g=9.8m/s^2)</remarks>
		/// </summary>
		[DataMember]
		public Point3F Values;

        /// <summary>
        /// Normalized accelerometer data.
        /// <remarks>Values range between 0 - ?, but values > 3 and &lt; -3 are inaccurate.</remarks>
        /// </summary>
        [DataMember]
        public Point3F Values2;

        /// <summary>
        /// Orientation Data
        /// <remarks>Values range between 0 - ?</remarks>
        /// </summary>
        [DataMember]
        public Point3F Orientation;


        /// <summary>
        ///  Pitch and Roll angles (Yaw can't be measured as no change in gravity)
        /// </summary>
        [DataMember]
        public Euler Angles;
	}

	/// <summary>
	/// Accelerometer calibration information
	/// </summary>
	[Serializable]
	[DataContract]
	public struct AccelCalibrationInfo
	{
		/// <summary>
		/// Zero point of accelerometer
		/// </summary>
		[DataMember]
		public byte X0, Y0, Z0;
		/// <summary>
		/// Gravity at rest of accelerometer
		/// </summary>
		[DataMember]
		public byte XG, YG, ZG;

      
	}

	/// <summary>
	/// Current button state
	/// </summary>
	[Serializable]
	[DataContract]
	public struct ButtonState
	{
		/// <summary>
		/// Digital button on the Wiimote
		/// </summary>
		[DataMember]
		public bool A, B, Plus, Home, Minus, One, Two, Up, Down, Left, Right;
	}

    /// <summary>
    /// 
    /// </summary>
    [DataContract]
    public enum ExtensionType:byte
    {
            None = 0x00,
            Nunchuck = 0x01,
            ClassicController = 0x02,
            ClassicControllerPro = 0x04,
            Guitar = 0x08,
            Drums = 0x10,
            MotionPlus = 0x20,
            BalancedBoard = 0x40,
            TaikoDrums = 0x80
    }

    /// <summary>
    /// 
    /// </summary>
    [DataContract]
    public enum PassThruMode : byte
    {
        None = 0x00,
        Nunchuck = 0x05,
        ClassicController = 0x07,
      
    }


	/// <summary>
	/// The extension number of the currently plugged into the Wiimote
	/// </summary>
	[DataContract]
	public enum ExtensionNumber : long
	{

     
		/// <summary>
		/// No extension
		/// </summary>
		None				= 0x000000000000,
		/// <summary>
		/// Nunchuk extension
		/// </summary>
		Nunchuk				= 0x0000a4200000,
		/// <summary>
		/// Classic Controller extension
		/// </summary>
		ClassicController	= 0x0000a4200101,
		/// <summary>
		/// Guitar controller from Guitar Hero 3/WorldTour
		/// </summary>
		Guitar				= 0x0000a4200103,
		/// <summary>
		/// Drum controller from Guitar Hero: World Tour
		/// </summary>
		Drums				= 0x0100a4200103,
		/// <summary>
		/// Wii Fit Balance Board controller
		/// </summary>
		BalanceBoard		= 0x0000a4200402,
		/// <summary>
		/// Taiko "TaTaCon" drum controller
		/// </summary>
		TaikoDrum			= 0x0000a4200111,
		/// <summary>
		/// Wii MotionPlus extension
		/// </summary>
		MotionPlus			= 0x0000a4200405,
        //MotionPlusInside has 0x0100 A420 0405
        //static const QWORD MOTION_PLUS		   = 0x050420A40000ULL;
		//	static const QWORD MOTION_PLUS_DETECT  = 0x050020a60000ULL;
		//	static const QWORD MOTION_PLUS_DETECT2 = 0x050420a60000ULL;
		/// <summary>
		/// Partially inserted extension.  This is an error condition.
		/// </summary>
		ParitallyInserted	= 0xffffffffffff
	};

	/// <summary>
	/// The mode of data reported for the IR sensor
	/// </summary>
	[DataContract]
	public enum IRMode : byte
	{
		/// <summary>
		/// IR sensor off
		/// </summary>
		Off			= 0x00,
		/// <summary>
		/// Basic mode
		/// </summary>
		Basic		= 0x01,	// 10 bytes
		/// <summary>
		/// Extended mode
		/// </summary>
		Extended	= 0x03,	// 12 bytes
		/// <summary>
		/// Full mode (unsupported)
		/// </summary>
		Full		= 0x05,	// 16 bytes * 2 (format unknown)
	};

	/// <summary>
	/// The report format in which the Wiimote should return data
	/// </summary>
	public enum InputReport : byte
	{
		/// <summary>
		/// Status report
		/// </summary>
		Status				= 0x20,
		/// <summary>
		/// Read data from memory location
		/// </summary>
		ReadData			= 0x21,
		/// <summary>
		/// Register write complete
		/// </summary>
		OutputReportAck		= 0x22,
		/// <summary>
		/// Button data only
		/// </summary>
		Buttons				= 0x30,
		/// <summary>
		/// Button and accelerometer data
		/// </summary>
		ButtonsAccel		= 0x31,
		/// <summary>
		/// IR sensor and accelerometer data
		/// </summary>
		IRAccel				= 0x33,
		/// <summary>
		/// Button and extension controller data
		/// </summary>
		ButtonsExtension	= 0x34,
		/// <summary>
		/// Extension and accelerometer data
		/// </summary>
		ExtensionAccel		= 0x35,
		/// <summary>
		/// IR sensor, extension controller and accelerometer data
		/// </summary>
		IRExtensionAccel	= 0x37,
        /// <summary>
        /// //0x3e / 0x3f: Interleaved Core Buttons and Accelerometer with 36 IR bytes
        /// </summary>
        InterleaveButtonsAccellIR1 = 0x3e,
        /// <summary>
        /// //0x3e / 0x3f: Interleaved Core Buttons and Accelerometer with 36 IR bytes
        /// </summary>
        InterleaveButtonsAccellIR2 = 0x3f,
	};

	/// <summary>
	/// Sensitivity of the IR camera on the Wiimote
	/// </summary>

	public enum IRSensitivity
	{
		/// <summary>
		/// Equivalent to level 1 on the Wii console
		/// </summary>
		WiiLevel1,
		/// <summary>
		/// Equivalent to level 2 on the Wii console
		/// </summary>
		WiiLevel2,
		/// <summary>
		/// Equivalent to level 3 on the Wii console (default)
		/// </summary>
		WiiLevel3,
		/// <summary>
		/// Equivalent to level 4 on the Wii console
		/// </summary>
		WiiLevel4,
		/// <summary>
		/// Equivalent to level 5 on the Wii console
		/// </summary>
		WiiLevel5,
		/// <summary>
		/// Maximum sensitivity
		/// </summary>
		Maximum
	}

	/// <summary>
	/// Type of guitar extension: Guitar Hero 3 or Guitar Hero World Tour
	/// </summary>
	public enum GuitarType
	{
		/// <summary>
		///  Guitar Hero 3 guitar controller
		/// </summary>
		GuitarHero3,
		/// <summary>
		/// Guitar Hero: World Tour guitar controller
		/// </summary>
		GuitarHeroWorldTour
	}

	/// <summary>
	/// Last ReadData status
	/// </summary>
	public enum LastReadStatus
	{
		/// <summary>
		/// Successful read
		/// </summary>
		Success,
		/// <summary>
		/// Attempt to read from write only memory
		/// </summary>
		ReadFromWriteOnlyMemory
	};
}
