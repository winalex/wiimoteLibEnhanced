//////////////////////////////////////////////////////////////////////////////////
//	Wiimote.cs
//	Managed Wiimote Library
//	Written by Brian Peek (http://www.brianpeek.com/)
//	for MSDN's Coding4Fun (http://msdn.microsoft.com/coding4fun/)
//	Visit http://blogs.msdn.com/coding4fun/archive/2007/03/14/1879033.aspx
//  and http://www.codeplex.com/WiimoteLib
//	for more information
//////////////////////////////////////////////////////////////////////////////////

using System;
using System.Runtime.InteropServices;
using System.Diagnostics;
using System.IO;
using System.Runtime.Serialization;
using Microsoft.Win32.SafeHandles;
using System.Threading;
using System.Collections.Generic;

namespace WiimoteLib
{
	/// <summary>
	/// Implementation of Wiimote
	/// </summary>
	public class Wiimote : IDisposable
	{
		/// <summary>
		/// Event raised when Wiimote state is changed
		/// </summary>
		public event EventHandler<WiimoteChangedEventArgs> WiimoteChanged;

		/// <summary>
		/// Event raised when an extension is inserted or removed
		/// </summary>
		public event EventHandler<WiimoteExtensionChangedEventArgs> WiimoteExtensionChanged;

        public bool USE_FACTORY_CALIBRATION = true;

		// VID = Nintendo, PID = Wiimote
		private const int VID = 0x057e;
		private const int PID = 0x0306;


     
      
      
        private const int PID_MOTION_PLUS_INSIDE = 0x0330;

		// sure, we could find this out the hard way using HID, but trust me, it's 22
		private const int REPORT_LENGTH = 22;

		// Wiimote output commands
		private enum OutputReport : byte
		{
			LEDs			= 0x11,
			DataReportType	= 0x12,
			IR				= 0x13,
			Status			= 0x15,
			WriteMemory		= 0x16,
			ReadMemory		= 0x17,
			IR2				= 0x1a,
		};

		// Wiimote registers
		private const int REGISTER_IR				= 0x04b00030;
		private const int REGISTER_IR_SENSITIVITY_1	= 0x04b00000;
		private const int REGISTER_IR_SENSITIVITY_2	= 0x04b0001a;
		private const int REGISTER_IR_MODE			= 0x04b00033;

		private const int REGISTER_EXTENSION_INIT_1			= 0x04a400f0;
		private const int REGISTER_EXTENSION_INIT_2			= 0x04a400fb;
        private const int REGISTER_EXTENSION_TYPE           = 0x04a400fa;//0XA400FA
		private const int REGISTER_EXTENSION_TYPE_2			= 0x04a400fe;//0xA400FE 
		private const int REGISTER_EXTENSION_CALIBRATION	= 0x04a40020;
        private const int REGISTER_CALIBRATION = 0x0016; //standard
		private const int REGISTER_MOTIONPLUS_INIT			= 0x04a600fe;
        private const int REGISTER_MOTIONPLUS_CALIBRATION = 0x04a60020;// 0x04A40000;
        private const int REGISTER_MOTIONPLUS_DETECT = 0x04a600fe;
        //0x(4)A400FA


		// length between board sensors
		private const int BSL = 43;

		// width between board sensors
		private const int BSW = 24;

       // read/write handle to the device
		private SafeFileHandle mHandle;

		// a pretty .NET stream to read/write from/to
		private FileStream mStream;

		// read data buffer
		private byte[] mReadBuff;

		// address to read from
		private int mAddress;

		// size of requested read
		private short mSize;

		// current state of controller
		private readonly WiimoteState mWiimoteState = new WiimoteState();

		// event for read data processing
		private readonly AutoResetEvent mReadDone = new AutoResetEvent(false);
		private readonly AutoResetEvent mWriteDone = new AutoResetEvent(false);

		// event for status report
		private readonly AutoResetEvent mStatusDone = new AutoResetEvent(false);

		// use a different method to write reports
		private bool mAltWriteMethod;

		// HID device path of this Wiimote
		private string mDevicePath = string.Empty;

		// unique ID
		private readonly Guid mID = Guid.NewGuid();

		// delegate used for enumerating found Wiimotes
		internal delegate bool WiimoteFoundDelegate(string devicePath);

		// kilograms to pounds
		private const float KG2LB = 2.20462262f;
  

        //Convert some units
        public static double RAD_TO_DEG = 180 / Math.PI;
        public static double DEG_TO_RAD = Math.PI / 180;

      

        //Interleave mode for use of M+ and Extension toghter
        protected PassThruMode PASS_THRU_MODE = PassThruMode.None;


        public  Point3F mMaxNoise;
       private  List<Point3F> mNoise;
        private  Point3F mBias;
        private  Point3F mMinNoise;

        private  Stopwatch mCalibrationStopWatch;
        private Point3F mPrevAngleRates;
        private Point3F mPrevAngles;
        private bool mMotionPlusCalibrated;

        // New calibration
	    // Use of mNoise vector seems to accumulate error
	    // TODO: find out why, fix, and use mNoise instead
	    int numCalibrationReadings;
	    double pitchSum = 0;
	    double yawSum = 0;
	    double rollSum = 0;

        private const double NOISE_FILTER =1.5;
        private const double CALIB_TIME =5000;//ms
       

        private  double mCalibrationTimeout;

        private Point3F mNoiseLevel;
        private Point3F mNoiseThreshold;

       
        private Point3 mNotHaveSubstaintialValue;

        private bool isStatusReady = true;
        private bool isMotionPlusDisabled = false;

		/// <summary>
		/// Default constructor
		/// </summary>
		public Wiimote()
		{

           mNoise = new List<Point3F>();
           mPrevAngleRates = new Point3F();
           mPrevAngles = new Point3F();
           
		}

		internal Wiimote(string devicePath):this()
		{
			mDevicePath = devicePath;
            
		}

		/// <summary>
		/// Connect to the first-found Wiimote
		/// </summary>
		/// <exception cref="WiimoteNotFoundException">Wiimote not found in HID device list</exception>
		public void Connect()
		{
			if(string.IsNullOrEmpty(mDevicePath))
				FindWiimote(WiimoteFound);
			else
				OpenWiimoteDeviceHandle(mDevicePath);
		}

		internal static void FindWiimote(WiimoteFoundDelegate wiimoteFound)
		{
			int index = 0;
			bool found = false;
			Guid guid;
			SafeFileHandle mHandle;

			// get the GUID of the HID class
			HIDImports.HidD_GetHidGuid(out guid);

			// get a handle to all devices that are part of the HID class
			// Fun fact:  DIGCF_PRESENT worked on my machine just fine.  I reinstalled Vista, and now it no longer finds the Wiimote with that parameter enabled...
			IntPtr hDevInfo = HIDImports.SetupDiGetClassDevs(ref guid, null, IntPtr.Zero, HIDImports.DIGCF_DEVICEINTERFACE);// | HIDImports.DIGCF_PRESENT);

			// create a new interface data struct and initialize its size
			HIDImports.SP_DEVICE_INTERFACE_DATA diData = new HIDImports.SP_DEVICE_INTERFACE_DATA();
			diData.cbSize = Marshal.SizeOf(diData);

			// get a device interface to a single device (enumerate all devices)
			while(HIDImports.SetupDiEnumDeviceInterfaces(hDevInfo, IntPtr.Zero, ref guid, index, ref diData))
			{
				UInt32 size;

				// get the buffer size for this device detail instance (returned in the size parameter)
				HIDImports.SetupDiGetDeviceInterfaceDetail(hDevInfo, ref diData, IntPtr.Zero, 0, out size, IntPtr.Zero);

				// create a detail struct and set its size
				HIDImports.SP_DEVICE_INTERFACE_DETAIL_DATA diDetail = new HIDImports.SP_DEVICE_INTERFACE_DETAIL_DATA();

				// yeah, yeah...well, see, on Win x86, cbSize must be 5 for some reason.  On x64, apparently 8 is what it wants.
				// someday I should figure this out.  Thanks to Paul Miller on this...
				diDetail.cbSize = (uint)(IntPtr.Size == 8 ? 8 : 5);

				// actually get the detail struct
				if(HIDImports.SetupDiGetDeviceInterfaceDetail(hDevInfo, ref diData, ref diDetail, size, out size, IntPtr.Zero))
				{
				//	Debug.WriteLine(string.Format("{0}: {1} - {2}", index, diDetail.DevicePath, Marshal.GetLastWin32Error()));

					// open a read/write handle to our device using the DevicePath returned
					mHandle = HIDImports.CreateFile(diDetail.DevicePath, FileAccess.ReadWrite, FileShare.ReadWrite, IntPtr.Zero, FileMode.Open, HIDImports.EFileAttributes.Overlapped, IntPtr.Zero);

					// create an attributes struct and initialize the size
					HIDImports.HIDD_ATTRIBUTES attrib = new HIDImports.HIDD_ATTRIBUTES();
					attrib.Size = Marshal.SizeOf(attrib);

					// get the attributes of the current device
					if(HIDImports.HidD_GetAttributes(mHandle.DangerousGetHandle(), ref attrib))
					{
						// if the vendor and product IDs match up
                        if ((attrib.VendorID == VID && attrib.ProductID == PID) || (attrib.VendorID == VID && attrib.ProductID == PID_MOTION_PLUS_INSIDE))
			
						{
							// it's a Wiimote
							Debug.WriteLine("Found one!");
							found = true;



							// fire the callback function...if the callee doesn't care about more Wiimotes, break out
							if(!wiimoteFound(diDetail.DevicePath))
								break;
						}
					}
					mHandle.Close();
				}
				else
				{
					// failed to get the detail struct
					throw new WiimoteException("SetupDiGetDeviceInterfaceDetail failed on index " + index);
				}

				// move to the next device
				index++;
			}

			// clean up our list
			HIDImports.SetupDiDestroyDeviceInfoList(hDevInfo);

			// if we didn't find a Wiimote, throw an exception
			if(!found)
				throw new WiimoteNotFoundException("No Wiimotes found in HID device list.");
		}

		private bool WiimoteFound(string devicePath)
		{
			mDevicePath = devicePath;

			// if we didn't find a Wiimote, throw an exception
			OpenWiimoteDeviceHandle(mDevicePath);

			return false;
		}

		private void OpenWiimoteDeviceHandle(string devicePath)
		{
			// open a read/write handle to our device using the DevicePath returned
			mHandle = HIDImports.CreateFile(devicePath, FileAccess.ReadWrite, FileShare.ReadWrite, IntPtr.Zero, FileMode.Open, HIDImports.EFileAttributes.Overlapped, IntPtr.Zero);

			// create an attributes struct and initialize the size
			HIDImports.HIDD_ATTRIBUTES attrib = new HIDImports.HIDD_ATTRIBUTES();
			attrib.Size = Marshal.SizeOf(attrib);

			// get the attributes of the current device
			if(HIDImports.HidD_GetAttributes(mHandle.DangerousGetHandle(), ref attrib))
			{
				// if the vendor and product IDs match up
                if ((attrib.VendorID == VID && attrib.ProductID == PID) || (attrib.VendorID == VID && attrib.ProductID == PID_MOTION_PLUS_INSIDE))
				{
					// create a nice .NET FileStream wrapping the handle above
					mStream = new FileStream(mHandle, FileAccess.ReadWrite, REPORT_LENGTH, true);

                   // DisableMotionPlus();

                    WriteData(REGISTER_EXTENSION_INIT_2, 0x00);

					// start an async read operation on it
					BeginAsyncRead();

                   
                    try
                    {
                        ReadWiimoteCalibration();
                    }
                    catch
                    {
                        // if we fail above, try the alternate HID writes
                        mAltWriteMethod = true;
                        ReadWiimoteCalibration();
                    }

                    

					// force a status check to get the state of any extensions plugged in at startup
					GetStatus();
				}
				else
				{
					// otherwise this isn't the controller, so close up the file handle
					mHandle.Close();				
					throw new WiimoteException("Attempted to open a non-Wiimote device.");
				}
			}
		}

        /// <summary>
        /// 
        /// </summary>
        public bool CheckMotionPlusCapabilities()
        {
            if ((mWiimoteState.Extension & (byte)ExtensionType.MotionPlus)!=0x00) return true;

            Debug.WriteLine("Try:1 to MOTIONPLUS_DETECT");
            BeginAsyncRead();
            byte[] buff = ReadData(REGISTER_MOTIONPLUS_DETECT, 0x02);

            if (buff[1] != 0x05)
            {
                Thread.Sleep(3000);

                Debug.WriteLine("Try:2 to MOTIONPLUS_DETECT");
                BeginAsyncRead();
                buff = ReadData(REGISTER_MOTIONPLUS_DETECT, 0x02);
           

                if (buff[1] != 0x05)
                {
                    Thread.Sleep(3000);

                    Debug.WriteLine("Try:3 to MOTIONPLUS_DETECT");
                    BeginAsyncRead();
                    buff = ReadData(REGISTER_MOTIONPLUS_DETECT, 0x02);
                }
            }

            Debug.WriteLine("Detected:" + (buff[1] == 0x05) + "buff:" + BitConverter.ToString(buff));

            return (buff[1] == 0x05);
        }
             


		/// <summary>
		/// Initialize the MotionPlus extension
		/// </summary>
		public void InitializeMotionPlus()
		{
           
          
        

			



            if ((mWiimoteState.Extension & (byte)ExtensionType.MotionPlus) == 0 )
            {
                Debug.WriteLine("InitializeMotionPlus");

                // Initialize it:
                WriteData(0x4a600f0, 0x55);

                WriteData(REGISTER_MOTIONPLUS_INIT, 0x04);//0xa600fe
            }

		}


  // void capability_discoverer::determine_capabilities(int wiimote_number)
  //{
  //  reader.read(wiimote_number, 0xA600FE, 0x02, std::bind(&capability_discoverer::handle_motionplus_id_message, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  //  init_and_identify_extension_controller(wiimote_number);
  //}

       


        /// <summary>
        /// 
        /// </summary>
        public void DisableMotionPlus()
        {
            isMotionPlusDisabled = true;
           
                WriteData(REGISTER_EXTENSION_INIT_1, 0x55);
                WriteData(REGISTER_EXTENSION_INIT_2, 0x00);

                this.WiimoteState.Extension &= (byte)0xDF;

                Debug.WriteLineIf(((this.WiimoteState.Extension & (byte)ExtensionType.MotionPlus) == 0), "MotionPlus disabled");
        }

		/// <summary>
		/// Disconnect from the controller and stop reading data from it
		/// </summary>
		public void Disconnect()
		{
			// close up the stream and handle
			if(mStream != null)
				mStream.Close();

			if(mHandle != null)
				mHandle.Close();
		}

		/// <summary>
		/// Start reading asynchronously from the controller
		/// </summary>
		private void BeginAsyncRead()
		{
			// if the stream is valid and ready
			if(mStream != null && mStream.CanRead)
			{
				// setup the read and the callback
				byte[] buff = new byte[REPORT_LENGTH];
				mStream.BeginRead(buff, 0, REPORT_LENGTH, new AsyncCallback(OnReadData), buff);
			}
		}

		/// <summary>
		/// Callback when data is ready to be processed
		/// </summary>
		/// <param name="ar">State information for the callback</param>
		private void OnReadData(IAsyncResult ar)
		{
			// grab the byte buffer
			byte[] buff = (byte[])ar.AsyncState;

			try
			{
				// end the current read
				mStream.EndRead(ar);

				// parse it
				if(ParseInputReport(buff))
				{
					// post an event
					if(WiimoteChanged != null)
						WiimoteChanged(this, new WiimoteChangedEventArgs(mWiimoteState));
				}

				// start reading again
				BeginAsyncRead();
			}
			catch(OperationCanceledException)
			{
				Debug.WriteLine("OperationCanceledException");
			}
		}

		/// <summary>
		/// Parse a report sent by the Wiimote
		/// </summary>
		/// <param name="buff">Data buffer to parse</param>
		/// <returns>Returns a boolean noting whether an event needs to be posted</returns>
		private bool ParseInputReport(byte[] buff)
		{
			InputReport type = (InputReport)buff[0];

           // Debug.WriteLine(type.ToString() + "  DATA:" + BitConverter.ToString(buff));

			switch(type)
			{
				case InputReport.Buttons:
					ParseButtons(buff);
					break;
				case InputReport.ButtonsAccel:
					ParseButtons(buff);
					ParseAccel(buff);
                   
                  	break;
				case InputReport.IRAccel:
					ParseButtons(buff);
					ParseAccel(buff);
					ParseIR(buff);
                  
					break;
				case InputReport.ButtonsExtension:
					ParseButtons(buff);
					ParseExtension(buff, 3);
					break;
				case InputReport.ExtensionAccel:
					ParseButtons(buff);
					ParseAccel(buff);
					ParseExtension(buff, 6);
                  
					break;
				case InputReport.IRExtensionAccel:
					ParseButtons(buff);
					ParseAccel(buff);
					ParseIR(buff);
					ParseExtension(buff, 16);
                  
					break;
				case InputReport.Status:

                    if (!isStatusReady) break;;

                  

					Debug.WriteLine("******** STATUS ********");
					ParseButtons(buff);
					mWiimoteState.BatteryRaw = buff[6];
					mWiimoteState.Battery = (((100.0f * 48.0f * (float)((int)buff[6] / 48.0f))) / 192.0f);

					// get the real LED values in case the values from SetLEDs() somehow becomes out of sync, which really shouldn't be possible
					mWiimoteState.LEDState.LED1 = (buff[3] & 0x10) != 0;
					mWiimoteState.LEDState.LED2 = (buff[3] & 0x20) != 0;
					mWiimoteState.LEDState.LED3 = (buff[3] & 0x40) != 0;
					mWiimoteState.LEDState.LED4 = (buff[3] & 0x80) != 0;

					
				

					// extension connected?
					bool extension = (buff[3] & 0x02) != 0;
					Debug.WriteLine("Extension, Old: " + (mWiimoteState.Extension!=0x00) + ", New: " + extension);



                 

                        if (extension)
                        {
                             
                               long extensionNumber;


                               //read extension type
                               BeginAsyncRead();

                               buff = ReadData(REGISTER_EXTENSION_TYPE, 6);
                               extensionNumber = ((long)buff[0] << 40) | ((long)buff[1] << 32) | ((long)buff[2]) << 24 | ((long)buff[3]) << 16 | ((long)buff[4]) << 8 | buff[5];



                               if (PASS_THRU_MODE!=PassThruMode.None)//NOT TESTED (PSEUDO)
                                {


                                    if ((extensionNumber & 0x0000ffffffff) == (long)ExtensionNumber.Nunchuk)
                                    {

                                        WriteData(REGISTER_MOTIONPLUS_INIT, (byte)PassThruMode.Nunchuck);

                                    }
                                    else if ((extensionNumber & 0x0000ffffffff) == (long)ExtensionNumber.ClassicController)
                                    {
                                        // DisableMotionPlus();/???
                                        WriteData(REGISTER_MOTIONPLUS_INIT, (byte)PassThruMode.ClassicController);
                                    }
                                           

                                }




                                InitializeExtension(extensionNumber);
                        
                        

                           // mWiimoteState.ExtensionType = ExtensionNumber.None;
                            

                        // only fire the extension changed event if we have a real extension (i.e. not a balance board)
                        if (WiimoteExtensionChanged != null && mWiimoteState.Extension != (byte)ExtensionType.BalancedBoard)
                            WiimoteExtensionChanged(this, new WiimoteExtensionChangedEventArgs((ExtensionNumber)extensionNumber, extension));


                      
                     }
                    else if(!isMotionPlusDisabled)
                    {
                        if (CheckMotionPlusCapabilities())
                        {
                            mStatusDone.Set();
                          
                            //initalization would call GetStatus automatically
                            InitializeMotionPlus();
                        }
                    }
                    
                    
					  mStatusDone.Set();

                    
					break;
				case InputReport.ReadData:
					ParseButtons(buff);
					ParseReadData(buff);
					break;
				case InputReport.OutputReportAck:
                    //Debug.WriteLine(type.ToString() + "  DATA:" + BitConverter.ToString(buff));
//					Debug.WriteLine("ack: " + buff[0] + " " +  buff[1] + " " +buff[2] + " " +buff[3] + " " +buff[4]);
					mWriteDone.Set();
					break;


                    //(a1) 3e BB BB AA II II II II II II II II II II II II II II II II II II
                    //(a1) 3f BB BB AA II II II II II II II II II II II II II II II II II II
                case InputReport.InterleaveButtonsAccellIR1:
                    ParseButtons(buff);
                    ParseAccel(type, buff);
                    break;
                case InputReport.InterleaveButtonsAccellIR2:
                    ParseButtons(buff);
                    ParseAccel(type, buff);
                    CalculateAccelRotation();
                    break;
				default:
					Debug.WriteLine("Unknown report type: " + type.ToString("x"));
					return false;
			}

			return true;
		}

		/// <summary>
		/// Handles setting up an extension when plugged in
		/// </summary>
         private void InitializeExtension(long type)
		//private void InitializeExtension(byte extensionType)
        //private void InitializeExtension()
		{
			

			// only initialize if it's not a MotionPlus
            //if (extensionType != 0x04)
            //{
            //    WriteData(REGISTER_EXTENSION_INIT_1, 0x55);
            //    WriteData(REGISTER_EXTENSION_INIT_2, 0x00);
            //}

			// start reading again
			//BeginAsyncRead();

            byte[] buff;
			//byte[] buff = ReadData(REGISTER_EXTENSION_TYPE, 6);
			//long type = ((long)buff[0] << 40) | ((long)buff[1] << 32) | ((long)buff[2]) << 24 | ((long)buff[3]) << 16 | ((long)buff[4]) << 8 | buff[5];

            //alex winx think need mask(as first 4bytes aren't inportant or subversion) except
            //ExtensionType.Guitar and ExtensionType.Drums having same base (seem have same hardware)
            if(type != (long)ExtensionNumber.Guitar && type!=(long)ExtensionNumber.Drums)
            type=type & 0x0000ffffffff;

			switch((ExtensionNumber)type)
			{
				case ExtensionNumber.None:
				case ExtensionNumber.ParitallyInserted:
				
					//mWiimoteState.ExtensionType = ExtensionNumber.None;
					return;
				case ExtensionNumber.Nunchuk:
                    if(PASS_THRU_MODE==(byte)PassThruMode.None)
                        mWiimoteState.Extension = (byte)ExtensionType.Nunchuck;
                    else 
                        mWiimoteState.Extension |= (byte)ExtensionType.Nunchuck;


                    buff = ReadData(REGISTER_EXTENSION_CALIBRATION, 16);

                    //X0,Y0,Z0 are actully MID
                    //XG,YG,ZG are actually MAX
					mWiimoteState.NunchukState.CalibrationInfo.AccelCalibration.X0 = buff[0];
					mWiimoteState.NunchukState.CalibrationInfo.AccelCalibration.Y0 = buff[1];
					mWiimoteState.NunchukState.CalibrationInfo.AccelCalibration.Z0 = buff[2];
					mWiimoteState.NunchukState.CalibrationInfo.AccelCalibration.XG = buff[4];
					mWiimoteState.NunchukState.CalibrationInfo.AccelCalibration.YG = buff[5];
					mWiimoteState.NunchukState.CalibrationInfo.AccelCalibration.ZG = buff[6];
                    mWiimoteState.NunchukState.CalibrationInfo.MaxX = buff[8]; //seem this calibration isn't correct
					mWiimoteState.NunchukState.CalibrationInfo.MinX = buff[9];
					mWiimoteState.NunchukState.CalibrationInfo.MidX = buff[10];
					mWiimoteState.NunchukState.CalibrationInfo.MaxY = buff[11];
					mWiimoteState.NunchukState.CalibrationInfo.MinY = buff[12];
					mWiimoteState.NunchukState.CalibrationInfo.MidY = buff[13];
                    //mWiimoteState.NunchukState.CalibrationInfo.MaxX =228;// buff[8]; seem this calibration isn't correct
                    //mWiimoteState.NunchukState.CalibrationInfo.MinX =22;// buff[9];
                    //mWiimoteState.NunchukState.CalibrationInfo.MidX =125;// buff[10];
                    //mWiimoteState.NunchukState.CalibrationInfo.MaxY =228;// buff[11];
                    //mWiimoteState.NunchukState.CalibrationInfo.MinY =36;//28;// buff[12];
                    //mWiimoteState.NunchukState.CalibrationInfo.MidY =132;// buff[13];

                    Debug.WriteLine(mWiimoteState.NunchukState.CalibrationInfo.MaxX + " " + mWiimoteState.NunchukState.CalibrationInfo.MidX + " " + mWiimoteState.NunchukState.CalibrationInfo.MinX);
                    Debug.WriteLine(mWiimoteState.NunchukState.CalibrationInfo.AccelCalibration.X0 + " " + mWiimoteState.NunchukState.CalibrationInfo.AccelCalibration.XG);

                    break;
				case ExtensionNumber.ClassicController:
                    if(PASS_THRU_MODE==(byte)PassThruMode.None)
                          mWiimoteState.Extension = (int)ExtensionType.ClassicController;
                      else
                         mWiimoteState.Extension |= (int)ExtensionType.ClassicController;

                    buff = ReadData(REGISTER_EXTENSION_CALIBRATION, 16);

					mWiimoteState.ClassicControllerState.CalibrationInfo.MaxXL = (byte)(buff[0] >> 2);
					mWiimoteState.ClassicControllerState.CalibrationInfo.MinXL = (byte)(buff[1] >> 2);
					mWiimoteState.ClassicControllerState.CalibrationInfo.MidXL = (byte)(buff[2] >> 2);
					mWiimoteState.ClassicControllerState.CalibrationInfo.MaxYL = (byte)(buff[3] >> 2);
					mWiimoteState.ClassicControllerState.CalibrationInfo.MinYL = (byte)(buff[4] >> 2);
					mWiimoteState.ClassicControllerState.CalibrationInfo.MidYL = (byte)(buff[5] >> 2);

					mWiimoteState.ClassicControllerState.CalibrationInfo.MaxXR = (byte)(buff[6] >> 3);
					mWiimoteState.ClassicControllerState.CalibrationInfo.MinXR = (byte)(buff[7] >> 3);
					mWiimoteState.ClassicControllerState.CalibrationInfo.MidXR = (byte)(buff[8] >> 3);
					mWiimoteState.ClassicControllerState.CalibrationInfo.MaxYR = (byte)(buff[9] >> 3);
					mWiimoteState.ClassicControllerState.CalibrationInfo.MinYR = (byte)(buff[10] >> 3);
					mWiimoteState.ClassicControllerState.CalibrationInfo.MidYR = (byte)(buff[11] >> 3);

					// this doesn't seem right...
//					mWiimoteState.ClassicControllerState.AccelCalibrationInfo.MinTriggerL = (byte)(buff[12] >> 3);
//					mWiimoteState.ClassicControllerState.AccelCalibrationInfo.MaxTriggerL = (byte)(buff[14] >> 3);
//					mWiimoteState.ClassicControllerState.AccelCalibrationInfo.MinTriggerR = (byte)(buff[13] >> 3);
//					mWiimoteState.ClassicControllerState.AccelCalibrationInfo.MaxTriggerR = (byte)(buff[15] >> 3);
					mWiimoteState.ClassicControllerState.CalibrationInfo.MinTriggerL = 0;
					mWiimoteState.ClassicControllerState.CalibrationInfo.MaxTriggerL = 31;
					mWiimoteState.ClassicControllerState.CalibrationInfo.MinTriggerR = 0;
					mWiimoteState.ClassicControllerState.CalibrationInfo.MaxTriggerR = 31;
                break;
				case ExtensionNumber.Guitar:
                     mWiimoteState.Extension = (int)ExtensionType.Guitar;
                break;
				case ExtensionNumber.BalanceBoard:
                    mWiimoteState.Extension = (int)ExtensionType.BalancedBoard;

                    buff = ReadData(REGISTER_EXTENSION_CALIBRATION, 32);

					mWiimoteState.BalanceBoardState.CalibrationInfo.Kg0.TopRight =		(short)((short)buff[4] << 8 | buff[5]);
					mWiimoteState.BalanceBoardState.CalibrationInfo.Kg0.BottomRight =	(short)((short)buff[6] << 8 | buff[7]);
					mWiimoteState.BalanceBoardState.CalibrationInfo.Kg0.TopLeft =		(short)((short)buff[8] << 8 | buff[9]);
					mWiimoteState.BalanceBoardState.CalibrationInfo.Kg0.BottomLeft =	(short)((short)buff[10] << 8 | buff[11]);

					mWiimoteState.BalanceBoardState.CalibrationInfo.Kg17.TopRight =		(short)((short)buff[12] << 8 | buff[13]);
					mWiimoteState.BalanceBoardState.CalibrationInfo.Kg17.BottomRight =	(short)((short)buff[14] << 8 | buff[15]);
					mWiimoteState.BalanceBoardState.CalibrationInfo.Kg17.TopLeft =		(short)((short)buff[16] << 8 | buff[17]);
					mWiimoteState.BalanceBoardState.CalibrationInfo.Kg17.BottomLeft =	(short)((short)buff[18] << 8 | buff[19]);

					mWiimoteState.BalanceBoardState.CalibrationInfo.Kg34.TopRight =		(short)((short)buff[20] << 8 | buff[21]);
					mWiimoteState.BalanceBoardState.CalibrationInfo.Kg34.BottomRight =	(short)((short)buff[22] << 8 | buff[23]);
					mWiimoteState.BalanceBoardState.CalibrationInfo.Kg34.TopLeft =		(short)((short)buff[24] << 8 | buff[25]);
					mWiimoteState.BalanceBoardState.CalibrationInfo.Kg34.BottomLeft =	(short)((short)buff[26] << 8 | buff[27]);
                break;
				case ExtensionNumber.Drums:
                    mWiimoteState.Extension = (int)ExtensionType.Drums;
                break;
				case ExtensionNumber.TaikoDrum:
                     mWiimoteState.Extension = (byte)ExtensionType.TaikoDrums;
                break;
				case ExtensionNumber.MotionPlus:
                     if(PASS_THRU_MODE==(byte)PassThruMode.None)
                         mWiimoteState.Extension = (byte)ExtensionType.MotionPlus;
                    else
                         mWiimoteState.Extension |= (byte)ExtensionType.MotionPlus;

                     isMotionPlusDisabled = false;
                  

					
                   

                    buff = ReadData(REGISTER_EXTENSION_CALIBRATION, 32);

                    // someday...


                    ////TEstando sem o offset - deixa que o filtro remove todo offset. S� fazer isso em slow
                    ////Not sure this order is correct. Need to check with other genuines MP
                    //mWiimoteState.MotionPlusState.CalibrationInfo.GyroCalibration.Z0 = (short)((short)buff[0] << 6 | buff[1] >> 2);
                    //mWiimoteState.MotionPlusState.CalibrationInfo.GyroCalibration.Y0 = (short)((short)buff[2] << 6 | buff[3] >> 2);
                    //mWiimoteState.MotionPlusState.CalibrationInfo.GyroCalibration.X0 = (short)((short)buff[4] << 6 | buff[5] >> 2);

                    ////Slow offset
                    //mWiimoteState.MotionPlusState.CalibrationInfo.GyroFastCalibration.Z0 = (short)((short)buff[0 + 16] << 6 | buff[1 + 16] >> 2);
                    //mWiimoteState.MotionPlusState.CalibrationInfo.GyroFastCalibration.Y0 = (short)((short)buff[2 + 16] << 6 | buff[3 + 16] >> 2);
                    //mWiimoteState.MotionPlusState.CalibrationInfo.GyroFastCalibration.X0 = (short)((short)buff[4 + 16] << 6 | buff[5 + 16] >> 2);

                    ////////Calibration for Epson - YAW - GZ
                    //ushort EpsonG1 = (ushort)((ushort)buff[6] << 8 | buff[7]);
                    //ushort EpsonG2 = (ushort)((ushort)buff[8] << 8 | buff[9]);
                    ////Counts/deg/s for epson. Shoulld be ~4.4
                    ////float Epson =4.4f;// (float)EpsonG2 / (float)EpsonG1;

                    //////Calibration for IDG600 - Pitch&Roll - GX,GY
                    //ushort IDGG1 = (ushort)((ushort)buff[22] << 8 | buff[23]);
                    //ushort IDGG2 = (ushort)((ushort)buff[24] << 8 | buff[25]);
                    ////COunts/deg/s for IDG. Should be ~3.8
                    ////float IDG =3.8f;// (float)IDGG2 / (float)IDGG1;

                    //////Factor Slow/Fast
                    //ushort FactorSF1 = (ushort)((ushort)buff[10] << 8 | buff[11]);
                    //ushort FactorSF2 = (ushort)((ushort)buff[12] << 8 | buff[13]);
                    ////Slow/Fast factor. Should be ~4 for original montion plus. 5 for my chinese MP.
                    ////float FactorSF =5;// (float)FactorSF2 / (float)FactorSF1;

                    //////Pith and Roll - IDG
                    //////Trying other calibration. Epson calibration seems to be right for all axis
                    //////Epson* FactorSF seems to be calibration for slow for 3 channels
                    //////OK, we need to think about the calibration again - this seems to be the best for now.
                    //////
                    ////mWiimoteState.MotionPlusState.CalibrationInfo.GyroFastCalibration.XG = FactorSF; // IDG;
                    ////mWiimoteState.MotionPlusState.CalibrationInfo.GyroFastCalibration.YG = FactorSF;
                    ////mWiimoteState.MotionPlusState.CalibrationInfo.GyroCalibration.XG = Epson * FactorSF; //IDG* FactorSF;
                    ////mWiimoteState.MotionPlusState.CalibrationInfo.GyroCalibration.YG = Epson * FactorSF; // IDG* FactorSF;

                    //////YAW - Epson
                    ////mWiimoteState.MotionPlusState.CalibrationInfo.GyroFastCalibration.ZG = FactorSF;
                    ////mWiimoteState.MotionPlusState.CalibrationInfo.GyroCalibration.ZG = Epson * FactorSF;
					break;
				default:
					throw new WiimoteException("Unknown extension controller found: " + type.ToString("x"));
			}

            Debug.WriteLine("InitExtension " + (ExtensionNumber)type);

          
		}

		/// <summary>
		/// Decrypts data sent from the extension to the Wiimote
		/// </summary>
		/// <param name="buff">Data buffer</param>
		/// <returns>Byte array containing decoded data</returns>
		private byte[] DecryptBuffer(byte[] buff)
		{
			for(int i = 0; i < buff.Length; i++)
				buff[i] = (byte)(((buff[i] ^ 0x17) + 0x17) & 0xff);

			return buff;
		}

		/// <summary>
		/// Parses a standard button report into the ButtonState struct
		/// </summary>
		/// <param name="buff">Data buffer</param>
		private void ParseButtons(byte[] buff)
		{
			mWiimoteState.ButtonState.A		= (buff[2] & 0x08) != 0;
			mWiimoteState.ButtonState.B		= (buff[2] & 0x04) != 0;
			mWiimoteState.ButtonState.Minus	= (buff[2] & 0x10) != 0;
			mWiimoteState.ButtonState.Home	= (buff[2] & 0x80) != 0;
			mWiimoteState.ButtonState.Plus	= (buff[1] & 0x10) != 0;
			mWiimoteState.ButtonState.One	= (buff[2] & 0x02) != 0;
			mWiimoteState.ButtonState.Two	= (buff[2] & 0x01) != 0;
			mWiimoteState.ButtonState.Up	= (buff[1] & 0x08) != 0;
			mWiimoteState.ButtonState.Down	= (buff[1] & 0x04) != 0;
			mWiimoteState.ButtonState.Left	= (buff[1] & 0x01) != 0;
			mWiimoteState.ButtonState.Right	= (buff[1] & 0x02) != 0;
		}


        /// <summary>
		/// Parse accelerometer data in Interleave mode
		/// </summary>
		/// <param name="buff">Data buffer</param>
        private void ParseAccel(InputReport type,byte[] buff)
        {
            if (type == InputReport.InterleaveButtonsAccellIR1)
            {
                mWiimoteState.AccelState.RawValues6b.X = buff[3];
                mWiimoteState.AccelState.RawValues6b.Z = (buff[1] << 1 & 0xC0) | (buff[0] >> 1 & 0x30);
            }
            else
            {
                mWiimoteState.AccelState.RawValues6b.Y = buff[3];
                mWiimoteState.AccelState.RawValues6b.Z |= (buff[0] >> 5 & 0x3) | (buff[1] >> 3 & 0xC);
            }
        }

		/// <summary>
		/// Parse accelerometer data
		/// </summary>
		/// <param name="buff">Data buffer</param>
		private void ParseAccel(byte[] buff)
		{
            //output.acceleration.x = (data[4] << 2) | (data[2] >> 5 & 0x3);
            //output.acceleration.y = (data[5] << 2) | (data[3] >> 4 & 0x2);
            //output.acceleration.z = (data[6] << 2) | (data[3] >> 5 & 0x2);
            
            //Stationary X=0; Y=0; Z=1(0.96);


           // var offset = (accX + accY) / 2d;
           // var gravity = accZ - offset;
           // var gain=9.81 / gravity;


           //var value= (RawValues(X,Y,Z) - offset) * gain;
        

       

            
         

			mWiimoteState.AccelState.RawValues6b.X = buff[3];
			mWiimoteState.AccelState.RawValues6b.Y = buff[4];
			mWiimoteState.AccelState.RawValues6b.Z = buff[5];

            //Note that X has 10 bits of precision, while Y and Z only have 9. For consistency, they are assumed all to have a 10-bit range and the LSB is always set to zero for Y and Z.
            mWiimoteState.AccelState.RawValues8b.X = (buff[3] << 2) | (buff[0]>>5 & 0x3);// 
            mWiimoteState.AccelState.RawValues8b.Y =  (buff[4] << 2) | (buff[1]>>4 & 0x2);//
            mWiimoteState.AccelState.RawValues8b.Z = (buff[5] << 2) | (buff[1]>>5 & 0x2);//


            //raw_accel.x = (u8)trim(accel.x * (calib.one_g.x - calib.zero_g.x) + calib.zero_g.x);
            //raw_accel.y = (u8)trim(accel.y * (calib.one_g.y - calib.zero_g.y) + calib.zero_g.y);
            //raw_accel.z = (u8)trim(accel.z * (calib.one_g.z - calib.zero_g.z) + calib.zero_g.z);

            if (USE_FACTORY_CALIBRATION)
            {
                mWiimoteState.AccelState.Values.X = (float)((float)mWiimoteState.AccelState.RawValues6b.X - ((int)mWiimoteState.AccelCalibrationInfo.X0)) /
                                                ((float)mWiimoteState.AccelCalibrationInfo.XG - ((int)mWiimoteState.AccelCalibrationInfo.X0));
                mWiimoteState.AccelState.Values.Y = (float)((float)mWiimoteState.AccelState.RawValues6b.Y - mWiimoteState.AccelCalibrationInfo.Y0) /
                                                ((float)mWiimoteState.AccelCalibrationInfo.YG - mWiimoteState.AccelCalibrationInfo.Y0);
                mWiimoteState.AccelState.Values.Z = (float)((float)mWiimoteState.AccelState.RawValues6b.Z - mWiimoteState.AccelCalibrationInfo.Z0) /
                                                ((float)mWiimoteState.AccelCalibrationInfo.ZG - mWiimoteState.AccelCalibrationInfo.Z0);
           

    //(N-512)/1024 * 3.0V * (9.8 g)/(0.300 V/g)

            //TODO try with 3.3V
            mWiimoteState.AccelState.Values2.X = ((float)((mWiimoteState.AccelState.RawValues8b.X - (mWiimoteState.AccelCalibrationInfo.X0 <<2))) / 1024f) * (3f  / 0.3f);
            mWiimoteState.AccelState.Values2.Y = ((float)((mWiimoteState.AccelState.RawValues8b.Y - (mWiimoteState.AccelCalibrationInfo.Y0 << 2))) / 1024f) * (3f   / 0.3f);
            mWiimoteState.AccelState.Values2.Z = ((float)((mWiimoteState.AccelState.RawValues8b.Z  - (mWiimoteState.AccelCalibrationInfo.Z0 <<2))) / 1024f) * (3f   / 0.3f);

            //mWiimoteState.AccelState.Values.X = mWiimoteState.AccelState.Values2.X;
            //mWiimoteState.AccelState.Values.Y = mWiimoteState.AccelState.Values2.Y;
            //mWiimoteState.AccelState.Values.Z = mWiimoteState.AccelState.Values2.Z;
            
            
            }



            //could go +/-3g
            if(mWiimoteState.AccelState.Values.X > 3)  mWiimoteState.AccelState.Values.X = 3;
            else if (mWiimoteState.AccelState.Values.X < -3) mWiimoteState.AccelState.Values.X = -3;

            //could go +/-3g
            if (mWiimoteState.AccelState.Values.Y > 3) mWiimoteState.AccelState.Values.Y = 3;
            else if (mWiimoteState.AccelState.Values.Y < -3) mWiimoteState.AccelState.Values.Y = -3;

            if (mWiimoteState.AccelState.Values.Z > 3) mWiimoteState.AccelState.Values.Z = 3;
            else if (mWiimoteState.AccelState.Values.Z < -3) mWiimoteState.AccelState.Values.Z = -3;


            CalculateAccelRotation();

            // The real purpose of this method is to convert the accelometer values into usable angle values.  
            // see app note: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf
            // That paper gives the math for implementing a tilt sensor using 3-axis accelerometers.  Roll(X) and pitch(Y) are directly
            // applicable.  Yaw(Z) is not since there is no 'tilt' with respect to the earth.    
            // Once the accelerometer values have been biased to zero (by subtracting calibration value above), then they should fall in a range from
            // -512 to +511.
          //  double x = angleInRadians(-512, 511, ax_m), y = angleInRadians(-512, 511, ay_m), z = angleInRadians(-512, 511, az_m);
             
           // mWiimoteState.
            // compute values that are used in multiple places
            //double xSquared = x * x; double ySquared = y * y; double zSquared = z * z;
          
           // accelAngleX = (accelAngleX * 0.9 + atan(x / sqrt(ySquared + zSquared)) * 0.1);
          //  accelAngleY = (accelAngleY * 0.9 + atan(y / sqrt(xSquared + zSquared)) * 0.1);
          //  accelAngleZ = (accelAngleZ * 0.9 + atan(sqrt(xSquared + ySquared) / z) * 0.1);
            // filter readings using low pass filter for acceleromters to remove jitter
            // big percentage of previous value plus smaller percentage of current value
            // effect is delay large changes from reaching the output but at cost of reduced sensitivity

            //in accelerometar roll,pitch and yaw are deffined towards gravity 
            //so wiimote is on the side X=1 cos X axis is aligned with gravity

          //   mWiimoteState.AccelState.Values.X=0.461105f;
         //   mWiimoteState.AccelState.Values.Y=0.082198f;
        //    mWiimoteState.AccelState.Values.Z=-0.887432f;

      


            //// wiimote seems to be stationary:  normalize the current acceleration
            ////  (ie. the assumed gravity vector)
            //float inv_len = 1.f / sqrt(length_sq);
            //float x = accel.X * inv_len;
            //float y = accel.Y * inv_len;
            //float z = accel.Z * inv_len;

            //// copy the values
            //accel.Orientation.X = x;
            //accel.Orientation.Y = y;
            //accel.Orientation.Z = z;

            //// and extract pitch & roll from them:
            //// (may not be optimal)
            //float pitch = -asin(y) * 57.2957795f;
            ////		float roll  =  asin(x)    * 57.2957795f;
            //float roll = atan2(x, z) * 57.2957795f;
            //if (z < 0)
            //{
            //    pitch = (y < 0) ? 180 - pitch : -180 - pitch;
            //    roll = (x < 0) ? -180 - roll : 180 - roll;
            //}
        
        }



        private void CalculateAccelRotation()
        {
            double xSquared = mWiimoteState.AccelState.Values.X * mWiimoteState.AccelState.Values.X;
            double ySquared = mWiimoteState.AccelState.Values.Y * mWiimoteState.AccelState.Values.Y;
            double zSquared = mWiimoteState.AccelState.Values.Z * mWiimoteState.AccelState.Values.Z;


            double inv_len = 1 / Math.Sqrt(xSquared+ySquared+zSquared);
            double x = mWiimoteState.AccelState.Values.X * inv_len;
            double y = mWiimoteState.AccelState.Values.Y * inv_len;
            double z = mWiimoteState.AccelState.Values.Z * inv_len;


            //float pitch = -asin(y) * 57.2957795f;
            ////		float roll  =  asin(x)    * 57.2957795f;
            //float roll = atan2(x, z) * 57.2957795f; 
            
            
            mWiimoteState.AccelState.Angles.Roll = (float)(Math.Atan2(x, z) * RAD_TO_DEG);
            mWiimoteState.AccelState.Angles.Pitch = -(float)(Math.Asin(y) * RAD_TO_DEG);
           // mWiimoteState.AccelState.Angles.Yaw = -(float)(Math.Atan2(y, z) * RAD_TO_DEG);

            //if (z < 0)
            //{
            //    mWiimoteState.AccelState.Angles.Pitch = (y < 0) ? 180 - mWiimoteState.AccelState.Angles.Pitch : -180 - mWiimoteState.AccelState.Angles.Pitch;
            //    mWiimoteState.AccelState.Angles.Roll = (x < 0) ? -180 - mWiimoteState.AccelState.Angles.Roll : 180 - mWiimoteState.AccelState.Angles.Roll;
            //}

           
           

            //mWiimoteState.AccelState.Angles.Roll = (float)(Math.Atan2(mWiimoteState.AccelState.Values.X, Math.Sqrt(ySquared + zSquared)) * RAD_TO_DEG);
            //mWiimoteState.AccelState.Angles.Pitch = (float)(Math.Atan2(-mWiimoteState.AccelState.Values.Y, Math.Sqrt(xSquared + zSquared)) * RAD_TO_DEG);
            mWiimoteState.AccelState.Angles.Yaw = (float)(Math.Atan2(Math.Sqrt(xSquared + ySquared), mWiimoteState.AccelState.Values.Z) * RAD_TO_DEG);


        }

		/// <summary>
		/// Parse IR data from report
		/// </summary>
		/// <param name="buff">Data buffer</param>
		private void ParseIR(byte[] buff)
		{
			mWiimoteState.IRState.IRSensors[0].RawPosition.X = buff[6] | ((buff[8] >> 4) & 0x03) << 8;
			mWiimoteState.IRState.IRSensors[0].RawPosition.Y = buff[7] | ((buff[8] >> 6) & 0x03) << 8;

			switch(mWiimoteState.IRState.Mode)
			{
				case IRMode.Basic:
					mWiimoteState.IRState.IRSensors[1].RawPosition.X = buff[9]  | ((buff[8] >> 0) & 0x03) << 8;
					mWiimoteState.IRState.IRSensors[1].RawPosition.Y = buff[10] | ((buff[8] >> 2) & 0x03) << 8;

					mWiimoteState.IRState.IRSensors[2].RawPosition.X = buff[11] | ((buff[13] >> 4) & 0x03) << 8;
					mWiimoteState.IRState.IRSensors[2].RawPosition.Y = buff[12] | ((buff[13] >> 6) & 0x03) << 8;

					mWiimoteState.IRState.IRSensors[3].RawPosition.X = buff[14] | ((buff[13] >> 0) & 0x03) << 8;
					mWiimoteState.IRState.IRSensors[3].RawPosition.Y = buff[15] | ((buff[13] >> 2) & 0x03) << 8;

					mWiimoteState.IRState.IRSensors[0].Size = 0x00;
					mWiimoteState.IRState.IRSensors[1].Size = 0x00;
					mWiimoteState.IRState.IRSensors[2].Size = 0x00;
					mWiimoteState.IRState.IRSensors[3].Size = 0x00;

					mWiimoteState.IRState.IRSensors[0].Found = !(buff[6] == 0xff && buff[7] == 0xff);
					mWiimoteState.IRState.IRSensors[1].Found = !(buff[9] == 0xff && buff[10] == 0xff);
					mWiimoteState.IRState.IRSensors[2].Found = !(buff[11] == 0xff && buff[12] == 0xff);
					mWiimoteState.IRState.IRSensors[3].Found = !(buff[14] == 0xff && buff[15] == 0xff);
					break;
				case IRMode.Extended:
					mWiimoteState.IRState.IRSensors[1].RawPosition.X = buff[9]  | ((buff[11] >> 4) & 0x03) << 8;
					mWiimoteState.IRState.IRSensors[1].RawPosition.Y = buff[10] | ((buff[11] >> 6) & 0x03) << 8;
					mWiimoteState.IRState.IRSensors[2].RawPosition.X = buff[12] | ((buff[14] >> 4) & 0x03) << 8;
					mWiimoteState.IRState.IRSensors[2].RawPosition.Y = buff[13] | ((buff[14] >> 6) & 0x03) << 8;
					mWiimoteState.IRState.IRSensors[3].RawPosition.X = buff[15] | ((buff[17] >> 4) & 0x03) << 8;
					mWiimoteState.IRState.IRSensors[3].RawPosition.Y = buff[16] | ((buff[17] >> 6) & 0x03) << 8;

					mWiimoteState.IRState.IRSensors[0].Size = buff[8] & 0x0f;
					mWiimoteState.IRState.IRSensors[1].Size = buff[11] & 0x0f;
					mWiimoteState.IRState.IRSensors[2].Size = buff[14] & 0x0f;
					mWiimoteState.IRState.IRSensors[3].Size = buff[17] & 0x0f;

					mWiimoteState.IRState.IRSensors[0].Found = !(buff[6] == 0xff && buff[7] == 0xff && buff[8] == 0xff);
					mWiimoteState.IRState.IRSensors[1].Found = !(buff[9] == 0xff && buff[10] == 0xff && buff[11] == 0xff);
					mWiimoteState.IRState.IRSensors[2].Found = !(buff[12] == 0xff && buff[13] == 0xff && buff[14] == 0xff);
					mWiimoteState.IRState.IRSensors[3].Found = !(buff[15] == 0xff && buff[16] == 0xff && buff[17] == 0xff);
					break;
			}

			mWiimoteState.IRState.IRSensors[0].Position.X = (float)(mWiimoteState.IRState.IRSensors[0].RawPosition.X / 1023.5f);
			mWiimoteState.IRState.IRSensors[1].Position.X = (float)(mWiimoteState.IRState.IRSensors[1].RawPosition.X / 1023.5f);
			mWiimoteState.IRState.IRSensors[2].Position.X = (float)(mWiimoteState.IRState.IRSensors[2].RawPosition.X / 1023.5f);
			mWiimoteState.IRState.IRSensors[3].Position.X = (float)(mWiimoteState.IRState.IRSensors[3].RawPosition.X / 1023.5f);

			mWiimoteState.IRState.IRSensors[0].Position.Y = (float)(mWiimoteState.IRState.IRSensors[0].RawPosition.Y / 767.5f);
			mWiimoteState.IRState.IRSensors[1].Position.Y = (float)(mWiimoteState.IRState.IRSensors[1].RawPosition.Y / 767.5f);
			mWiimoteState.IRState.IRSensors[2].Position.Y = (float)(mWiimoteState.IRState.IRSensors[2].RawPosition.Y / 767.5f);
			mWiimoteState.IRState.IRSensors[3].Position.Y = (float)(mWiimoteState.IRState.IRSensors[3].RawPosition.Y / 767.5f);

			if(mWiimoteState.IRState.IRSensors[0].Found && mWiimoteState.IRState.IRSensors[1].Found)
			{
				mWiimoteState.IRState.RawMidpoint.X = (mWiimoteState.IRState.IRSensors[1].RawPosition.X + mWiimoteState.IRState.IRSensors[0].RawPosition.X) / 2;
				mWiimoteState.IRState.RawMidpoint.Y = (mWiimoteState.IRState.IRSensors[1].RawPosition.Y + mWiimoteState.IRState.IRSensors[0].RawPosition.Y) / 2;
		
				mWiimoteState.IRState.Midpoint.X = (mWiimoteState.IRState.IRSensors[1].Position.X + mWiimoteState.IRState.IRSensors[0].Position.X) / 2.0f;
				mWiimoteState.IRState.Midpoint.Y = (mWiimoteState.IRState.IRSensors[1].Position.Y + mWiimoteState.IRState.IRSensors[0].Position.Y) / 2.0f;
			}
			else
				mWiimoteState.IRState.Midpoint.X = mWiimoteState.IRState.Midpoint.Y = 0.0f;
		}

		/// <summary>
		/// Parse data from an extension controller
		/// </summary>
		/// <param name="buff">Data buffer</param>
		/// <param name="offset">Offset into data buffer</param>
		private void ParseExtension(byte[] buff, int offset)
		{
			
                if  ((mWiimoteState.Extension & (byte)ExtensionType.Nunchuck) !=0){
				

                  

                 
                   

                    if(PASS_THRU_MODE == PassThruMode.Nunchuck){
                        if((buff[offset + 5] & 0x03)==0x00){
   //interleave mode
                             //if (extension_data.size() >= 6 && !(extension_data[5] & 0x03))
                    //{
                    //    output.valid_data_flags |= dolphiimote_NUNCHUCK_VALID;

                    //    output.nunchuck.stick_x = extension_data[0];
                    //    output.nunchuck.stick_y = extension_data[1];
                    //    output.nunchuck.x = (extension_data[2] << 1) | (extension_data[5] & 0x10) >> 4;
                    //    output.nunchuck.y = (extension_data[3] << 1) | (extension_data[5] & 0x20) >> 5;
                    //    output.nunchuck.z = ((extension_data[4] & ~0x1) << 1) | (extension_data[5] & 0xC0) >> 6;

                    //    output.nunchuck.buttons = ~(extension_data[5] >> 2) & 0x3;
                    //}



                        mWiimoteState.NunchukState.RawJoystick.X = buff[offset];
                        mWiimoteState.NunchukState.RawJoystick.Y = buff[offset + 1];


                        mWiimoteState.NunchukState.AccelState.RawValues6b.X = buff[offset + 2];
                        mWiimoteState.NunchukState.AccelState.RawValues6b.Y = buff[offset + 3];
                        mWiimoteState.NunchukState.AccelState.RawValues6b.Z = buff[offset + 4];


                        mWiimoteState.NunchukState.AccelState.RawValues8b.X = (buff[offset + 2]<<1) | (buff[offset + 5] & 0x10) >> 4;
                        mWiimoteState.NunchukState.AccelState.RawValues8b.Y = (buff[offset + 3] << 1)| (buff[offset + 5] & 0x30) >> 4;;
                        mWiimoteState.NunchukState.AccelState.RawValues8b.Z = ((buff[offset + 4]& ~0x1) << 1) | (buff[offset + 5] & 0xC0) >> 6;



                        }else return;
                        
                    }  
                    else
                    {

                        mWiimoteState.NunchukState.RawJoystick.X = buff[offset];
                        mWiimoteState.NunchukState.RawJoystick.Y = buff[offset + 1];

                    
                      
                        mWiimoteState.NunchukState.AccelState.RawValues6b.X = buff[offset + 2];
                        mWiimoteState.NunchukState.AccelState.RawValues6b.Y = buff[offset + 3];
                        mWiimoteState.NunchukState.AccelState.RawValues6b.Z = buff[offset + 4];

                         //    output.nunchuck.x = (extension_data[2] << 2) | (extension_data[5] & 0x0C) >> 2;
                    //    output.nunchuck.y = (extension_data[3] << 2) | (extension_data[5] & 0x30) >> 4;
                    //    output.nunchuck.z = (extension_data[4] << 2) | (extension_data[5] & 0xC0) >> 6;



                         //10bit precision but calibration data is 8bit
                         mWiimoteState.NunchukState.AccelState.RawValues8b.X = (buff[offset + 2]<<2) | (buff[offset + 5] & 0x0C) >> 2;
                        mWiimoteState.NunchukState.AccelState.RawValues8b.Y = (buff[offset + 3] << 2)| (buff[offset+5] & 0x30) >> 4;;
                        mWiimoteState.NunchukState.AccelState.RawValues8b.Z = (buff[offset + 4]<< 2)| (buff[offset + 5] & 0xC0) >> 6;

                        mWiimoteState.NunchukState.C = (buff[offset + 5] & 0x02) == 0;
                        mWiimoteState.NunchukState.Z = (buff[offset + 5] & 0x01) == 0;
                    }



                   // float range=mWiimoteState.NunchukState.CalibrationInfo.AccelCalibration.XG<<2 - mWiimoteState.NunchukState.CalibrationInfo.AccelCalibration.X0<<2;
                  //  mWiimoteState.NunchukState.AccelState.Values.X = ((float)(mWiimoteState.NunchukState.AccelState.RawValues.X - mWiimoteState.NunchukState.CalibrationInfo.AccelCalibration.X0<<2)) /range;
                                                    
				    //orgininal 
					mWiimoteState.NunchukState.AccelState.Values.X = (float)((float)mWiimoteState.NunchukState.AccelState.RawValues6b.X - mWiimoteState.NunchukState.CalibrationInfo.AccelCalibration.X0) / 
													((float)mWiimoteState.NunchukState.CalibrationInfo.AccelCalibration.XG - mWiimoteState.NunchukState.CalibrationInfo.AccelCalibration.X0);
					mWiimoteState.NunchukState.AccelState.Values.Y = (float)((float)mWiimoteState.NunchukState.AccelState.RawValues6b.Y - mWiimoteState.NunchukState.CalibrationInfo.AccelCalibration.Y0) /
													((float)mWiimoteState.NunchukState.CalibrationInfo.AccelCalibration.YG - mWiimoteState.NunchukState.CalibrationInfo.AccelCalibration.Y0);
					mWiimoteState.NunchukState.AccelState.Values.Z = (float)((float)mWiimoteState.NunchukState.AccelState.RawValues6b.Z - mWiimoteState.NunchukState.CalibrationInfo.AccelCalibration.Z0) /
													((float)mWiimoteState.NunchukState.CalibrationInfo.AccelCalibration.ZG - mWiimoteState.NunchukState.CalibrationInfo.AccelCalibration.Z0);


                    //seem like joystick calibration is done without of restricting nunchuk hull
					if(mWiimoteState.NunchukState.CalibrationInfo.MaxX != 0x00)
                        mWiimoteState.NunchukState.Joystick.X = 2f*(mWiimoteState.NunchukState.RawJoystick.X - mWiimoteState.NunchukState.CalibrationInfo.MidX) /
                                                ((float)mWiimoteState.NunchukState.CalibrationInfo.MaxX - mWiimoteState.NunchukState.CalibrationInfo.MinX);

					
					if(mWiimoteState.NunchukState.CalibrationInfo.MaxY != 0x00)
						mWiimoteState.NunchukState.Joystick.Y = 2f*(mWiimoteState.NunchukState.RawJoystick.Y - mWiimoteState.NunchukState.CalibrationInfo.MidY) / 
												((float)mWiimoteState.NunchukState.CalibrationInfo.MaxY - mWiimoteState.NunchukState.CalibrationInfo.MinY);

//void FillRawAccelFromGForceData(wm_accel& raw_accel,
//    const accel_cal& calib,
//    const WiimoteEmu::AccelData& accel)
//{
//    raw_accel.x = (u8)trim(accel.x * (calib.one_g.x - calib.zero_g.x) + calib.zero_g.x);
//    raw_accel.y = (u8)trim(accel.y * (calib.one_g.y - calib.zero_g.y) + calib.zero_g.y);
//    raw_accel.z = (u8)trim(accel.z * (calib.one_g.z - calib.zero_g.z) + calib.zero_g.z);
//}


                }

				 if  ((mWiimoteState.Extension & (byte)ExtensionType.ClassicController) !=0){
                    //if (extension_data.size() >= 6)
                    //{
                    //    output.valid_data_flags |= dolphiimote_CLASSIC_CONTROLLER_VALID;

                    //    output.classic_controller.left_stick_x = extension_data[0] & 0x3F;
                    //    output.classic_controller.left_stick_y = extension_data[1] & 0x3F;

                    //    output.classic_controller.right_stick_x = ((extension_data[0] & 0xC0) >> 3) | ((extension_data[1] & 0xC0) >> 5) | ((extension_data[2] & 0xC0) >> 7);
                    //    output.classic_controller.right_stick_y = extension_data[2] & 0x1F;

                    //    output.classic_controller.left_trigger = ((extension_data[2] & 0x60) >> 2) | ((extension_data[3] & 0xE0) >> 5);
                    //    output.classic_controller.right_trigger = extension_data[3] & 0x1F;

                    //    output.classic_controller.buttons = ~((extension_data[4] << 8) | extension_data[5]);
                    //}



                  

                     if(PASS_THRU_MODE == PassThruMode.ClassicController){

                       //interleave mode    
                    //if (extension_data.size() >= 6)
                    //{
                    //    if ((extension_data[5] & 0x03) == 0)
                    //    {
                    //        output.valid_data_flags |= dolphiimote_CLASSIC_CONTROLLER_VALID;

                    //        output.classic_controller.left_stick_x = extension_data[0] & 0x3E;
                    //        output.classic_controller.left_stick_y = extension_data[1] & 0x3E;

                    //        output.classic_controller.right_stick_x = ((extension_data[0] & 0xC0) >> 3) | ((extension_data[1] & 0xC0) >> 5) | ((extension_data[2] & 0xC0) >> 7);
                    //        output.classic_controller.right_stick_y = extension_data[2] & 0x1F;

                    //        output.classic_controller.left_trigger = ((extension_data[2] & 0x60) >> 2) | ((extension_data[3] & 0xE0) >> 5);
                    //        output.classic_controller.right_trigger = extension_data[3] & 0x1F;

                    //        output.classic_controller.buttons = ~(((extension_data[4] & 0xFE) << 8) | (extension_data[5] & 0xFC) | ((extension_data[1] & 0x01) << 1) | (extension_data[0] & 0x01));
                    //    }
                    //}

                         if ((buff[offset + 5] & 0x03) == 0)
                         {
                             mWiimoteState.ClassicControllerState.RawJoystickL.X = (byte)(buff[offset] & 0x3E);
                             mWiimoteState.ClassicControllerState.RawJoystickL.Y = (byte)(buff[offset + 1] & 0x3E);

                             mWiimoteState.ClassicControllerState.RawJoystickR.X = (byte)((buff[offset] & 0xC0) >> 3) | ((buff[offset + 1] & 0xC0) >> 5) | ((buff[offset + 2] & 0xC0) >> 7);
                             mWiimoteState.ClassicControllerState.RawJoystickR.Y = (byte)(buff[offset+2] & 0x1F);

                             mWiimoteState.ClassicControllerState.RawTriggerL = (byte)(((buff[offset+2] & 0x60) >> 2) | ((buff[offset+3] & 0xE0) >> 5));
                             mWiimoteState.ClassicControllerState.RawTriggerR = (byte)(buff[offset + 3] & 0x1f);


                             //TODO CHECK if buttons are the same as in normal mode
                             mWiimoteState.ClassicControllerState.ButtonState.TriggerR = (buff[offset + 4] & 0x02) == 0;
                             mWiimoteState.ClassicControllerState.ButtonState.Plus = (buff[offset + 4] & 0x04) == 0;
                             mWiimoteState.ClassicControllerState.ButtonState.Home = (buff[offset + 4] & 0x08) == 0;
                             mWiimoteState.ClassicControllerState.ButtonState.Minus = (buff[offset + 4] & 0x10) == 0;
                             mWiimoteState.ClassicControllerState.ButtonState.TriggerL = (buff[offset + 4] & 0x20) == 0;
                             mWiimoteState.ClassicControllerState.ButtonState.Down = (buff[offset + 4] & 0x40) == 0;
                             mWiimoteState.ClassicControllerState.ButtonState.Right = (buff[offset + 4] & 0x80) == 0;

                             mWiimoteState.ClassicControllerState.ButtonState.Up = (buff[offset + 5] & 0x01) == 0;
                             mWiimoteState.ClassicControllerState.ButtonState.Left = (buff[offset + 5] & 0x02) == 0;
                             mWiimoteState.ClassicControllerState.ButtonState.ZR = (buff[offset + 5] & 0x04) == 0;
                             mWiimoteState.ClassicControllerState.ButtonState.X = (buff[offset + 5] & 0x08) == 0;
                             mWiimoteState.ClassicControllerState.ButtonState.A = (buff[offset + 5] & 0x10) == 0;
                             mWiimoteState.ClassicControllerState.ButtonState.Y = (buff[offset + 5] & 0x20) == 0;
                             mWiimoteState.ClassicControllerState.ButtonState.B = (buff[offset + 5] & 0x40) == 0;
                             mWiimoteState.ClassicControllerState.ButtonState.ZL = (buff[offset + 5] & 0x80) == 0;
                         }
                         else return;
                     }
                     else 
                    {

                        mWiimoteState.ClassicControllerState.RawJoystickL.X = (byte)(buff[offset] & 0x3f);
                        mWiimoteState.ClassicControllerState.RawJoystickL.Y = (byte)(buff[offset + 1] & 0x3f);
                        mWiimoteState.ClassicControllerState.RawJoystickR.X = (byte)((buff[offset + 2] >> 7) | (buff[offset + 1] & 0xc0) >> 5 | (buff[offset] & 0xc0) >> 3);
                        mWiimoteState.ClassicControllerState.RawJoystickR.Y = (byte)(buff[offset + 2] & 0x1f);

                        mWiimoteState.ClassicControllerState.RawTriggerL = (byte)(((buff[offset + 2] & 0x60) >> 2) | (buff[offset + 3] >> 5));
                        mWiimoteState.ClassicControllerState.RawTriggerR = (byte)(buff[offset + 3] & 0x1f);

                        mWiimoteState.ClassicControllerState.ButtonState.TriggerR = (buff[offset + 4] & 0x02) == 0;
                        mWiimoteState.ClassicControllerState.ButtonState.Plus = (buff[offset + 4] & 0x04) == 0;
                        mWiimoteState.ClassicControllerState.ButtonState.Home = (buff[offset + 4] & 0x08) == 0;
                        mWiimoteState.ClassicControllerState.ButtonState.Minus = (buff[offset + 4] & 0x10) == 0;
                        mWiimoteState.ClassicControllerState.ButtonState.TriggerL = (buff[offset + 4] & 0x20) == 0;
                        mWiimoteState.ClassicControllerState.ButtonState.Down = (buff[offset + 4] & 0x40) == 0;
                        mWiimoteState.ClassicControllerState.ButtonState.Right = (buff[offset + 4] & 0x80) == 0;

                        mWiimoteState.ClassicControllerState.ButtonState.Up = (buff[offset + 5] & 0x01) == 0;
                        mWiimoteState.ClassicControllerState.ButtonState.Left = (buff[offset + 5] & 0x02) == 0;
                        mWiimoteState.ClassicControllerState.ButtonState.ZR = (buff[offset + 5] & 0x04) == 0;
                        mWiimoteState.ClassicControllerState.ButtonState.X = (buff[offset + 5] & 0x08) == 0;
                        mWiimoteState.ClassicControllerState.ButtonState.A = (buff[offset + 5] & 0x10) == 0;
                        mWiimoteState.ClassicControllerState.ButtonState.Y = (buff[offset + 5] & 0x20) == 0;
                        mWiimoteState.ClassicControllerState.ButtonState.B = (buff[offset + 5] & 0x40) == 0;
                        mWiimoteState.ClassicControllerState.ButtonState.ZL = (buff[offset + 5] & 0x80) == 0;
                    }



					if(mWiimoteState.ClassicControllerState.CalibrationInfo.MaxXL != 0x00)
						mWiimoteState.ClassicControllerState.JoystickL.X = (float)((float)mWiimoteState.ClassicControllerState.RawJoystickL.X - mWiimoteState.ClassicControllerState.CalibrationInfo.MidXL) / 
						(float)(mWiimoteState.ClassicControllerState.CalibrationInfo.MaxXL - mWiimoteState.ClassicControllerState.CalibrationInfo.MinXL);

					if(mWiimoteState.ClassicControllerState.CalibrationInfo.MaxYL != 0x00)
						mWiimoteState.ClassicControllerState.JoystickL.Y = (float)((float)mWiimoteState.ClassicControllerState.RawJoystickL.Y - mWiimoteState.ClassicControllerState.CalibrationInfo.MidYL) / 
						(float)(mWiimoteState.ClassicControllerState.CalibrationInfo.MaxYL - mWiimoteState.ClassicControllerState.CalibrationInfo.MinYL);

					if(mWiimoteState.ClassicControllerState.CalibrationInfo.MaxXR != 0x00)
						mWiimoteState.ClassicControllerState.JoystickR.X = (float)((float)mWiimoteState.ClassicControllerState.RawJoystickR.X - mWiimoteState.ClassicControllerState.CalibrationInfo.MidXR) / 
						(float)(mWiimoteState.ClassicControllerState.CalibrationInfo.MaxXR - mWiimoteState.ClassicControllerState.CalibrationInfo.MinXR);

					if(mWiimoteState.ClassicControllerState.CalibrationInfo.MaxYR != 0x00)
						mWiimoteState.ClassicControllerState.JoystickR.Y = (float)((float)mWiimoteState.ClassicControllerState.RawJoystickR.Y - mWiimoteState.ClassicControllerState.CalibrationInfo.MidYR) / 
						(float)(mWiimoteState.ClassicControllerState.CalibrationInfo.MaxYR - mWiimoteState.ClassicControllerState.CalibrationInfo.MinYR);

					if(mWiimoteState.ClassicControllerState.CalibrationInfo.MaxTriggerL != 0x00)
						mWiimoteState.ClassicControllerState.TriggerL = (mWiimoteState.ClassicControllerState.RawTriggerL) / 
						(float)(mWiimoteState.ClassicControllerState.CalibrationInfo.MaxTriggerL - mWiimoteState.ClassicControllerState.CalibrationInfo.MinTriggerL);

					if(mWiimoteState.ClassicControllerState.CalibrationInfo.MaxTriggerR != 0x00)
						mWiimoteState.ClassicControllerState.TriggerR = (mWiimoteState.ClassicControllerState.RawTriggerR) / 
						(float)(mWiimoteState.ClassicControllerState.CalibrationInfo.MaxTriggerR - mWiimoteState.ClassicControllerState.CalibrationInfo.MinTriggerR);
                 }

                
				 if  ((mWiimoteState.Extension & (byte)ExtensionType.Guitar) !=0){
					mWiimoteState.GuitarState.GuitarType = ((buff[offset] & 0x80) == 0) ? GuitarType.GuitarHeroWorldTour : GuitarType.GuitarHero3;

					mWiimoteState.GuitarState.ButtonState.Plus		= (buff[offset + 4] & 0x04) == 0;
					mWiimoteState.GuitarState.ButtonState.Minus		= (buff[offset + 4] & 0x10) == 0;
					mWiimoteState.GuitarState.ButtonState.StrumDown	= (buff[offset + 4] & 0x40) == 0;

					mWiimoteState.GuitarState.ButtonState.StrumUp		= (buff[offset + 5] & 0x01) == 0;
					mWiimoteState.GuitarState.FretButtonState.Yellow	= (buff[offset + 5] & 0x08) == 0;
					mWiimoteState.GuitarState.FretButtonState.Green		= (buff[offset + 5] & 0x10) == 0;
					mWiimoteState.GuitarState.FretButtonState.Blue		= (buff[offset + 5] & 0x20) == 0;
					mWiimoteState.GuitarState.FretButtonState.Red		= (buff[offset + 5] & 0x40) == 0;
					mWiimoteState.GuitarState.FretButtonState.Orange	= (buff[offset + 5] & 0x80) == 0;

					// it appears the joystick values are only 6 bits
					mWiimoteState.GuitarState.RawJoystick.X	= (buff[offset + 0] & 0x3f);
					mWiimoteState.GuitarState.RawJoystick.Y	= (buff[offset + 1] & 0x3f);

					// and the whammy bar is only 5 bits
					mWiimoteState.GuitarState.RawWhammyBar			= (byte)(buff[offset + 3] & 0x1f);

					mWiimoteState.GuitarState.Joystick.X			= (float)(mWiimoteState.GuitarState.RawJoystick.X - 0x1f) / 0x3f;	// not fully accurate, but close
					mWiimoteState.GuitarState.Joystick.Y			= (float)(mWiimoteState.GuitarState.RawJoystick.Y - 0x1f) / 0x3f;	// not fully accurate, but close
					mWiimoteState.GuitarState.WhammyBar				= (float)(mWiimoteState.GuitarState.RawWhammyBar) / 0x0a;	// seems like there are 10 positions?

					mWiimoteState.GuitarState.TouchbarState.Yellow	= false;
					mWiimoteState.GuitarState.TouchbarState.Green	= false;
					mWiimoteState.GuitarState.TouchbarState.Blue	= false;
					mWiimoteState.GuitarState.TouchbarState.Red		= false;
					mWiimoteState.GuitarState.TouchbarState.Orange	= false;

					switch(buff[offset + 2] & 0x1f)
					{
						case 0x04:
							mWiimoteState.GuitarState.TouchbarState.Green = true;
							break;
						case 0x07:
							mWiimoteState.GuitarState.TouchbarState.Green = true;
							mWiimoteState.GuitarState.TouchbarState.Red = true;
							break;
						case 0x0a:
							mWiimoteState.GuitarState.TouchbarState.Red = true;
							break;
						case 0x0c:
						case 0x0d:
							mWiimoteState.GuitarState.TouchbarState.Red = true;
							mWiimoteState.GuitarState.TouchbarState.Yellow = true;
							break;
						case 0x12:
						case 0x13:
							mWiimoteState.GuitarState.TouchbarState.Yellow = true;
							break;
						case 0x14:
						case 0x15:
							mWiimoteState.GuitarState.TouchbarState.Yellow = true;
							mWiimoteState.GuitarState.TouchbarState.Blue = true;
							break;
						case 0x17:
						case 0x18:
							mWiimoteState.GuitarState.TouchbarState.Blue = true;
							break;
						case 0x1a:
							mWiimoteState.GuitarState.TouchbarState.Blue = true;
							mWiimoteState.GuitarState.TouchbarState.Orange = true;
							break;
						case 0x1f:
							mWiimoteState.GuitarState.TouchbarState.Orange = true;
							break;
					}
                 }

				 if  ((mWiimoteState.Extension & (byte)ExtensionType.Drums) !=0){
					// it appears the joystick values are only 6 bits
					mWiimoteState.DrumsState.RawJoystick.X	= (buff[offset + 0] & 0x3f);
					mWiimoteState.DrumsState.RawJoystick.Y	= (buff[offset + 1] & 0x3f);

					mWiimoteState.DrumsState.Plus			= (buff[offset + 4] & 0x04) == 0;
					mWiimoteState.DrumsState.Minus			= (buff[offset + 4] & 0x10) == 0;

					mWiimoteState.DrumsState.Pedal			= (buff[offset + 5] & 0x04) == 0;
					mWiimoteState.DrumsState.Blue			= (buff[offset + 5] & 0x08) == 0;
					mWiimoteState.DrumsState.Green			= (buff[offset + 5] & 0x10) == 0;
					mWiimoteState.DrumsState.Yellow			= (buff[offset + 5] & 0x20) == 0;
					mWiimoteState.DrumsState.Red			= (buff[offset + 5] & 0x40) == 0;
					mWiimoteState.DrumsState.Orange			= (buff[offset + 5] & 0x80) == 0;

					mWiimoteState.DrumsState.Joystick.X		= (float)(mWiimoteState.DrumsState.RawJoystick.X - 0x1f) / 0x3f;	// not fully accurate, but close
					mWiimoteState.DrumsState.Joystick.Y		= (float)(mWiimoteState.DrumsState.RawJoystick.Y - 0x1f) / 0x3f;	// not fully accurate, but close

					if((buff[offset + 2] & 0x40) == 0)
					{
						int pad = (buff[offset + 2] >> 1) & 0x1f;
						int velocity = (buff[offset + 3] >> 5);

						if(velocity != 7)
						{
							switch(pad)
							{
								case 0x1b:
									mWiimoteState.DrumsState.PedalVelocity = velocity;
									break;
								case 0x19:
									mWiimoteState.DrumsState.RedVelocity = velocity;
									break;
								case 0x11:
									mWiimoteState.DrumsState.YellowVelocity = velocity;
									break;
								case 0x0f:
									mWiimoteState.DrumsState.BlueVelocity = velocity;
									break;
								case 0x0e:
									mWiimoteState.DrumsState.OrangeVelocity = velocity;
									break;
								case 0x12:
									mWiimoteState.DrumsState.GreenVelocity = velocity;
									break;
							}
						}
					}

                 }

				 if  ((mWiimoteState.Extension & (byte)ExtensionType.BalancedBoard) !=0){
					mWiimoteState.BalanceBoardState.SensorValuesRaw.TopRight = (short)((short)buff[offset + 0] << 8 | buff[offset + 1]);
					mWiimoteState.BalanceBoardState.SensorValuesRaw.BottomRight = (short)((short)buff[offset + 2] << 8 | buff[offset + 3]);
					mWiimoteState.BalanceBoardState.SensorValuesRaw.TopLeft = (short)((short)buff[offset + 4] << 8 | buff[offset + 5]);
					mWiimoteState.BalanceBoardState.SensorValuesRaw.BottomLeft = (short)((short)buff[offset + 6] << 8 | buff[offset + 7]);

					mWiimoteState.BalanceBoardState.SensorValuesKg.TopLeft = GetBalanceBoardSensorValue(mWiimoteState.BalanceBoardState.SensorValuesRaw.TopLeft, mWiimoteState.BalanceBoardState.CalibrationInfo.Kg0.TopLeft, mWiimoteState.BalanceBoardState.CalibrationInfo.Kg17.TopLeft, mWiimoteState.BalanceBoardState.CalibrationInfo.Kg34.TopLeft);
					mWiimoteState.BalanceBoardState.SensorValuesKg.TopRight = GetBalanceBoardSensorValue(mWiimoteState.BalanceBoardState.SensorValuesRaw.TopRight, mWiimoteState.BalanceBoardState.CalibrationInfo.Kg0.TopRight, mWiimoteState.BalanceBoardState.CalibrationInfo.Kg17.TopRight, mWiimoteState.BalanceBoardState.CalibrationInfo.Kg34.TopRight);
					mWiimoteState.BalanceBoardState.SensorValuesKg.BottomLeft = GetBalanceBoardSensorValue(mWiimoteState.BalanceBoardState.SensorValuesRaw.BottomLeft, mWiimoteState.BalanceBoardState.CalibrationInfo.Kg0.BottomLeft, mWiimoteState.BalanceBoardState.CalibrationInfo.Kg17.BottomLeft, mWiimoteState.BalanceBoardState.CalibrationInfo.Kg34.BottomLeft);
					mWiimoteState.BalanceBoardState.SensorValuesKg.BottomRight = GetBalanceBoardSensorValue(mWiimoteState.BalanceBoardState.SensorValuesRaw.BottomRight, mWiimoteState.BalanceBoardState.CalibrationInfo.Kg0.BottomRight, mWiimoteState.BalanceBoardState.CalibrationInfo.Kg17.BottomRight, mWiimoteState.BalanceBoardState.CalibrationInfo.Kg34.BottomRight);

					mWiimoteState.BalanceBoardState.SensorValuesLb.TopLeft = (mWiimoteState.BalanceBoardState.SensorValuesKg.TopLeft * KG2LB);
					mWiimoteState.BalanceBoardState.SensorValuesLb.TopRight = (mWiimoteState.BalanceBoardState.SensorValuesKg.TopRight * KG2LB);
					mWiimoteState.BalanceBoardState.SensorValuesLb.BottomLeft = (mWiimoteState.BalanceBoardState.SensorValuesKg.BottomLeft * KG2LB);
					mWiimoteState.BalanceBoardState.SensorValuesLb.BottomRight = (mWiimoteState.BalanceBoardState.SensorValuesKg.BottomRight * KG2LB);

					mWiimoteState.BalanceBoardState.WeightKg = (mWiimoteState.BalanceBoardState.SensorValuesKg.TopLeft + mWiimoteState.BalanceBoardState.SensorValuesKg.TopRight + mWiimoteState.BalanceBoardState.SensorValuesKg.BottomLeft + mWiimoteState.BalanceBoardState.SensorValuesKg.BottomRight) / 4.0f;
					mWiimoteState.BalanceBoardState.WeightLb = (mWiimoteState.BalanceBoardState.SensorValuesLb.TopLeft + mWiimoteState.BalanceBoardState.SensorValuesLb.TopRight + mWiimoteState.BalanceBoardState.SensorValuesLb.BottomLeft + mWiimoteState.BalanceBoardState.SensorValuesLb.BottomRight) / 4.0f;

					float Kx = (mWiimoteState.BalanceBoardState.SensorValuesKg.TopLeft + mWiimoteState.BalanceBoardState.SensorValuesKg.BottomLeft) / (mWiimoteState.BalanceBoardState.SensorValuesKg.TopRight + mWiimoteState.BalanceBoardState.SensorValuesKg.BottomRight);
					float Ky = (mWiimoteState.BalanceBoardState.SensorValuesKg.TopLeft + mWiimoteState.BalanceBoardState.SensorValuesKg.TopRight) / (mWiimoteState.BalanceBoardState.SensorValuesKg.BottomLeft + mWiimoteState.BalanceBoardState.SensorValuesKg.BottomRight);

					mWiimoteState.BalanceBoardState.CenterOfGravity.X = ((float)(Kx - 1) / (float)(Kx + 1)) * (float)(-BSL / 2);
					mWiimoteState.BalanceBoardState.CenterOfGravity.Y = ((float)(Ky - 1) / (float)(Ky + 1)) * (float)(-BSW / 2);
                 }

				 if  ((mWiimoteState.Extension & (byte)ExtensionType.TaikoDrums) !=0){
					mWiimoteState.TaikoDrumState.OuterLeft  = (buff[offset + 5] & 0x20) == 0;
					mWiimoteState.TaikoDrumState.InnerLeft  = (buff[offset + 5] & 0x40) == 0;
					mWiimoteState.TaikoDrumState.InnerRight = (buff[offset + 5] & 0x10) == 0;
					mWiimoteState.TaikoDrumState.OuterRight = (buff[offset + 5] & 0x08) == 0;
                 }

                 if ((mWiimoteState.Extension & (byte)ExtensionType.MotionPlus) != 0)
                 {
            //            short yaw   = ((unsigned short)buff[offset+3] & 0xFC)<<6 |
            //               (unsigned short)buff[offset+0];
            //short pitch = ((unsigned short)buff[offset+5] & 0xFC)<<6 |
            //               (unsigned short)buff[offset+2];
            //short roll  = ((unsigned short)buff[offset+4] & 0xFC)<<6 |
            //               (unsigned short)buff[offset+1];

            //// we get one set of bogus values when the MotionPlus is disconnected,
            ////  so ignore them
            //if((yaw != 0x3fff) || (pitch != 0x3fff) || (roll != 0x3fff))
            //    {
            //    wiimote_state::motion_plus::sensors_raw &raw = Internal.MotionPlus.Raw;
	
            //    if((raw.Yaw != yaw) || (raw.Pitch != pitch) || (raw.Roll  != roll))
            //        changed |= MOTIONPLUS_SPEED_CHANGED;

            //    raw.Yaw   = yaw;
            //    raw.Pitch = pitch;
            //    raw.Roll  = roll;
	
            //    // convert to float values
            //    bool    yaw_slow = (buff[offset+3] & 0x2) == 0x2;
            //    bool  pitch_slow = (buff[offset+3] & 0x1) == 0x1;
            //    bool   roll_slow = (buff[offset+4] & 0x2) == 0x2;
            //    float y_scale    =   yaw_slow? 0.05f : 0.25f;
            //    float p_scale    = pitch_slow? 0.05f : 0.25f;
            //    float r_scale    =  roll_slow? 0.05f : 0.25f;

//                    While the Wiimote is still, the values will be about 0x1F7F (8,063), although it is best to calibrate for a few seconds every time you start, to get the actual zero values.
//Voltage reference is 1.35V that you can assume it as 8192 unit (half of the ADC range), using 2.27 mV/deg/s, 8192 is 595 deg/s (1.35V/2.27mV), you must divide by ~13.768 unit/deg/s (8192/595) to know the correct deg/s.
//At high speed (slow bit = 0) raw values read are small with the same deg/s to reach higher values on top, so you must multiply it by 2000/440 (they are the max reference in the two modes in deg/s [1]). Example: reading 8083 raw value and assuming 8063 as zero, 20 unit in slow/normal mode are 1,45 deg/s and in fast mode are 1.45*2000/440=6.59 deg/s.
//Yaw/Roll/Pitch fast/slow bits are 1 when the Wii Remote is rotating slowly (or not rotating at all), 0 when the Wii Remote is rotating fast.
//Extension connected is 1 when an extension is connected to the MotionPlus extension port.


    //                u8 speed_mask = ~0x03;

    //if(extension_data.size() >= 6 && (extension_data[5] & 0x02))
    //{
    //  output.valid_data_flags |= dolphiimote_MOTIONPLUS_VALID;

    //  output.motionplus.yaw_down_speed = extension_data[0] + ((u16)(extension_data[3] & speed_mask) << 6);
    //  output.motionplus.roll_left_speed = extension_data[1] + ((u16)(extension_data[4] & speed_mask) << 6);
    //  output.motionplus.pitch_left_speed = extension_data[2] + ((u16)(extension_data[5] & speed_mask) << 6);
                   // YPR
    //  output.motionplus.slow_modes = (extension_data[3] & 0x03) << 1 | (extension_data[4] & 0x02) >> 1;
    //  output.motionplus.extension_connected = extension_data[4] & 0x01;
    //}


                     //const double fastModeFactor = 2000.0 / 440.0; //According to wiibrew
          //  const double fastModeFactor = 20.0 / 4.0; //According to wiic
          //  var gyro = calibration.NormalizeMotionplus(DateTime.Now, motionplus.yaw_down_speed,
                                                    //                 motionplus.pitch_left_speed,
                                                   //                  motionplus.roll_left_speed);

          //  return new CalibratedValue<Gyro>(gyro.DidCalibrate, new Gyro((motionplus.slow_modes & 0x1) == 0x1 ? gyro.Value.x : gyro.Value.x * fastModeFactor,
                                                     //                    (motionplus.slow_modes & 0x4) == 0x4 ? gyro.Value.y : gyro.Value.y * fastModeFactor,
                                                                          // (motionplus.slow_modes & 0x2) == 0x2 ? gyro.Value.z : gyro.Value.z * fastModeFactor));
                            //double offest=8192.0;
                            //const double gain = 1.0 / 20.0;
                            // (value - offest) *gain;

                    //Epson 3600CB 0.67mV/deg/s  +/- 100 deg/s

                  //  const double fastModeFactor = 2000.0 / 440.0; //According to wiibrew
                    const float fastModeFactor =5; //20f / 4f; //According to wiic
                      Point3 unitsAtZeroDeg=new Point3();
                      unitsAtZeroDeg.Y = 8192;// 8323;// 8192;//.0;
                      unitsAtZeroDeg.X = 8192;//8173;//8220;//.0;
                      unitsAtZeroDeg.Z = 8192;// 8846;// 8233;//.0;
                     const float gain =0.05f;// 1f / 20f;//

                    //fast
                   // (N-8192)/16384 * 3.0V * (1 deg/s)/.0005V * (Pi radians)/(180 deg)

                    //slow
                     // (N-8192)/16384 * 3.0V * (1 deg/s)/.002V * (Pi radians)/(180 deg)

                     const float slowFactor = 500 / 8192f;//deg/unit  0.05f;//   500/8192f;//deg/unit
                     const float fastFactor = 2000 / 8192f;//deg/unit 0.25f;// 2000/8192f ;//deg/unit
                            // (value - offest) *gain;
                    //  short yaw   = ((unsigned short)buff[offset+3] & 0xFC)<<6 |
            //               (unsigned short)buff[offset+0];
                    
                         mWiimoteState.MotionPlusState.YawFast = ((buff[offset + 3] & 0x02) >> 1) == 0;
                         mWiimoteState.MotionPlusState.PitchFast = ((buff[offset + 3] & 0x01) >> 0) == 0;
                         mWiimoteState.MotionPlusState.RollFast = ((buff[offset + 4] & 0x02) >> 1) == 0;


                         //// (may not be optimal)
                         //float pitch = -asin(y) * 57.2957795f;
                         ////		float roll  =  asin(x)    * 57.2957795f;
                         //float roll = atan2(x, z) * 57.2957795f;

                         //short yaw   = ((unsigned short)buff[offset+3] & 0xFC)<<6 |
                         //               (unsigned short)buff[offset+0];
                         //short pitch = ((unsigned short)buff[offset+5] & 0xFC)<<6 |
                         //               (unsigned short)buff[offset+2];
                         //short roll  = ((unsigned short)buff[offset+4] & 0xFC)<<6 |
                         //               (unsigned short)buff[offset+1];

                         mPrevAngleRates.X = mWiimoteState.MotionPlusState.RawValues.X;
                         mPrevAngleRates.Y = mWiimoteState.MotionPlusState.RawValues.Y;
                         mPrevAngleRates.Z = mWiimoteState.MotionPlusState.RawValues.Z;

                         mWiimoteState.MotionPlusState.RawValues.Z = (buff[offset + 0] | (buff[offset + 3] & 0xfc) << 6);//YAW
                         mWiimoteState.MotionPlusState.RawValues.Y = (buff[offset + 1] | (buff[offset + 4] & 0xfc) << 6);//ROLL
                         mWiimoteState.MotionPlusState.RawValues.X = (buff[offset + 2] | (buff[offset + 5] & 0xfc) << 6);//PITCH
                     

                    
                    

                    //mask should be 0xfc(1111 1100) not fa 
                   // mWiimoteState.MotionPlusState.RawValues.X = (buff[offset + 0] | (buff[offset + 3] & 0xfa) << 6);
                  //  mWiimoteState.MotionPlusState.RawValues.Y = (buff[offset + 1] | (buff[offset + 4] & 0xfa) << 6);
                  //  mWiimoteState.MotionPlusState.RawValues.Z = (buff[offset + 2] | (buff[offset + 5] & 0xfa) << 6);


                         if (!IsMotionPlusCalibrating())
                         {
                             if (!IsMotionPlusCalibrated())
                             {
                                 InitCalibration(mWiimoteState.MotionPlusState.RawValues.X, mWiimoteState.MotionPlusState.RawValues.Y, mWiimoteState.MotionPlusState.RawValues.Z);
                                 return;
                             }
                         }
                         else
                         {

                             UpdateCalibration(mCalibrationStopWatch.ElapsedMilliseconds, mWiimoteState.MotionPlusState.RawValues.X, mWiimoteState.MotionPlusState.RawValues.Y, mWiimoteState.MotionPlusState.RawValues.Z);
                             return;
                         }


                         unitsAtZeroDeg.X = (int)mBias.X;
                         unitsAtZeroDeg.Y = (int)mBias.Y;
                         unitsAtZeroDeg.Z = (int)mBias.Z;

                        //float diff;
                        // diff = mWiimoteState.MotionPlusState.RawValues.X - mPrevAngleRates.X;

                        // if ((diff > 0 && (mWiimoteState.MotionPlusState.RawValues.X < (mMaxNoise.X - mBias.X) + mPrevAngleRates.X)) || (diff < 0 && (mWiimoteState.MotionPlusState.RawValues.X > mPrevAngleRates.X - (mBias.X - mMinNoise.X))))
                        // {
                        //     mNotHaveSubstaintialValue.X++;

                        //     if (mNotHaveSubstaintialValue.X > 4)
                        //     {
                        //         mWiimoteState.MotionPlusState.RawValues.X = (int)mBias.X;
                        //         mNotHaveSubstaintialValue.X = 0;
                        //     }
                        //     else
                        //         mWiimoteState.MotionPlusState.RawValues.X = (int)mPrevAngleRates.X;
                        // }
                        // else
                        // {
                           
                        //     mNotHaveSubstaintialValue.X = 0;
                        // }


                        // diff = mWiimoteState.MotionPlusState.RawValues.Y - mPrevAngleRates.Y;

                        // if ((diff > 0 && (mWiimoteState.MotionPlusState.RawValues.Y < (mMaxNoise.Y - mBias.Y) + mPrevAngleRates.Y)) || (diff < 0 && (mWiimoteState.MotionPlusState.RawValues.Y > mPrevAngleRates.Y - (mBias.Y - mMinNoise.Y))))
                        // {
                        //     mNotHaveSubstaintialValue.Y++;

                        //     if (mNotHaveSubstaintialValue.Y > 4)
                        //     {
                        //         mWiimoteState.MotionPlusState.RawValues.Y = (int)mBias.Y;
                        //         mNotHaveSubstaintialValue.Y = 0;
                        //     }
                        //     else
                        //         mWiimoteState.MotionPlusState.RawValues.Y = (int)mPrevAngleRates.Y;
                        // }
                        // else
                        // {

                        //     mNotHaveSubstaintialValue.Y = 0;
                        // }


                         //diff = mWiimoteState.MotionPlusState.RawValues.Y - mPrevAngleRates.Y;
                         //if ((diff > 0 && diff < (mMaxNoise.Y - mBias.Y)) || (diff < 0 && diff > (mMinNoise.Y - mBias.Y)))
                         //{
                         //    mWiimoteState.MotionPlusState.RawValues.Y = (int)mPrevAngleRates.Y;
                         //}

                         //diff = mWiimoteState.MotionPlusState.RawValues.Z - mPrevAngleRates.Z;
                         //if ((diff > 0 && diff < (mMaxNoise.Z - mBias.Z)) || (diff < 0 && diff > (mMinNoise.Z - mBias.Z)))
                         //{
                         //    mWiimoteState.MotionPlusState.RawValues.Z = (int)mPrevAngleRates.Z;
                         //}



                        


                    //alex winx deg/s
                    mWiimoteState.MotionPlusState.Values.X = mWiimoteState.MotionPlusState.PitchFast ? (mWiimoteState.MotionPlusState.RawValues.X - unitsAtZeroDeg.X) * fastFactor : (mWiimoteState.MotionPlusState.RawValues.X - unitsAtZeroDeg.X) * slowFactor;
                    mWiimoteState.MotionPlusState.Values.Y = mWiimoteState.MotionPlusState.RollFast ? (mWiimoteState.MotionPlusState.RawValues.Y - unitsAtZeroDeg.Y) * fastFactor : (mWiimoteState.MotionPlusState.RawValues.Y - unitsAtZeroDeg.Y) * slowFactor;
                    mWiimoteState.MotionPlusState.Values.Z = mWiimoteState.MotionPlusState.YawFast ? (mWiimoteState.MotionPlusState.RawValues.Z - unitsAtZeroDeg.Z) * fastFactor : (mWiimoteState.MotionPlusState.RawValues.Z - unitsAtZeroDeg.Z) * slowFactor;


                    //if (mWiimoteState.MotionPlusState.Values.X > -5 && mWiimoteState.MotionPlusState.Values.X < 5)
                    //{
                    //    mWiimoteState.MotionPlusState.Values.X = 0;
                    //}

                    //if (mWiimoteState.MotionPlusState.Values.Y > -5 && mWiimoteState.MotionPlusState.Values.Y < 5)
                    //{
                    //    mWiimoteState.MotionPlusState.Values.Y = 0;
                    //}




                    //if (mCalibrationStopWatch.IsRunning)
                    //{
                    //    mCalibrationStopWatch.Start();
                    //}

                     //diff=mWiimoteState.MotionPlusState.Values.X-mPrevAngles.X;

                     ////Debug.WriteLine("Callculated:"+mWiimoteState.MotionPlusState.Values.X);

                     //float correction=0f;
                     //correction = (float)(0.075502316 * Math.Sqrt(mCalibrationStopWatch.ElapsedMilliseconds * 0.001));
                     //if (mWiimoteState.MotionPlusState.Values.X > 0)
                     //{
                     //    mWiimoteState.MotionPlusState.Values.X -= correction;
                     //}
                     //else if(mWiimoteState.MotionPlusState.Values.X < 0)
                     //{
                     //    mWiimoteState.MotionPlusState.Values.X += correction;
                     //}

                     //if (diff > 0)
                     //{
                     //  correction=(float)(0.075502316* Math.Sqrt(mCalibrationStopWatch.ElapsedMilliseconds*0.001));
                     //  mWiimoteState.MotionPlusState.Values.X -= correction;
                     //}else if(diff<0){
                     //    correction = (float)(0.075502316 * Math.Sqrt(mCalibrationStopWatch.ElapsedMilliseconds * 0.001));
                     //    mWiimoteState.MotionPlusState.Values.X += correction;
                     //}
                   //  Debug.WriteLine("5.0000000e-002 "+String.Format("0:0.000000");//mWiimoteState.MotionPlusState.Values.X 
                   // Debug.WriteLine("5.0000000e-002 "+mWiimoteState.MotionPlusState.Values.X);
                    //Debug.WriteLine( mWiimoteState.MotionPlusState.Values.X);

                    // Debug.WriteLine("Correction:" + correction);


                   //  Debug.WriteLine("Corrected:" + mWiimoteState.MotionPlusState.Values.X);

                   //  Debug.WriteLine("-----------------------:") ;

                    mPrevAngles.X = mWiimoteState.MotionPlusState.Values.X;
                    mPrevAngles.Y = mWiimoteState.MotionPlusState.Values.Y;
                    mPrevAngles.Z = mWiimoteState.MotionPlusState.Values.Z;


                  //  Debug.WriteLine(mWiimoteState.MotionPlusState.Values.X);
                  
                 // To find error in orientation due to gyro white noise multiply ARW by the square root of the integration time (t).
                     //ARWX=0.075502316;
                   // Error = ARW *sqrt( t)




                     //if(!IsMotionPlusCalibrating() ){
                     //    if(!IsMotionPlusCalibrated())
                     //     InitCalibration(mWiimoteState.MotionPlusState.Values);
                     //}else {

                     //     UpdateCalibration(mCalibrationStopWatch.ElapsedMilliseconds,mWiimoteState.MotionPlusState.Values);
                     //    return;
                     //}


                      //mWiimoteState.MotionPlusState.Values.X -= mBias.X;
                      //mWiimoteState.MotionPlusState.Values.Y -= mBias.Y;
                      //mWiimoteState.MotionPlusState.Values.Z -= mBias.Z;



                    //  gyroFilter.removeOffset(ref mWiimoteState.MotionPlusState.Values.X, ref mWiimoteState.MotionPlusState.Values.Y, ref mWiimoteState.MotionPlusState.Values.Z);


                    //alex winx deg/s
                    mWiimoteState.MotionPlusState.Values2.X = mWiimoteState.MotionPlusState.PitchFast ? 2*2000f * (mWiimoteState.MotionPlusState.RawValues.X - unitsAtZeroDeg.X)/16384f  :  (2*500f * (mWiimoteState.MotionPlusState.RawValues.X - unitsAtZeroDeg.X))/16384f ;
                    mWiimoteState.MotionPlusState.Values2.Y = mWiimoteState.MotionPlusState.RollFast ? 2*2000f * (mWiimoteState.MotionPlusState.RawValues.Y - unitsAtZeroDeg.Y) / 16384f : (2*500f * (mWiimoteState.MotionPlusState.RawValues.Y - unitsAtZeroDeg.Y)) / 16384f;
                    mWiimoteState.MotionPlusState.Values2.Z = mWiimoteState.MotionPlusState.YawFast ? 2*2000f * (mWiimoteState.MotionPlusState.RawValues.Z - unitsAtZeroDeg.Z) / 16384f : (2*500f * (mWiimoteState.MotionPlusState.RawValues.Z - unitsAtZeroDeg.Z)) / 16384f;
              
                   // mWiimoteState.MotionPlusState.Angles.Yaw = mWiimoteState.MotionPlusState.Values.X;



                    // New mapping - more intuitive and consistent
                    //Yaw = Z (rotation axis Z)
                    //Pitch = X(rotation axis X)
                    //Roll = Y (rotation axis Y)
                    //mWiimoteState.MotionPlusState.ZFast = ((buff[offset + 3] & 0x02) >> 1) == 0;
                    //mWiimoteState.MotionPlusState.XFast = ((buff[offset + 3] & 0x01) >> 0) == 0;
                    //mWiimoteState.MotionPlusState.YFast = ((buff[offset + 4] & 0x02) >> 1) == 0;

                    //mWiimoteState.MotionPlusState.RawValues.Z = (buff[offset + 0] | (buff[offset + 3] & 0xfa) << 6);
                    //mWiimoteState.MotionPlusState.RawValues.Y = (buff[offset + 1] | (buff[offset + 4] & 0xfa) << 6);
                    //mWiimoteState.MotionPlusState.RawValues.X = (buff[offset + 2] | (buff[offset + 5] & 0xfa) << 6);

                    //if (mWiimoteState.MotionPlusState.XFast)
                    //{
                    //    mWiimoteState.MotionPlusState.Values.X = (mWiimoteState.MotionPlusState.RawValues.X
                    //        - mWiimoteState.MotionPlusState.CalibrationInfo.GyroFastCalibration.X0) /
                    //        mWiimoteState.MotionPlusState.CalibrationInfo.GyroFastCalibration.XG;
                    //}
                    //else
                    //{
                    //    mWiimoteState.MotionPlusState.Values.X = (mWiimoteState.MotionPlusState.RawValues.X
                    //        - mWiimoteState.MotionPlusState.CalibrationInfo.GyroCalibration.X0) /
                    //        mWiimoteState.MotionPlusState.CalibrationInfo.GyroCalibration.XG;
                    //}

                    //if (mWiimoteState.MotionPlusState.YFast)
                    //{
                    //    mWiimoteState.MotionPlusState.Values.Y = (mWiimoteState.MotionPlusState.RawValues.Y
                    //        - mWiimoteState.MotionPlusState.CalibrationInfo.GyroFastCalibration.Y0) /
                    //        mWiimoteState.MotionPlusState.CalibrationInfo.GyroFastCalibration.YG;
                    //}
                    //else
                    //{
                    //    mWiimoteState.MotionPlusState.Values.Y = (mWiimoteState.MotionPlusState.RawValues.Y
                    //        - mWiimoteState.MotionPlusState.CalibrationInfo.GyroCalibration.Y0) /
                    //        mWiimoteState.MotionPlusState.CalibrationInfo.GyroCalibration.YG;
                    //}

                    //if (mWiimoteState.MotionPlusState.ZFast)
                    //{
                    //    mWiimoteState.MotionPlusState.Values.Z = (mWiimoteState.MotionPlusState.RawValues.Z
                    //        - mWiimoteState.MotionPlusState.CalibrationInfo.GyroFastCalibration.Z0) /
                    //        mWiimoteState.MotionPlusState.CalibrationInfo.GyroFastCalibration.ZG;
                    //}
                    //else
                    //{
                    //    mWiimoteState.MotionPlusState.Values.Z = (mWiimoteState.MotionPlusState.RawValues.Z
                    //        - mWiimoteState.MotionPlusState.CalibrationInfo.GyroCalibration.Z0) /
                    //        mWiimoteState.MotionPlusState.CalibrationInfo.GyroCalibration.ZG;
                    //}



                    if ((buff[offset + 4] & 0x01) == 1 && isStatusReady)
                    {
                        isStatusReady = false;

                        if (PASS_THRU_MODE == PassThruMode.None)
                        {
                           
                                DisableMotionPlus(); //would trigger status
                          
                              
                        }

                        GetStatus();

                        isStatusReady = true;
                    }

                  //Debug.WriteLineIf((buff[offset + 4] & 0x01)==1,"Extension is connected in continuation of Motion");
					
			}
		}

		private float GetBalanceBoardSensorValue(short sensor, short min, short mid, short max)
		{
			if(max == mid || mid == min)
				return 0;

			if(sensor < mid)
				return 68.0f * ((float)(sensor - min) / (mid - min));
			else
				return 68.0f * ((float)(sensor - mid) / (max - mid)) + 68.0f;
		}


		/// <summary>
		/// Parse data returned from a read report
		/// </summary>
		/// <param name="buff">Data buffer</param>
		private void ParseReadData(byte[] buff)
		{
			if((buff[3] & 0x08) != 0)
				throw new WiimoteException("Error reading data from Wiimote: Bytes do not exist.");

			if((buff[3] & 0x07) != 0)
			{
				Debug.WriteLine("*** read from write-only");
				LastReadStatus = LastReadStatus.ReadFromWriteOnlyMemory;
				mReadDone.Set();
				return;
			}

			// get our size and offset from the report
			int size = (buff[3] >> 4) + 1;
			int offset = (buff[4] << 8 | buff[5]);

			// add it to the buffer
			Array.Copy(buff, 6, mReadBuff, offset - mAddress, size);

			// if we've read it all, set the event
			if(mAddress + mSize == offset + size)
				mReadDone.Set();

			LastReadStatus = LastReadStatus.Success;
		}

		/// <summary>
		/// Returns whether rumble is currently enabled.
		/// </summary>
		/// <returns>Byte indicating true (0x01) or false (0x00)</returns>
		private byte GetRumbleBit()
		{
			return (byte)(mWiimoteState.Rumble ? 0x01 : 0x00);
		}

		/// <summary>
		/// Read calibration information stored on Wiimote
		/// </summary>
		private void ReadWiimoteCalibration()
		{
			// this appears to change the report type to 0x31
			byte[] buff = ReadData(REGISTER_CALIBRATION, 7);

            

			mWiimoteState.AccelCalibrationInfo.X0 = buff[0];
			mWiimoteState.AccelCalibrationInfo.Y0 = buff[1];
			mWiimoteState.AccelCalibrationInfo.Z0 = buff[2];
			mWiimoteState.AccelCalibrationInfo.XG = buff[4];
			mWiimoteState.AccelCalibrationInfo.YG = buff[5];
			mWiimoteState.AccelCalibrationInfo.ZG = buff[6];

            Debug.WriteLine("Wiimote callibration X0:" + buff[0] + " Y0:" + buff[1] + " Z0:" + buff[2] + " XG:" + buff[4]+" YG:"+buff[5]+" ZG:"+buff[6]);

		}

		/// <summary>
		/// Set Wiimote reporting mode (if using an IR report type, IR sensitivity is set to WiiLevel3)
		/// </summary>
		/// <param name="type">Report type</param>
		/// <param name="continuous">Continuous data</param>
		public void SetReportType(InputReport type, bool continuous)
		{
			Debug.WriteLine("SetReportType: " + type);
			SetReportType(type, IRSensitivity.Maximum, continuous);
		}

        public void SetReportType2(InputReport type, bool continuous)
        {
            byte[] buff = CreateReport();
            buff[0] = (byte)OutputReport.DataReportType;
            buff[1] =  (byte)((continuous ? 0x04 : 0x00) | (byte)(mWiimoteState.Rumble ? 0x01 : 0x00));
            buff[2] = (byte)type;

            WriteReport(buff);
        }


		/// <summary>
		/// Set Wiimote reporting mode
		/// </summary>
		/// <param name="type">Report type</param>
		/// <param name="irSensitivity">IR sensitivity</param>
		/// <param name="continuous">Continuous data</param>
		public void SetReportType(InputReport type, IRSensitivity irSensitivity, bool continuous)
		{
			// only 1 report type allowed for the BB
			if((mWiimoteState.Extension & (byte)ExtensionType.BalancedBoard) != 0)
				type = InputReport.ButtonsExtension;

			switch(type)
			{
				case InputReport.IRAccel:
					EnableIR(IRMode.Extended, irSensitivity);
					break;
				case InputReport.IRExtensionAccel:
					EnableIR(IRMode.Basic, irSensitivity);
					break;
				default:
					DisableIR();
					break;
			}

			byte[] buff = CreateReport();
			buff[0] = (byte)OutputReport.DataReportType;
			buff[1] = (byte)((continuous ? 0x04 : 0x00) | (byte)(mWiimoteState.Rumble ? 0x01 : 0x00));
			buff[2] = (byte)type;

			WriteReport(buff);
		}

		/// <summary>
		/// Set the LEDs on the Wiimote
		/// </summary>
		/// <param name="led1">LED 1</param>
		/// <param name="led2">LED 2</param>
		/// <param name="led3">LED 3</param>
		/// <param name="led4">LED 4</param>
		public void SetLEDs(bool led1, bool led2, bool led3, bool led4)
		{
			mWiimoteState.LEDState.LED1 = led1;
			mWiimoteState.LEDState.LED2 = led2;
			mWiimoteState.LEDState.LED3 = led3;
			mWiimoteState.LEDState.LED4 = led4;

			byte[] buff = CreateReport();

			buff[0] = (byte)OutputReport.LEDs;
			buff[1] =	(byte)(
						(led1 ? 0x10 : 0x00) |
						(led2 ? 0x20 : 0x00) |
						(led3 ? 0x40 : 0x00) |
						(led4 ? 0x80 : 0x00) |
						GetRumbleBit());

			WriteReport(buff);
		}

		/// <summary>
		/// Set the LEDs on the Wiimote
		/// </summary>
		/// <param name="leds">The value to be lit up in base2 on the Wiimote</param>
		public void SetLEDs(int leds)
		{
			mWiimoteState.LEDState.LED1 = (leds & 0x01) > 0;
			mWiimoteState.LEDState.LED2 = (leds & 0x02) > 0;
			mWiimoteState.LEDState.LED3 = (leds & 0x04) > 0;
			mWiimoteState.LEDState.LED4 = (leds & 0x08) > 0;

			byte[] buff = CreateReport();

			buff[0] = (byte)OutputReport.LEDs;
			buff[1] =	(byte)(
						((leds & 0x01) > 0 ? 0x10 : 0x00) |
						((leds & 0x02) > 0 ? 0x20 : 0x00) |
						((leds & 0x04) > 0 ? 0x40 : 0x00) |
						((leds & 0x08) > 0 ? 0x80 : 0x00) |
						GetRumbleBit());

			WriteReport(buff);
		}

		/// <summary>
		/// Toggle rumble
		/// </summary>
		/// <param name="on">On or off</param>
		public void SetRumble(bool on)
		{
			mWiimoteState.Rumble = on;

			// the LED report also handles rumble
			SetLEDs(mWiimoteState.LEDState.LED1, 
					mWiimoteState.LEDState.LED2,
					mWiimoteState.LEDState.LED3,
					mWiimoteState.LEDState.LED4);
		}

        public void SetMode(PassThruMode mode)
        {
            PASS_THRU_MODE = mode;
        }

		/// <summary>
		/// Retrieve the current status of the Wiimote and extensions.  Replaces GetBatteryLevel() since it was poorly named.
		/// </summary>
		public void GetStatus()
		{
			Debug.WriteLine("GetStatus");

           

			byte[] buff = CreateReport();

			buff[0] = (byte)OutputReport.Status;
			buff[1] = GetRumbleBit();

			WriteReport(buff);

			// signal the status report finished
			//if(!mStatusDone.WaitOne(3000, false))
			//	throw new WiimoteException("Timed out waiting for status report");
		}

		/// <summary>
		/// Turn on the IR sensor
		/// </summary>
		/// <param name="mode">The data report mode</param>
		/// <param name="irSensitivity">IR sensitivity</param>
		private void EnableIR(IRMode mode, IRSensitivity irSensitivity)
		{
			mWiimoteState.IRState.Mode = mode;

			byte[] buff = CreateReport();
			buff[0] = (byte)OutputReport.IR;
			buff[1] = (byte)(0x04 | GetRumbleBit());
			WriteReport(buff);

			Array.Clear(buff, 0, buff.Length);
			buff[0] = (byte)OutputReport.IR2;
			buff[1] = (byte)(0x04 | GetRumbleBit());
			WriteReport(buff);

			WriteData(REGISTER_IR, 0x08);
			switch(irSensitivity)
			{
				case IRSensitivity.WiiLevel1:
					WriteData(REGISTER_IR_SENSITIVITY_1, 9, new byte[] {0x02, 0x00, 0x00, 0x71, 0x01, 0x00, 0x64, 0x00, 0xfe});
					WriteData(REGISTER_IR_SENSITIVITY_2, 2, new byte[] {0xfd, 0x05});
					break;
				case IRSensitivity.WiiLevel2:
					WriteData(REGISTER_IR_SENSITIVITY_1, 9, new byte[] {0x02, 0x00, 0x00, 0x71, 0x01, 0x00, 0x96, 0x00, 0xb4});
					WriteData(REGISTER_IR_SENSITIVITY_2, 2, new byte[] {0xb3, 0x04});
					break;
				case IRSensitivity.WiiLevel3:
					WriteData(REGISTER_IR_SENSITIVITY_1, 9, new byte[] {0x02, 0x00, 0x00, 0x71, 0x01, 0x00, 0xaa, 0x00, 0x64});
					WriteData(REGISTER_IR_SENSITIVITY_2, 2, new byte[] {0x63, 0x03});
					break;
				case IRSensitivity.WiiLevel4:
					WriteData(REGISTER_IR_SENSITIVITY_1, 9, new byte[] {0x02, 0x00, 0x00, 0x71, 0x01, 0x00, 0xc8, 0x00, 0x36});
					WriteData(REGISTER_IR_SENSITIVITY_2, 2, new byte[] {0x35, 0x03});
					break;
				case IRSensitivity.WiiLevel5:
					WriteData(REGISTER_IR_SENSITIVITY_1, 9, new byte[] {0x07, 0x00, 0x00, 0x71, 0x01, 0x00, 0x72, 0x00, 0x20});
					WriteData(REGISTER_IR_SENSITIVITY_2, 2, new byte[] {0x1, 0x03});
					break;
				case IRSensitivity.Maximum:
					WriteData(REGISTER_IR_SENSITIVITY_1, 9, new byte[] {0x02, 0x00, 0x00, 0x71, 0x01, 0x00, 0x90, 0x00, 0x41});
					WriteData(REGISTER_IR_SENSITIVITY_2, 2, new byte[] {0x40, 0x00});
					break;
				default:
					throw new ArgumentOutOfRangeException("irSensitivity");
			}
			WriteData(REGISTER_IR_MODE, (byte)mode);
			WriteData(REGISTER_IR, 0x08);
		}

		/// <summary>
		/// Disable the IR sensor
		/// </summary>
		private void DisableIR()
		{
			mWiimoteState.IRState.Mode = IRMode.Off;

			byte[] buff = CreateReport();
			buff[0] = (byte)OutputReport.IR;
			buff[1] = GetRumbleBit();
			WriteReport(buff);

			Array.Clear(buff, 0, buff.Length);
			buff[0] = (byte)OutputReport.IR2;
			buff[1] = GetRumbleBit();
			WriteReport(buff);
		}

		/// <summary>
		/// Initialize the report data buffer
		/// </summary>
		private byte[] CreateReport()
		{
			return new byte[REPORT_LENGTH];
		}

		/// <summary>
		/// Write a report to the Wiimote
		/// </summary>
		private void WriteReport(byte[] buff)
		{
			Debug.WriteLine("WriteReport: " + Enum.Parse(typeof(OutputReport), buff[0].ToString()));
            if (mAltWriteMethod)
            {
                bool success=HIDImports.HidD_SetOutputReport(this.mHandle.DangerousGetHandle(), buff, (uint)buff.Length);

            }
            else if (mStream != null)
                mStream.Write(buff, 0, REPORT_LENGTH);

			if(buff[0] == (byte)OutputReport.WriteMemory)
			{
//				Debug.WriteLine("Wait");
				if(!mWriteDone.WaitOne(1000, false))
					Debug.WriteLine("Wait failed");
				//throw new WiimoteException("Error writing data to Wiimote...is it connected?");
			}
		}

		/// <summary>
		/// Read data or register from Wiimote
		/// </summary>
		/// <param name="address">Address to read</param>
		/// <param name="size">Length to read</param>
		/// <returns>Data buffer</returns>
		public byte[] ReadData(int address, short size)
		{
			byte[] buff = CreateReport();

			mReadBuff = new byte[size];
			mAddress = address & 0xffff;
			mSize = size;

			buff[0] = (byte)OutputReport.ReadMemory;
			buff[1] = (byte)(((address & 0xff000000) >> 24) | GetRumbleBit());
			buff[2] = (byte)((address & 0x00ff0000)  >> 16);
			buff[3] = (byte)((address & 0x0000ff00)  >>  8);
			buff[4] = (byte)(address & 0x000000ff);

			buff[5] = (byte)((size & 0xff00) >> 8);
			buff[6] = (byte)(size & 0xff);

//            buf[0]=flags & (CWIID_RW_EEPROM | CWIID_RW_REG);
//141	        buf[1]=(unsigned char)((offset>>16) & 0xFF);
//142	        buf[2]=(unsigned char)((offset>>8) & 0xFF);
//143	        buf[3]=(unsigned char)(offset & 0xFF);
//144	        buf[4]=(unsigned char)((len>>8) & 0xFF);
//145	        buf[5]=(unsigned char)(len & 0xFF);

            Debug.WriteLine("ReadMemory :" + AddressToName(address));
			WriteReport(buff);

            if (!mReadDone.WaitOne(1000, false))
            {
                Debug.WriteLine("ReadMemory Failed at address:" + AddressToName(address));
                    
                //throw new WiimoteException("Error reading data from Wiimote...is it connected?");
            }

			return mReadBuff;
		}


        private string AddressToName(int address)
        {
            string name;
            switch(address){
                case 0x04b00030: name="REGISTER_IR"; break;
                case 0x04b00000:  name="REGISTER_IR_SENSITIVITY_1" ; break;
                case 0x04b0001a:name="REGISTER_IR_SENSITIVITY_2";break;
                case 0x04b00033: name="REGISTER_IR_MODE"; break;

                case  0x04a400f0: name="REGISTER_EXTENSION_INIT_1"; break;
                case 0x04a400fb:			name="REGISTER_EXTENSION_INIT_2"; break;
                case 0x04a400fa:			name="REGISTER_EXTENSION_TYPE"; break;
                case 0x04a400fe:			name="REGISTER_EXTENSION_TYPE_2"; break;
                case 0x04a40020:	name=" REGISTER_EXTENSION_CALIBRATION"; break;
                case 0x0016:name="REGISTER_CALIBRATION";   break;//standard
                case 0x04a600fe: name = "REGISTER_MOTIONPLUS_DETECT"; break;
                case REGISTER_MOTIONPLUS_CALIBRATION: name = "REGISTER_MOTIONPLUS_CALIBRATION"; break;
                default: name = "Unknown address"; break;
            }


            return name;
        }

		/// <summary>
		/// Write a single byte to the Wiimote
		/// </summary>
		/// <param name="address">Address to write</param>
		/// <param name="data">Byte to write</param>
		public void WriteData(int address, byte data)
		{
			WriteData(address, 1, new byte[] { data });
		}

		/// <summary>
		/// Write a byte array to a specified address
		/// </summary>
		/// <param name="address">Address to write</param>
		/// <param name="size">Length of buffer</param>
		/// <param name="data">Data buffer</param>
		public void WriteData(int address, byte size, byte[] data)
		{
			byte[] buff = CreateReport();

			buff[0] = (byte)OutputReport.WriteMemory;
			buff[1] = (byte)(((address & 0xff000000) >> 24) | GetRumbleBit());
			buff[2] = (byte)((address & 0x00ff0000)  >> 16);
			buff[3] = (byte)((address & 0x0000ff00)  >>  8);
			buff[4] = (byte)(address & 0x000000ff);
			buff[5] = size;
			Array.Copy(data, 0, buff, 6, size);

			WriteReport(buff);
		}


       

    #region Calibration
	void InitCalibration(float x,float y,float z)
	{
		mCalibrationTimeout = CALIB_TIME;

        numCalibrationReadings = 0;
        pitchSum = 0;
        yawSum = 0;
        rollSum = 0;

        mMaxNoise =new Point3F();
        mBias =new Point3F();
        mMinNoise=new Point3F();

        if(mCalibrationStopWatch==null)
        mCalibrationStopWatch=new Stopwatch();

        mCalibrationStopWatch.Reset();
        mCalibrationStopWatch.Start();

        mMaxNoise.X=mBias.X=mMinNoise.X=x;
        mMaxNoise.Y=mBias.Y=mMinNoise.Y=y;
        mMaxNoise.Z=mBias.Z=mMinNoise.Z=z;


        //mNoiseLevel=new Point3F();
        //mNoiseThreshold=new Point3F();
        //mPrevAngleRates=new Point3F();
        //mAngleRates=new Point3F();

        //mMaxNoise = mMinNoise = mBias = vector3f(new_state.MotionPlus.Speed.Yaw, 
        //    new_state.MotionPlus.Speed.Pitch, new_state.MotionPlus.Speed.Roll);
        //mNoiseLevel = mNoiseThreshold = mPrevAngleRates = mAngleRates = vector3f(0,0,0);


		mNoise.Clear();
		
	}

    bool IsMotionPlusCalibrated()
	{
		return mMotionPlusCalibrated;
	}

	bool IsMotionPlusCalibrating()
	{
		return mCalibrationTimeout > 0;
	}

	// calculate the bias and std of angulr speeds
	// set mBias and mNoiseLevel
	void calculateCalibration()
	{
		int n = mNoise.Count;
		Point3F sum =  new Point3F();//vector3f(0,0,0);
        Point3F currentNoise;

		for (int i=0; i<n; i++) {
			//sum += mNoise.at(i);
            currentNoise = mNoise[i];
            sum.X += currentNoise.X;
            sum.Y += currentNoise.Y;
            sum.Z += currentNoise.Z;
               
		}


        

		mBias.X =(float) pitchSum/numCalibrationReadings;
		mBias.Y =(float)rollSum /numCalibrationReadings;
		mBias.Z = (float)yawSum/numCalibrationReadings;
		//mBias = sum/n;

		sum = new Point3F();//vector3f(0,0,0);
        float diff;
		for (int i=0; i<n; i++){
            //Point3F diff = mNoise.at(i) - mBias;
            //sum += diff % diff;
            currentNoise=mNoise[i];

            diff=(currentNoise.X-mBias.X);
            sum.X += diff % diff;

            diff = (currentNoise.Y - mBias.Y);
            sum.Y += diff % diff;

            diff = (currentNoise.Z - mBias.Z);
            sum.Z += diff % diff;
		}

		sum.X = sum.X / n;
        sum.Y = sum.Y / n;
        sum.Z = sum.Z / n;

        //mNoiseLevel = vector3f(sqrt(sum.x), sqrt(sum.y), sqrt(sum.z));
        //mNoiseThreshold = mNoiseLevel*3;

        mNoiseLevel = new Point3F((float)Math.Sqrt(sum.X),(float) Math.Sqrt(sum.Y),(float) Math.Sqrt(sum.Z));
        mNoiseThreshold = new Point3F(mNoiseLevel.X*3,mNoiseLevel.Y * 3,mNoiseLevel.Z * 3);
		
    

       
	}


    // calculate the avg and std of the bias in yaw, pitch, and roll
	//void UpdateCalibration(double dt, const wiimote_state new_state)
    void UpdateCalibration(double dt, float x,float y,float z)
    {
        //vector3f rates(new_state.MotionPlus.Speed.Yaw, new_state.MotionPlus.Speed.Pitch, new_state.MotionPlus.Speed.Roll);	
        Point3F rates = new Point3F(x, y, z);

        UpdateCalibration(dt, rates);
    }

	// calculate the avg and std of the bias in yaw, pitch, and roll
	//void UpdateCalibration(double dt, const wiimote_state new_state)
    void UpdateCalibration(double dt, Point3F values)
    {


        mMaxNoise.X = Math.Max(mMaxNoise.X, values.X);
        mMaxNoise.Y = Math.Max(mMaxNoise.Y, values.Y);
        mMaxNoise.Z = Math.Max(mMaxNoise.Z, values.Z);
        mMinNoise.X = Math.Min(mMinNoise.X, values.X);
        mMinNoise.Y = Math.Min(mMinNoise.Y, values.Y);
        mMinNoise.Z = Math.Min(mMinNoise.Z, values.Z);

        // If the wiimote is moving we need to recalibrate
        //if (((vector3f)(rates - ((mMaxNoise + mMinNoise) * 0.5))).length() > NOISE_FILTER) {
        //    InitCalibration(new_state);
        //    return;

         // If the wiimote is moving we need to recalibrate
        //double x = values.X - ((mMaxNoise.X + mMinNoise.X) * 0.5);
        //double y = values.Y - ((mMaxNoise.Y + mMinNoise.Y) * 0.5);
        //double z = values.Z - ((mMaxNoise.Z + mMinNoise.Z) * 0.5);

        //if (Math.Sqrt(x * x + y * y + z * z) > NOISE_FILTER)
        //{

        //    InitCalibration(values.X,values.Y,values.Z);
        //    return;
        //}


            // Store the "reading" in mNoise
            //mNoise.push_back(rates);
        mNoise.Add(values);





            mCalibrationTimeout -= dt;

            if (mCalibrationTimeout <= 0)
            {
                mMotionPlusCalibrated = true;
                mCalibrationStopWatch.Stop();

                calculateCalibration();


            }
            else
            {


                pitchSum += values.X;
                yawSum += values.Z;
                rollSum += values.Y;

                numCalibrationReadings++;
            }

        }
    
#endregion Calibration


 



     

		/// <summary>
		/// Current Wiimote state
		/// </summary>
		public WiimoteState WiimoteState
		{
			get { return mWiimoteState; }
		}

		///<summary>
		/// Unique identifier for this Wiimote (not persisted across application instances)
		///</summary>
		public Guid ID
		{
			get { return mID; }
		}

		/// <summary>
		/// HID device path for this Wiimote (valid until Wiimote is disconnected)
		/// </summary>
		public string HIDDevicePath
		{
			get { return mDevicePath; }
		}

		/// <summary>
		/// Status of last ReadMemory operation
		/// </summary>
		public LastReadStatus LastReadStatus { get; private set; }

		#region IDisposable Members

		/// <summary>
		/// Dispose Wiimote
		/// </summary>
		public void Dispose()
		{
			Dispose(true);
			GC.SuppressFinalize(this);
		}

		/// <summary>
		/// Dispose wiimote
		/// </summary>
		/// <param name="disposing">Disposing?</param>
		protected virtual void Dispose(bool disposing)
		{
			// close up our handles
			if(disposing)
				Disconnect();
		}
		#endregion



        
    }

	/// <summary>
	/// Thrown when no Wiimotes are found in the HID device list
	/// </summary>
	[Serializable]
	public class WiimoteNotFoundException : ApplicationException
	{
		/// <summary>
		/// Default constructor
		/// </summary>
		public WiimoteNotFoundException()
		{
		}

		/// <summary>
		/// Constructor
		/// </summary>
		/// <param name="message">Error message</param>
		public WiimoteNotFoundException(string message) : base(message)
		{
		}

		/// <summary>
		/// Constructor
		/// </summary>
		/// <param name="message">Error message</param>
		/// <param name="innerException">Inner exception</param>
		public WiimoteNotFoundException(string message, Exception innerException) : base(message, innerException)
		{
		}

		/// <summary>
		/// Constructor
		/// </summary>
		/// <param name="info">Serialization info</param>
		/// <param name="context">Streaming context</param>
		protected WiimoteNotFoundException(SerializationInfo info, StreamingContext context) : base(info, context)
		{
		}
	}

	/// <summary>
	/// Represents errors that occur during the execution of the Wiimote library
	/// </summary>
	[Serializable]
	public class WiimoteException : ApplicationException
	{
		/// <summary>
		/// Default constructor
		/// </summary>
		public WiimoteException()
		{
		}

		/// <summary>
		/// Constructor
		/// </summary>
		/// <param name="message">Error message</param>
		public WiimoteException(string message) : base(message)
		{
		}

		/// <summary>
		/// Constructor
		/// </summary>
		/// <param name="message">Error message</param>
		/// <param name="innerException">Inner exception</param>
		public WiimoteException(string message, Exception innerException) : base(message, innerException)
		{
		}

		/// <summary>
		/// Constructor
		/// </summary>
		/// <param name="info">Serialization info</param>
		/// <param name="context">Streaming context</param>
		protected WiimoteException(SerializationInfo info, StreamingContext context) : base(info, context)
		{
		}
	}
}