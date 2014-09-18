//////////////////////////////////////////////////////////////////////////////////
//	MultipleWiimoteForm.cs
//	Managed Wiimote Library Tester
//	Written by Brian Peek (http://www.brianpeek.com/)
//  for MSDN's Coding4Fun (http://msdn.microsoft.com/coding4fun/)
//	Visit http://blogs.msdn.com/coding4fun/archive/2007/03/14/1879033.aspx
//  and http://www.codeplex.com/WiimoteLib
//  for more information
//////////////////////////////////////////////////////////////////////////////////

using System;
using System.Drawing;
using System.Drawing.Imaging;
using System.Windows.Forms;
using WiimoteLib;

namespace WiimoteTest
{
	public partial class WiimoteInfo : UserControl
	{
		private delegate void UpdateWiimoteStateDelegate(WiimoteChangedEventArgs args);
		private delegate void UpdateExtensionChangedDelegate(WiimoteExtensionChangedEventArgs args);

		private Bitmap b = new Bitmap(256, 192, PixelFormat.Format24bppRgb);
		private Graphics g;
		private Wiimote mWiimote;
        private IFuser fuser;



        public WiimoteInfo()
        {
            InitializeComponent();
            g = Graphics.FromImage(b);

            
          //fuser = new KalmanMotionPlusFuser();
           //fuser = new MahonyMotionPlusFuser();
           fuser = new ComplementaryFilterFuser();
        }

		public WiimoteInfo(Wiimote wm) : this()
		{
			mWiimote = wm;

            

		}

		public void UpdateState(WiimoteChangedEventArgs args)
		{
			BeginInvoke(new UpdateWiimoteStateDelegate(UpdateWiimoteChanged), args);
		}

		public void UpdateExtension(WiimoteExtensionChangedEventArgs args)
		{
			BeginInvoke(new UpdateExtensionChangedDelegate(UpdateExtensionChanged), args);
		}

		private void chkLED_CheckedChanged(object sender, EventArgs e)
		{
			mWiimote.SetLEDs(chkLED1.Checked, chkLED2.Checked, chkLED3.Checked, chkLED4.Checked);
		}

		private void chkRumble_CheckedChanged(object sender, EventArgs e)
		{
			mWiimote.SetRumble(chkRumble.Checked);
		}

		private void UpdateWiimoteChanged(WiimoteChangedEventArgs args)
		{
			WiimoteState ws = args.WiimoteState;

			clbButtons.SetItemChecked(0, ws.ButtonState.A);
			clbButtons.SetItemChecked(1, ws.ButtonState.B);
			clbButtons.SetItemChecked(2, ws.ButtonState.Minus);
			clbButtons.SetItemChecked(3, ws.ButtonState.Home);
			clbButtons.SetItemChecked(4, ws.ButtonState.Plus);
			clbButtons.SetItemChecked(5, ws.ButtonState.One);
			clbButtons.SetItemChecked(6, ws.ButtonState.Two);
			clbButtons.SetItemChecked(7, ws.ButtonState.Up);
			clbButtons.SetItemChecked(8, ws.ButtonState.Down);
			clbButtons.SetItemChecked(9, ws.ButtonState.Left);
			clbButtons.SetItemChecked(10, ws.ButtonState.Right);

            lblAccel.Text = ws.AccelState.Values.ToString();// ws.AccelState.RawValues6b.ToString();// ws.AccelState.Values.ToString();
            lblAccOrientation.Text = ws.AccelState.Angles.ToString();// ws.AccelState.RawValues8b.ToString();// ws.AccelState.Angles.ToString();
           // lblOrientation.Text=ws.

			chkLED1.Checked = ws.LEDState.LED1;
			chkLED2.Checked = ws.LEDState.LED2;
			chkLED3.Checked = ws.LEDState.LED3;
			chkLED4.Checked = ws.LEDState.LED4;

                

			if  ((ws.Extension & (byte)ExtensionType.Nunchuck) !=0){
			
                    //ws.NunchukState.AccelState.RawValues.ToString();
                    lblChuk.Text = ws.NunchukState.AccelState.Values.ToString();
                    lblChukJoy.Text =ws.NunchukState.Joystick.ToString();//ws.NunchukState.RawJoystick.ToString();////w// 
					chkC.Checked = ws.NunchukState.C;
					chkZ.Checked = ws.NunchukState.Z;
					
            }
            
            if  ((ws.Extension & (byte)ExtensionType.ClassicController) !=0){
				
					clbCCButtons.SetItemChecked(0, ws.ClassicControllerState.ButtonState.A);
					clbCCButtons.SetItemChecked(1, ws.ClassicControllerState.ButtonState.B);
					clbCCButtons.SetItemChecked(2, ws.ClassicControllerState.ButtonState.X);
					clbCCButtons.SetItemChecked(3, ws.ClassicControllerState.ButtonState.Y);
					clbCCButtons.SetItemChecked(4, ws.ClassicControllerState.ButtonState.Minus);
					clbCCButtons.SetItemChecked(5, ws.ClassicControllerState.ButtonState.Home);
					clbCCButtons.SetItemChecked(6, ws.ClassicControllerState.ButtonState.Plus);
					clbCCButtons.SetItemChecked(7, ws.ClassicControllerState.ButtonState.Up);
					clbCCButtons.SetItemChecked(8, ws.ClassicControllerState.ButtonState.Down);
					clbCCButtons.SetItemChecked(9, ws.ClassicControllerState.ButtonState.Left);
					clbCCButtons.SetItemChecked(10, ws.ClassicControllerState.ButtonState.Right);
					clbCCButtons.SetItemChecked(11, ws.ClassicControllerState.ButtonState.ZL);
					clbCCButtons.SetItemChecked(12, ws.ClassicControllerState.ButtonState.ZR);
					clbCCButtons.SetItemChecked(13, ws.ClassicControllerState.ButtonState.TriggerL);
					clbCCButtons.SetItemChecked(14, ws.ClassicControllerState.ButtonState.TriggerR);

					lblCCJoy1.Text = ws.ClassicControllerState.JoystickL.ToString();
					lblCCJoy2.Text = ws.ClassicControllerState.JoystickR.ToString();

					lblTriggerL.Text = ws.ClassicControllerState.TriggerL.ToString();
					lblTriggerR.Text = ws.ClassicControllerState.TriggerR.ToString();
            }
            
            if  ((ws.Extension & (byte)ExtensionType.Guitar) !=0){

				    clbGuitarButtons.SetItemChecked(0, ws.GuitarState.FretButtonState.Green);
				    clbGuitarButtons.SetItemChecked(1, ws.GuitarState.FretButtonState.Red);
				    clbGuitarButtons.SetItemChecked(2, ws.GuitarState.FretButtonState.Yellow);
				    clbGuitarButtons.SetItemChecked(3, ws.GuitarState.FretButtonState.Blue);
				    clbGuitarButtons.SetItemChecked(4, ws.GuitarState.FretButtonState.Orange);
				    clbGuitarButtons.SetItemChecked(5, ws.GuitarState.ButtonState.Minus);
				    clbGuitarButtons.SetItemChecked(6, ws.GuitarState.ButtonState.Plus);
				    clbGuitarButtons.SetItemChecked(7, ws.GuitarState.ButtonState.StrumUp);
				    clbGuitarButtons.SetItemChecked(8, ws.GuitarState.ButtonState.StrumDown);

					clbTouchbar.SetItemChecked(0, ws.GuitarState.TouchbarState.Green);
					clbTouchbar.SetItemChecked(1, ws.GuitarState.TouchbarState.Red);
					clbTouchbar.SetItemChecked(2, ws.GuitarState.TouchbarState.Yellow);
					clbTouchbar.SetItemChecked(3, ws.GuitarState.TouchbarState.Blue);
					clbTouchbar.SetItemChecked(4, ws.GuitarState.TouchbarState.Orange);

					lblGuitarJoy.Text = ws.GuitarState.Joystick.ToString();
					lblGuitarWhammy.Text = ws.GuitarState.WhammyBar.ToString();
					lblGuitarType.Text = ws.GuitarState.GuitarType.ToString();
				   
            }
				 if  ((ws.Extension & (byte)ExtensionType.Drums) !=0){
					clbDrums.SetItemChecked(0, ws.DrumsState.Red);
					clbDrums.SetItemChecked(1, ws.DrumsState.Blue);
					clbDrums.SetItemChecked(2, ws.DrumsState.Green);
					clbDrums.SetItemChecked(3, ws.DrumsState.Yellow);
					clbDrums.SetItemChecked(4, ws.DrumsState.Orange);
					clbDrums.SetItemChecked(5, ws.DrumsState.Pedal);
					clbDrums.SetItemChecked(6, ws.DrumsState.Minus);
					clbDrums.SetItemChecked(7, ws.DrumsState.Plus);

					lbDrumVelocity.Items.Clear();
					lbDrumVelocity.Items.Add(ws.DrumsState.RedVelocity);
					lbDrumVelocity.Items.Add(ws.DrumsState.BlueVelocity);
					lbDrumVelocity.Items.Add(ws.DrumsState.GreenVelocity);
					lbDrumVelocity.Items.Add(ws.DrumsState.YellowVelocity);
					lbDrumVelocity.Items.Add(ws.DrumsState.OrangeVelocity);
					lbDrumVelocity.Items.Add(ws.DrumsState.PedalVelocity);

					lblDrumJoy.Text = ws.DrumsState.Joystick.ToString();
                 }

				 if  ((ws.Extension & (byte)ExtensionType.BalancedBoard) !=0){
					if(chkLbs.Checked)
					{
						lblBBTL.Text = ws.BalanceBoardState.SensorValuesLb.TopLeft.ToString();
						lblBBTR.Text = ws.BalanceBoardState.SensorValuesLb.TopRight.ToString();
						lblBBBL.Text = ws.BalanceBoardState.SensorValuesLb.BottomLeft.ToString();
						lblBBBR.Text = ws.BalanceBoardState.SensorValuesLb.BottomRight.ToString();
						lblBBTotal.Text = ws.BalanceBoardState.WeightLb.ToString();
					}
					else
					{
						lblBBTL.Text = ws.BalanceBoardState.SensorValuesKg.TopLeft.ToString();
						lblBBTR.Text = ws.BalanceBoardState.SensorValuesKg.TopRight.ToString();
						lblBBBL.Text = ws.BalanceBoardState.SensorValuesKg.BottomLeft.ToString();
						lblBBBR.Text = ws.BalanceBoardState.SensorValuesKg.BottomRight.ToString();
						lblBBTotal.Text = ws.BalanceBoardState.WeightKg.ToString();
					}
					lblCOG.Text = ws.BalanceBoardState.CenterOfGravity.ToString();
                 }

				 if  ((ws.Extension & (byte)ExtensionType.TaikoDrums) !=0){
					clbTaiko.SetItemChecked(0, ws.TaikoDrumState.OuterLeft);
					clbTaiko.SetItemChecked(1, ws.TaikoDrumState.InnerLeft);
					clbTaiko.SetItemChecked(2, ws.TaikoDrumState.InnerRight);
					clbTaiko.SetItemChecked(3, ws.TaikoDrumState.OuterRight);
                 }

                 if ((ws.Extension & (byte)ExtensionType.MotionPlus) != 0)
                 {
                //    lblOrientation.Text = "Roll: " + (ws.Position.Roll * Wiimote.RAD_TO_DEG).ToString("F3") + Environment.NewLine +
                //             "Pitch: " + (ws.Position.Pitch * Wiimote.RAD_TO_DEG).ToString("F3") + Environment.NewLine +
                //             "Yaw: " + (ws.Position.Yaw * Wiimote.RAD_TO_DEG).ToString("F3");
           
            
                     
                //        lblOrientationFiltered.Text="FGx: " + ws.MotionPlusState.FilteredValues.X.ToString("F3") + Environment.NewLine +
                //      "FGy: " + ws.MotionPlusState.FilteredValues.Y.ToString("F3") + Environment.NewLine +
                //      "FGz: " + ws.MotionPlusState.FilteredValues.Z.ToString("F3");


					//lblMotionPlusRaw.Text = ws.MotionPlusState.RawValues.ToString();
                   // lblMotionPlusRaw.Text=ws.MotionPlusState.Values.ToString();
                   // lblMotionPlus.Text = ws.MotionPlusState.Values2.ToString();
                   
					clbSpeed.SetItemChecked(0, ws.MotionPlusState.YawFast);
					clbSpeed.SetItemChecked(1, ws.MotionPlusState.PitchFast);
					clbSpeed.SetItemChecked(2, ws.MotionPlusState.RollFast);

                  

                  //  fuser.HandleIMUData(ws.MotionPlusState.Values.Z, ws.MotionPlusState.Values.X, ws.MotionPlusState.Values.Y, ws.AccelState.Values2.X, ws.AccelState.Values2.Y, ws.AccelState.Values2.Z);
                    fuser.HandleIMUData(ws.MotionPlusState.Values.Z, ws.MotionPlusState.Values.X, ws.MotionPlusState.Values.Y, ws.AccelState.Values.X, ws.AccelState.Values.Y, ws.AccelState.Values.Z);

                    lblMotionPlusRaw.Text = ws.MotionPlusState.RawValues.ToString();

                   
                    lblMotionPlus.Text= ws.MotionPlusState.Values.ToString();


                    lblOrientation.Text = fuser.FusedValues.ToString();
					
			}

			g.Clear(Color.Black);

			UpdateIR(ws.IRState.IRSensors[0], lblIR1, lblIR1Raw, 0, Color.Red);
			UpdateIR(ws.IRState.IRSensors[1], lblIR2, lblIR2Raw, 1, Color.Blue);
			UpdateIR(ws.IRState.IRSensors[2], lblIR3, lblIR3Raw, 2, Color.Yellow);
			UpdateIR(ws.IRState.IRSensors[3], lblIR4, lblIR4Raw, 3, Color.Orange);

			if(ws.IRState.IRSensors[0].Found && ws.IRState.IRSensors[1].Found)
				g.DrawEllipse(new Pen(Color.Green), (int)(ws.IRState.RawMidpoint.X / 4), (int)(ws.IRState.RawMidpoint.Y / 4), 2, 2);

			pbIR.Image = b;

			pbBattery.Value = (ws.Battery > 0xc8 ? 0xc8 : (int)ws.Battery);
			lblBattery.Text = ws.Battery.ToString();
			lblDevicePath.Text = "Device Path: " + mWiimote.HIDDevicePath;
		}

		private void UpdateIR(IRSensor irSensor, Label lblNorm, Label lblRaw, int index, Color color)
		{
			clbIR.SetItemChecked(index, irSensor.Found);

			if(irSensor.Found)
			{
				lblNorm.Text = irSensor.Position.ToString() + ", " + irSensor.Size;
				lblRaw.Text = irSensor.RawPosition.ToString();
				g.DrawEllipse(new Pen(color), (int)(irSensor.RawPosition.X / 4), (int)(irSensor.RawPosition.Y / 4),
							 irSensor.Size+1, irSensor.Size+1);
			}
		}

		private void UpdateExtensionChanged(WiimoteExtensionChangedEventArgs args)
		{
			chkExtension.Text = args.ExtensionType.ToString();
			chkExtension.Checked = args.Inserted;
		}

		public Wiimote Wiimote
		{
			set { mWiimote = value; }
		}

		private void button1_Click(object sender, EventArgs e)
		{
			mWiimote.InitializeMotionPlus();
		}

        private void disableMotionPlusButton_Click(object sender, EventArgs e)
        {
            mWiimote.DisableMotionPlus();
        }

        private void comboBox1_SelectedIndexChanged(object sender, EventArgs e)
        {
            string selectedType = ((ComboBox)sender).SelectedItem as String;

            mWiimote.SetReportType2(((InputReport)Enum.Parse(typeof(InputReport), selectedType)), true);
        }

        private void button2_Click(object sender, EventArgs e)
        {
            mWiimote.CheckMotionPlusCapabilities();

        }

       

       

        private void genericRealValueGen1_Generate(object Sender, Mitov.SignalLab.RealValueGenEventArgs Args)
        {
            Args.OutValue = mWiimote.WiimoteState.MotionPlusState.Values.X;                 //mWiimote.WiimoteState.AccelState.Angles.Pitch;
        }

        private void genericFilter1_ProcessData(object Sender, Mitov.SignalLab.ProcessBlockNotifyArgs Args)
        {
            uint size = Args.InBuffer.Size;
        }

        private void chkFactoryCalibration_CheckedChanged(object sender, EventArgs e)
        {
            mWiimote.USE_FACTORY_CALIBRATION = chkFactoryCalibration.Checked;
        }

        private void MotionPlusValue2_X_Generate(object Sender, Mitov.SignalLab.RealValueGenEventArgs Args)
        {
            Args.OutValue = mWiimote.WiimoteState.MotionPlusState.Values2.X;            
        }
	}
}
