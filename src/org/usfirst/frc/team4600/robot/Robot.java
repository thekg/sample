
package org.usfirst.frc.team4600.robot;

import org.usfirst.frc.team4600.robot.commands.BLeftAutoCommand;
import org.usfirst.frc.team4600.robot.commands.BRightAutoCommand;
import org.usfirst.frc.team4600.robot.commands.MidAutoCommand;
import org.usfirst.frc.team4600.robot.commands.RLeftAutoCommand;
import org.usfirst.frc.team4600.robot.commands.RRightAutoCommand;
import org.usfirst.frc.team4600.robot.commands.CancelAuto;
import org.usfirst.frc.team4600.robot.commands.Servo1Pos1;
import org.usfirst.frc.team4600.robot.commands.Servo1Pos2;
import org.usfirst.frc.team4600.robot.commands.Servo2Pos1;
import org.usfirst.frc.team4600.robot.commands.Servo2Pos2;
import org.usfirst.frc.team4600.robot.commands.Angle60;
import org.usfirst.frc.team4600.robot.commands.AngleN60;
import org.usfirst.frc.team4600.robot.commands.Angle0;
import org.usfirst.frc.team4600.robot.commands.AngleCancel;
//import org.usfirst.frc.team4600.robot.subsystems.Intake_Subsystem;
import org.usfirst.frc.team4600.robot.subsystems.ExampleSubsystem;

import com.kauailabs.navx.frc.AHRS;   

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.WaitCommand;
  
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the Apackage after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot implements PIDOutput 
{
	
	public static final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();
	//public static final Intake_Subsystem intakeSubsystem = new Intake_Subsystem();
	public static OI oi;
	
	public static Servo Servo1, Servo2;
	//NavX  
	public static AHRS ahrs; //Added public static due to Auto1C
	//
	public static RobotDrive robotDrive;
	public XboxController Xbox; 
	boolean applyBrake = false;
	Timer timer = new Timer();
	Command MidShipCommand =  new Angle0();
	Command LeftShipCommand = new AngleN60();
	Command RightShipCommand = new Angle60();
	Command CancelCommand = new AngleCancel();
	Command angleCommand;
	SendableChooser<Command> angleChoose = new SendableChooser<>();
	Command autonomousCommand;
	SendableChooser<Command> chooser = new SendableChooser<>();
	public static SendableChooser<Command> servo1chooser = new SendableChooser<>();
	//For rotate to Angle   
	public static PIDController turnController;
    public double rotateToAngleRate = 0.2;
    //
   
    //For Rotate to Angle 
    static final double kP = 0.03;
    static final double kI = 0.00;
    static final double kD = 0.00;
    static final double kF = 0.00;
    static final double kToleranceDegrees = 0.5f;
    //
    
	// Channels for the wheels
    final int frontLeftChannel	= 3;
    final int rearLeftChannel	= 1;
    final int frontRightChannel	= 4;
    final int rearRightChannel	= 2;
    static final int mXboxController	= 0;
    //
    
    Talon liftMotor ,retrieve, spit;
    
	// factor to convert sensor values to a distance in inches
	//private static final double kValueToInches = 0.125;
	//private static final int kUltrasonicPort = 0;
	//
	
	//private AnalogInput ultrasonic1 = new AnalogInput(kUltrasonicPort); //MatBotix
		/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */  
	
	Ultrasonic ultrasonic1 = new Ultrasonic(0, 1);//VexUltrasonic
	Ultrasonic ultrasonic2 = new Ultrasonic(2, 3);
	Ultrasonic ultrasonic3 = new Ultrasonic(4, 5);//
	int Counter = 0;
	
	@Override
	public void robotInit() 
	{    
		oi = new OI();
		
		
		chooser.addObject("MiddleAuto", new MidAutoCommand());
		chooser.addObject("BLeftAuto", new BLeftAutoCommand());
		chooser.addObject("BRightAuto", new BRightAutoCommand());
		chooser.addObject("RLefttAuto", new RLeftAutoCommand());
		chooser.addObject("RRightAuto", new RRightAutoCommand());
		chooser.addDefault("CancelAuto", new CancelAuto());
		SmartDashboard.putData("Auto mode", chooser);
   
		angleChoose.addObject("LeftShip", LeftShipCommand);
		angleChoose.addObject("MidShip", MidShipCommand);
		angleChoose.addObject("RightShip", RightShipCommand);
		angleChoose.addDefault("Cancel", CancelCommand);
		SmartDashboard.putData("ShipAngle", angleChoose);
		
		robotDrive = new RobotDrive(frontLeftChannel, rearLeftChannel, frontRightChannel, rearRightChannel);
    	robotDrive.setInvertedMotor(MotorType.kFrontLeft, true);	// invert the left side motors
    	robotDrive.setInvertedMotor(MotorType.kRearLeft, true);		// you may need to change or remove this to match your robot
        robotDrive.setExpiration(0.1);
         
        ultrasonic1.setAutomaticMode(true);
        ultrasonic2.setAutomaticMode(true);
        ultrasonic3.setAutomaticMode(true);
        //CameraServer.getInstance().startAutomaticCapture();
        
        Xbox = new XboxController(mXboxController);
        Servo1 = new Servo(9);
        Servo2 = new Servo(8);
        
        liftMotor = new Talon(7);
        retrieve = new Talon(5);
        spit = new Talon(6);
        
        LiveWindow.addSensor("UltrasonicT", "DigitalT", ultrasonic1);
        //LiveWindow.addSensor("Ultrasonic", "DigitalI", DI);
        LiveWindow.addActuator("Servo1", "Position", Servo1);
       
        try 
        {
			/***********************************************************************
			 * navX-MXP:
			 * - Communication via RoboRIO MXP (SPI, I2C, TTL UART) and USB.            
			 * - See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
			 * 
			 * navX-Micro:
			 * - Communication via I2C (RoboRIO MXP or Onboard) and USB.
			 * - See http://navx-micro.kauailabs.com/guidance/selecting-an-interface.
			 * 
			 * Multiple navX-model devices on a single robot are supported.
			 ************************************************************************/
            ahrs = new AHRS(SPI.Port.kMXP);
        }
        
        catch (RuntimeException ex ) 
        {
            DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
        }
        
      //For Rotate to Angle
        turnController = new PIDController(kP, kI, kD, kF, ahrs, this);
        turnController.setInputRange(-180.0f,  180.0f);
        turnController.setOutputRange(-1.0, 1.0);
        turnController.setAbsoluteTolerance(kToleranceDegrees);
        turnController.setContinuous(true);
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() 
	{

	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString code to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons
	 * to the switch structure below with additional strings & commands.
	 */  
	@Override
	public void autonomousInit() 
	{
		
		autonomousCommand = chooser.getSelected();
		autonomousCommand.start();
		/*
		  String autoSelected = SmartDashboard.getString("Auto Selector",
		  "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
		  = new MyAutoCommand(); break; case "Default Auto": default:
		  autonomousCommand = new ExampleCommand(); break; }
		 */

		// schedule the autonomous command (example)
		if (autonomousCommand != null)
			autonomousCommand.start();
	}
	
	//int counter = 10;
	
	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() 
	{
		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() 
	{
	
		angleCommand = angleChoose.getSelected();
		
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (autonomousCommand != null)
			autonomousCommand.cancel();
		
		//UltraS = new UltrasonicRangeFinder(0,1);
	}  

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() 
	{	  
		//NavX
        double NavXGyro = ahrs.getAngle();
        
        boolean StrtButton = Xbox. getStartButton();
        boolean AButton = Xbox.getAButton();
        SmartDashboard.putBoolean("AButton", AButton);
        boolean BButton = Xbox.getBButton();
   	 	SmartDashboard.putBoolean("BButton", BButton);
   	 	boolean XButton = Xbox.getXButton();
   	 	SmartDashboard.putBoolean("XButton", XButton);
		boolean YButton = Xbox.getYButton();
		SmartDashboard.putBoolean("YButton", YButton);
		boolean RBumper = Xbox.getBumper(GenericHID.Hand.kRight);
		boolean LBumper = Xbox.getBumper(GenericHID.Hand.kLeft);
        
		double rightX = Xbox.getX(GenericHID.Hand.kRight);
		double leftY = Xbox.getY(GenericHID.Hand.kLeft);
		double leftX  = Xbox.getX(GenericHID.Hand.kLeft);
        boolean rotateToAngle = false;
		
		SmartDashboard.putNumber("turnController", turnController.getSetpoint());
		 
        SmartDashboard.putData("servo1Pos1", new Servo1Pos1());
		SmartDashboard.putData("servo1Pos2", new Servo1Pos2());
		
		SmartDashboard.putData("servo2Pos1", new Servo2Pos1());
		SmartDashboard.putData("servo2Pos2", new Servo2Pos2());
		
		if (angleChoose.getSelected() == MidShipCommand)
		{
			SmartDashboard.putString("AngleCommand", "Mid");
			turnController.setSetpoint(0.0f);
	    	turnController.enable();
	    	rotateToAngle = true;
		}
		if (angleChoose.getSelected() == LeftShipCommand)
		{
			SmartDashboard.putString("AngleCommand", "Left");
			turnController.setSetpoint(-60.0f);
	    	turnController.enable();
	    	leftY = Xbox.getX(GenericHID.Hand.kLeft);
	        leftX = Xbox.getY(GenericHID.Hand.kLeft);
	        leftY *= -1;
	        leftX *= -1;
	    	rotateToAngle = true;
		}
		if (angleChoose.getSelected() == RightShipCommand)
		{
			SmartDashboard.putString("AngleCommand", "Right");
			turnController.setSetpoint(60.0f);
	    	turnController.enable();
	    	leftY = Xbox.getX(GenericHID.Hand.kLeft);
	        leftX = Xbox.getY(GenericHID.Hand.kLeft);
	        leftY *= -1;
	        leftX *= -1;
	    	rotateToAngle = true;
		}
		if (angleChoose.getSelected() == CancelCommand)
		{
			SmartDashboard.putString("AngleCommand", "Cancel");
			rotateToAngle = false;
		}
		
        robotDrive.setSafetyEnabled(true);
        

        
        if (AButton == true) 
        {
        	rotateToAngle = true;
        	turnController.setSetpoint(0.0f);
        } 
        
        else if (BButton == true) 
        {
            turnController.setSetpoint(-60.0f);//-90
            rotateToAngle = true;
            leftY = Xbox.getX(GenericHID.Hand.kLeft);
            leftX = Xbox.getY(GenericHID.Hand.kLeft);
        } 
        
        else if (XButton == true) 
        {
            turnController.setSetpoint(60.0f);//90
            rotateToAngle = true;
            leftY = Xbox.getX(GenericHID.Hand.kLeft);
            leftX = Xbox.getY(GenericHID.Hand.kLeft);
        } 
        
        else if (YButton == true) 
        {
            turnController.setSetpoint(179.9f);
            rotateToAngle = true;
        }
        
    
        if (rotateToAngle == true) 
        {  
            turnController.enable();
            rightX = rotateToAngleRate;
            rightX *= -1.0;
            if (Math.abs(rightX)>0.5)
            	{
            	 rightX *= 0.4;
            	}
            if (Math.abs(turnController.getError())<= 10)
            {
            	rightX *= 0.2;
            }
            else
            {
            	rightX = 0.2;
            }
            
            if (Math.abs(leftY) >= 0.5)
            {
            	 if (Math.abs(leftX) >= 0.5)
            	 {
            		 if (Math.abs(turnController.getError())<= 10)
            		 {
            			 rightX *= 2;
            		 }
            	 }
            }
        	SmartDashboard.putNumber("Get()", turnController.get());
        	SmartDashboard.putNumber("GetError", turnController.getError());
        } 
        if (rotateToAngle == false) 
        {
        	NavXGyro = 0;
            turnController.disable();
            rightX = Xbox.getX(GenericHID.Hand.kRight);
        }
        
        SmartDashboard.putBoolean("rotateToAngle", rotateToAngle);
    	SmartDashboard.putNumber("rotateToAngleRate", rotateToAngleRate);
		Scheduler.getInstance().run();

		double R_Trigger =  Xbox.getTriggerAxis(GenericHID.Hand.kRight);
	   	
	   	boolean RTrigger = Xbox.getTriggerAxis(GenericHID.Hand.kRight)>0.5;
	   	SmartDashboard.putNumber("RTrigger", R_Trigger);
	   	
	   	boolean LTrigger = Xbox.getTriggerAxis(GenericHID.Hand.kLeft)>0.5;
	   	
	   	SmartDashboard.putNumber("leftX", leftX);
	   	if (Math.abs(leftX)<0.5)
	   	{leftX = 0;}
	   	else if (leftX>0)
	   	{leftX -= 0.5;}
	   	else 
	   	{leftX += 0.5;}
	   
	   	SmartDashboard.putNumber("leftY", leftY);
	   	if (Math.abs(leftY)<0.5)
	   		{leftY = 0;}
	   	else if (leftY>0)
	   		{leftY -= 0.5;}
	   	else 
	   		{leftY += 0.5;}
	   			
	   	if (rotateToAngle == false)
	   	{
	   		rightX *= -1;
	   		if (Math.abs(rightX)<0.5)
	   		{rightX = 0;}
	   		else if (rightX>0)
	   		{
	   			rightX -= 0.5;
	   		}
	   		else 
	   		{
	   			rightX += 0.5;
	   		}
	   	}
	   	
	   	if(LTrigger)
	   	{
	   		leftX *=-1;
	   		leftY *=-1;
	   	}
	   	
	   	if (Xbox.getPOV() == 0)
		{
			leftY = -0.25;
		}
		
	   	if (Xbox.getPOV() == 90)
		{
			leftX = 0.25;
		}
	   	
	   	if (Xbox.getPOV() == 180)
		{
			leftY = 0.25;
		}
	   	
	   	if (Xbox.getPOV() == 270)
		{
			leftX = -0.25;
		}
	   	
	   	if (RBumper == true)
	   	{
	   		if(LTrigger)
	   		{
		   		liftMotor.set(1.0); // try harder	
	   		}
	   		else if (RTrigger)
	   		{
		   		liftMotor.set(-0.5); // unravel	   			
	   		}
	   		else
	   		{
	   		liftMotor.set(0.75); // start slower
	   		}
	   	}
	   	if (RBumper == false)
	   	{
	   		liftMotor.set(0);
	   	}
	   	if (LBumper == false)
	   	{
	   		retrieve.set(0);
   			spit.set(0);
	   	}
	   	if (LBumper == true)
	   	{
	   		retrieve.set(-1.0);
	   		if (RTrigger == true)
	   		{
	   			spit.set(1.0);
	   		}
	   		if (RTrigger == false)
	   		{
	   			spit.set(-1.0);
	   		}
	   	}
	   	SmartDashboard.putNumber("UltraInch1", ultrasonic1.getRangeInches());
	   	SmartDashboard.putNumber("UltraInch2", ultrasonic2.getRangeInches());
	   	SmartDashboard.putNumber("UltraInch3", ultrasonic3.getRangeInches());
	   	
		Counter += 1;
		SmartDashboard.putNumber("IntCount", Counter);
	   	
	   	SmartDashboard.putNumber("DPad", Xbox.getPOV(0));
	   	
	   	//robotDrive.mecanumDrive_Cartesian(leftX*-1.0, leftY * -1.0, rightX, 0);
	   	// Use the joystick X axis for lateral movement, Y axis for forward movement, and Z axis for rotation.
	   	// This sample does not use field-oriented drive, so the gyro input is set to zero.
      
	   	robotDrive.setSafetyEnabled(true);
	   		
		SmartDashboard.putBoolean("StrtButton", StrtButton);
		if (StrtButton == true) 
		{
	        ahrs.reset();
		}
	    
		SmartDashboard.putNumber("rightX", rightX *-1.0);
		SmartDashboard.putNumber("leftX", leftX*-1.0);
		SmartDashboard.putNumber("leftY", leftY * -1.0);
		
		try 
		{
	        /* Use the joystick X axis for lateral movement,            */
	        /* Y axis for forward movement, and Z axis for rotation.    */
	        /* Use navX MXP yaw angle to define Field-centric transform */
	        robotDrive.mecanumDrive_Cartesian(leftX*1.0, leftY *1.0, rightX, NavXGyro);
		} 
	
		catch( RuntimeException ex ) 
		{
	        DriverStation.reportError("Error communicating with drive system:  " + ex.getMessage(), true);
		}
		Timer.delay(0.005);		// wait for a motor update time

   	 
	   	// sensor returns a value from 0-4095 that is scaled to inches
	   	//double currentDistance = ultrasonic1.getValue() * kValueToInches;   	 
	   	//SmartDashboard.putNumber("UltraS", currentDistance);
		
		Timer.delay(0.005);	// wait 5ms to avoid hogging CPU cycles
       
	   
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() 
	{
		LiveWindow.run();
	}

	@Override
	public void pidWrite(double output) 
	{
		// TODO Auto-generated method stub
		rotateToAngleRate = output;
	}
	
}
