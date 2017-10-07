package org.usfirst.frc.team4600.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team4600.robot.*;

/*
Make foward is ff\orward on the real robot. Make sure it has THE gear box going foward.  
*/
public class RLeftAutoCommand extends Command 
{
	Timer timer = new Timer();
	WaitCommand wait3 = new WaitCommand(3);
	double rotateToAngleRate;
	
	public RLeftAutoCommand() 
	{
		// Use requires() here to declare subsystem dependencies
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() 
	{
		Robot.ahrs.reset();
		timer.reset();
		timer.start();
		rotateToAngleRate = 0.2;
		SmartDashboard.putString("RghtAutoTurn", "False");
	}
  
	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() 
	{
		
		SmartDashboard.putNumber("AutoRightTurnController", Robot.turnController.getSetpoint());
		if(timer.get() <= 1.4)
		{
			Robot.turnController.setSetpoint(0.0f);
			Robot.turnController.enable();
			if (Math.abs(Robot.turnController.getError())<= 4)
            {
				rotateToAngleRate *= 0.2;
            }
			else
            {
				rotateToAngleRate = 0.2;
            }
			
			Robot.robotDrive.mecanumDrive_Cartesian(0, -0.3, 0, 0);
		}

		if(timer.get() > 2.2 && timer.get() < 3.8)
		{
			rotateToAngleRate = 0.2;
			SmartDashboard.putString("RghtAutoTurn", "True");
			Robot.turnController.setSetpoint(-60f);
			Robot.turnController.enable();
			if (Math.abs(Robot.turnController.getError())<= 10)
            {
				rotateToAngleRate *= 0.2;
            }
			Robot.robotDrive.mecanumDrive_Cartesian(0, 0,  rotateToAngleRate, 0);
		}
		if(timer.get() > 4.4)
		{
			Robot.turnController.setSetpoint(-60f);
			Robot.turnController.enable();
			if (Math.abs(Robot.turnController.getError())<= 4)
            {
				rotateToAngleRate *= 0.2;
            }
			else
            {
				rotateToAngleRate = 0.2;
            }
			Robot.robotDrive.mecanumDrive_Cartesian(0, -0.3, rotateToAngleRate, 0);
		}
	  
	}  
   
	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() 
	{
		if(timer.get() > 4.9)
		{
			return true;
		}
		return false;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
	}
}
