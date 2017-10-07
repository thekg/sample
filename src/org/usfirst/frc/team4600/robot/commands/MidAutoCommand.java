package org.usfirst.frc.team4600.robot.commands;

import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team4600.robot.*;

/**
 *
 */
public class MidAutoCommand extends Command 
{
	Timer timer = new Timer();
	double rotateToAngleRate;
	
	public MidAutoCommand() 
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
	}
  
	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() 
	{
		Robot.turnController.enable();
		if (timer.get() <= 1.6 /*1.8*/)
		{
			Robot.turnController.setSetpoint(0.0f);
			if (Math.abs(Robot.turnController.getError())<= 4)
            {
				rotateToAngleRate *= 0.2;
            }
			
			else
            {
				rotateToAngleRate = 0.2;
            }
            
		Robot.robotDrive.mecanumDrive_Cartesian(0, -0.25, rotateToAngleRate, 0);
		}
	}  
   
	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() 
	{
		if(timer.get() > 4.4)
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
