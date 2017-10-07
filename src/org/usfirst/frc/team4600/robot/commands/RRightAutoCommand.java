package org.usfirst.frc.team4600.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team4600.robot.*;

/**
 *
 */
public class RRightAutoCommand extends Command 
{
	Timer timer = new Timer();
	
	public RRightAutoCommand() 
	{
		// Use requires() here to declare subsystem dependencies
		requires(Robot.exampleSubsystem);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() 
	{
		timer.reset();
		timer.start();
	}
  
	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() 
	{
		double leftY = -0.15;
		Robot.robotDrive.mecanumDrive_Cartesian(0, leftY, 0, 0);
	}  
   
	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() 
	{
		if(timer.get() > 1.5)
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
