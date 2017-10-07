package org.usfirst.frc.team4600.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

import org.usfirst.frc.team4600.robot.*;
/**
 *
 */
public class Servo2Pos1 extends Command 
{
    public Servo2Pos1() 
    {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() 
    {
    	Robot.Servo2.setAngle(0);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() 
    {
    	//SmartDashboard.putNumber("Timer1", timer.get());
    	
    	//timer.start();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() 
    {
    	if(Robot.Servo2.getAngle() == 0)
    	{
    		return true;
    	}
        return false;
    }

    // Called once after isFinished returns true
    protected void end() 
    {
  
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
