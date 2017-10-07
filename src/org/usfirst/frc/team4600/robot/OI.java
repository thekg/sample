package org.usfirst.frc.team4600.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	//// CREATING BUTTONS
	//XboxController Driver = new XboxController(0);
	Joystick Action = new Joystick(1);
    public OI(){
    	
    }
    public boolean getIntakeButton(){
       return Action.getTrigger();
    }
    
}
