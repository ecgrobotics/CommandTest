package org.usfirst.frc.team9817.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;


/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	Joystick joy = new Joystick(1);
	Button tri = new JoystickButton(joy, 0);
	Button sqr = new JoystickButton(joy, 1);
	Button x = new JoystickButton(joy, 2);
	Button o = new JoystickButton(joy, 3);
	
	public void move(){
		
	}
	
	public void Swerve(){
		
	}
}