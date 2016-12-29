package org.usfirst.frc.team9817.robot.commands;

import org.usfirst.frc.team9817.robot.subsystems.Swerve;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;

public abstract class CommandBase extends Command {
	Swerve swerve = new Swerve();
	Joystick joy = new Joystick(0);
	
	
	public abstract void initialize();
	public abstract void execute();
	public abstract boolean isFinished();
	public abstract void end();
	public abstract void interrupted();
	
}
