package org.usfirst.frc.team9817.robot.commands;

import org.usfirst.frc.team9817.robot.Robot;

public class DriveSwerve extends CommandBase {

	@Override
	public void initialize() {	
		requires(swerve);
	}

	@Override
	public void execute() {
		if(Math.abs(joy.getRawAxis(0)) > .1 || Math.abs(joy.getRawAxis(1)) > .1 || Math.abs(joy.getRawAxis(2)) > .1)
		swerve.driveNormal(Robot.joy.getRawAxis(0)*.7, Robot.joy.getRawAxis(1)*.7, Robot.joy.getRawAxis(2)*.5);
		else
			swerve.driveNormal(0, 0, 0);
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end() {
		// TODO Auto-generated method stub
	}

	@Override
	public void interrupted() {
		// TODO Auto-generated method stub

	}

}
