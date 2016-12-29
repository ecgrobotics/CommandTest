package org.usfirst.frc.team9817.lib.utils;


import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI.Port;

public class Gyro {
	public static ADXRS450_Gyro gyro;
	double currentangle;
	
	public Gyro(Port channel){
		gyro = new ADXRS450_Gyro(channel);
		gyro.calibrate();
	}

	public double getAngle() {
		return gyro.getAngle();
	}
	
	public double getRate() {
		return gyro.getRate();
	}
	
	public double angleCorrect(){
		return gyro.getAngle() * -.015;
	}
	public void reset(){
		gyro.reset();
	}
	public double straight(boolean angle){
		if(angle){
			currentangle = gyro.getAngle();
			org.usfirst.frc.team9817.robot.subsystems.Swerve.angle = false;
		}
		return	(gyro.getAngle()-currentangle)*.015;
	}


}
