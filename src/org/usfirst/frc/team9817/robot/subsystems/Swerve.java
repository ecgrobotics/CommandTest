package org.usfirst.frc.team9817.robot.subsystems;


import org.usfirst.frc.team9817.lib.utils.AbsoluteEncoder;
import org.usfirst.frc.team9817.lib.utils.Constants;
import org.usfirst.frc.team9817.lib.utils.Gyro;
import org.usfirst.frc.team9817.lib.utils.Vector;
import org.usfirst.frc.team9817.robot.commands.DriveSwerve;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * @author Duncan
 *
 */
public class Swerve extends Subsystem {
	double pivotX, pivotY, lastpressed, startangle, angleRotation, transAngle, pivotspeed;
	boolean lockwheels, drivingField;
	public static boolean rotating, angle;
	public SwerveModule[] modules;
	boolean fieldOrientation = false;
	SpeedController flDrive, frDrive, blDrive, brDrive, flsteer, frsteer, blsteer, brsteer;
	PIDController pid;
	Gyro gyro = new Gyro(SPI.Port.kOnboardCS0);

	/**
	 * Custom constructor for current robot.
	 */
	public Swerve() {
		lockwheels = false;	
		drivingField = false;
		//initialize array of modules
		//array can be any size, as long as the position of each module is specified in its constructor
		modules = new SwerveModule[] {
				//front left
				new SwerveModule(new Spark(Constants.RobotMap.driveFL),
						new Spark(Constants.RobotMap.steerFL),
						new AbsoluteEncoder(Constants.RobotMap.encFL, Constants.RobotMap.encFLOffset),
						-Constants.RobotMap.robotWidth/2,
						Constants.RobotMap.robotLength/2
						),
				//front right
				new SwerveModule(new Spark(Constants.RobotMap.driveFR), 
						new Spark(Constants.RobotMap.steerFR),
						new AbsoluteEncoder(Constants.RobotMap.encFR, Constants.RobotMap.encFROffset),
						Constants.RobotMap.robotWidth/2,
						Constants.RobotMap.robotLength/2
						),
				//back left
				new SwerveModule(new Spark(Constants.RobotMap.driveBL),
						new Spark(Constants.RobotMap.steerBL),
						new AbsoluteEncoder(Constants.RobotMap.encBL, Constants.RobotMap.encBLOffset),
						-Constants.RobotMap.robotWidth/2,
						-Constants.RobotMap.robotLength/2
						),
				//back right
				new SwerveModule(new Spark(Constants.RobotMap.driveBR), 
						new Spark(Constants.RobotMap.steerBR),
						new AbsoluteEncoder(Constants.RobotMap.encBR, Constants.RobotMap.encBROffset),
						Constants.RobotMap.robotWidth/2,
						-Constants.RobotMap.robotLength/2
						)
		};
		enable();
	}

	/**
	 * @param pivotX x coordinate in inches of pivot point relative to center of robot
	 * @param pivotY y coordinate in inches of pivot point relative to center of robot
	 */
	public void setPivot(double pivotX, double pivotY) {
		this.pivotX = pivotX;
		this.pivotY = pivotY;
	}

	public void debugMode(){

	}
	/**
	 * Drive with field oriented capability
	 * @param translationX relative speed in left/right direction (-1 to 1)
	 * @param translationY relative speed in forward/reverse direction (-1 to 1)
	 * @param rotation relative rate of rotation around pivot point (-1 to 1) positive is clockwise
	 * @param heading offset in heading in radians (used for field oriented control)
	 */
	private void driveWithOrient(double translationX, double translationY, double rotation, boolean fieldOrientation) {
		Vector correctOrientation = correctOrientationVector(translationX, translationY);
		translationX = fieldOrientation ? correctOrientation.x: translationX;
		translationY = fieldOrientation ? correctOrientation.y : translationY;
		Vector[] vects = new Vector[modules.length];
		Vector transVect = new Vector(translationX, translationY),
				pivotVect = new Vector(pivotX, pivotY);


		//if there is only one module ignore rotation
		if (modules.length < 2)
			for (SwerveModule module : modules) 
				module.set(transVect.getAngle(), Math.min(1, transVect.getMagnitude())); //cap magnitude at 1

		double maxDist = 0;
		for (int i = 0; i < modules.length; i++) {
			vects[i] = new Vector(modules[i].positionX, modules[i].positionY);
			vects[i].subtract(pivotVect); //calculate module's position relative to pivot point
			maxDist = Math.max(maxDist, vects[i].getMagnitude()); //find farthest distance from pivot
		}

		double maxPower = 1;
		for (int i = 0; i < modules.length; i++) {
			//rotation motion created by driving each module perpendicular to
			//the vector from the pivot point
			vects[i].makePerpendicular();
			//scale by relative rate and normalize to the farthest module
			//i.e. the farthest module drives with power equal to 'rotation' variable
			vects[i].scale(rotation / maxDist);
			vects[i].add(transVect);
			//calculate largest power assigned to modules
			//if any exceed 100%, all must be scale down
			maxPower = Math.max(maxPower, vects[i].getMagnitude());
		}


		double power;
		for (int i = 0; i < modules.length; i++) {
			power = vects[i].getMagnitude() / maxPower; //scale down by the largest power that exceeds 100%
			if (power > .05) {
				modules[i].set(vects[i].getAngle()-Math.PI/2, power);
			} else {
				modules[i].rest();
			}
		}
	}


	/**
	 * Regular robot oriented control.
	 * @param translationX relative speed in left/right direction (-1 to 1)
	 * @param translationY relative speed in forward/reverse direction (-1 to 1)
	 * @param rotation relative rate of rotation around pivot point (-1 to 1) positive is clockwise
	 */
	private Vector correctOrientationVector(double x, double y) {
		double angle = gyro.getAngle() * Math.PI / 180;
		return new Vector (x*Math.cos(angle) - y*Math.sin(angle), x*Math.sin(angle) + y*Math.cos(angle));
	}

	public void driveNormal(double translationX, double translationY, double rotation) {
		driveWithOrient(translationX, translationY, rotation, false);
	}
	public void driveField(double translationX, double translationY, double rotation){
		driveWithOrient(translationX, translationY, rotation, true);
	}

	public void enable() {
		for (SwerveModule module : modules) module.enable();
	}

	public void disable() {
		for (SwerveModule module : modules) module.disable();
	}

	
	public void lockWheels(){
		modules[0].set(45, 0);
		modules[1].set(-45, 0);
		modules[2].set(-45, 0);
		modules[3].set(45, 0);
	}


	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
		this.setDefaultCommand(new DriveSwerve());
	}
}

