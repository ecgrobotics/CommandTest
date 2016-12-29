package org.usfirst.frc.team9817.lib.utils;

public class Constants {
	public class RobotMap{
		public static final int steerBR = 5;
		public static final int steerBL = 6;
		public static final int steerFR = 4;
		public static final int steerFL = 7;
		
		public static final int driveBR = 1;
		public static final int driveBL = 2;
		public static final int driveFR = 0;
		public static final int driveFL = 3;

		public static final int encBR = 1;
		public static final int encBL = 2;
		public static final int encFR = 0;
		public static final int encFL = 3;
		
		public static final double encFLOffset = -96;
		public static final double encFROffset = -94;
		public static final double encBLOffset = -156;
		public static final double encBROffset = -13;
		
		public static final double robotWidth = 22;
		public static final double robotLength = 22;

	}
	public class PID{
		public static final double swerveP = .9;
		public static final double swerveI = 0;
		public static final double swerveD = .1;
		
		public static final double swerveCap = .4;
	}
	

}
