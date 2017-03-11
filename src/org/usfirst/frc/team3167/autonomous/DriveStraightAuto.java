package org.usfirst.frc.team3167.autonomous;

import org.usfirst.frc.team3167.robot.drive.SimpleMecanumDrive;
import org.usfirst.frc.team3167.robot.RobotConfiguration;

public class DriveStraightAuto {
			
	private final double autoSpeed;
	private final double driveTime;// [sec]
	
	private final SimpleMecanumDrive drive;
	
	// Can't use a standard java timer in a real-time application
	// We'll keep track of time on our own
	private double elapsedTime = 0.0;// [sec]
	
	public DriveStraightAuto(SimpleMecanumDrive drive, double autoSpeed, double driveTime) {
		this.drive = drive;
		this.autoSpeed = autoSpeed;
		this.driveTime = driveTime;
	}

	public void execute() {
		if (elapsedTime < driveTime) {
			drive.Drive(0.0, autoSpeed, 0.0, false);
			elapsedTime += RobotConfiguration.timeStep;
		} else {
			drive.Drive(0.0, 0.0, 0.0, false);
		}
	}
	
	public void resetTime() {
		elapsedTime = 0.0; 
	}
}