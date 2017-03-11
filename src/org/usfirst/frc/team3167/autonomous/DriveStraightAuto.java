package org.usfirst.frc.team3167.autonomous;

import org.usfirst.frc.team3167.objectcontrol.GearHanger;
import org.usfirst.frc.team3167.robot.RobotConfiguration;
import org.usfirst.frc.team3167.robot.drive.SimpleMecanumDrive;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveStraightAuto {
	private final double autoSpeed;
	private final double driveTime;// [sec]
	
	private final Joystick stick; 
	private final Joystick stick2; 
	
	private final SimpleMecanumDrive drive;
	private final GearHanger gearHanger; 
	
	// Can't use a standard java timer in a real-time application
	// We'll keep track of time on our own
	private double elapsedTime;// [sec]
	
	public DriveStraightAuto(SimpleMecanumDrive drive, double autoSpeed, double driveTime) {
		this.drive = drive;
		this.autoSpeed = autoSpeed;
		this.driveTime = driveTime;
		
		stick = new Joystick(1); 
		stick2 = new Joystick(0); 
		
		gearHanger = new GearHanger(1, 2, 6, 8, 9); 
		
		elapsedTime = 0.0;
	}

	public void execute() {
		if (elapsedTime < driveTime) {
			drive.Drive(0.0, -autoSpeed, 0.0, false);
			elapsedTime += RobotConfiguration.timeStep;
			SmartDashboard.putNumber("autoElapsedTime: ", elapsedTime);
		} else {
			drive.Drive(0.0, 0.0, 0.0, false);
		}
	}
	
	public void resetTime() {
		elapsedTime = 0.0; 
	}
}