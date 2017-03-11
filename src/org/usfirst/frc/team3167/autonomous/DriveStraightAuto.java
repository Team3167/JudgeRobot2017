package org.usfirst.frc.team3167.autonomous;

import org.usfirst.frc.team3167.objectcontrol.GearHanger;
import org.usfirst.frc.team3167.robot.RobotConfiguration;
import org.usfirst.frc.team3167.robot.drive.SimpleMecanumDrive;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveStraightAuto {
	private final double fastSpeed = 0.75;
	private final double slowSpeed = 0.3;
	private final double driveFastTime = 1.0;// [sec]
	private final double driveSlowTime = 0.6;// [sec]
	private final double pauseTime = 0.5;// [sec]
	private final double reverseTime = 0.4;// [sec]
	
	private final SimpleMecanumDrive drive;
	private final GearHanger gearHanger; 
	
	// Can't use a standard java timer in a real-time application
	// We'll keep track of time on our own
	private double elapsedTime;// [sec]
	
	private final double hookSpeed = 0.7; 
	
	public DriveStraightAuto(SimpleMecanumDrive drive, GearHanger gearHanger) {
		this.drive = drive;
		this.gearHanger = gearHanger;
		resetTime(); 
	}
	
	private enum State
	{
		DriveForwardFast,
		DriveForwardSlow,
		PauseBeforeHang,
		HangGear,
		PauseAfterHang,
		Reverse,
		Stop
	}
	
	private State state;

	public void execute() {	
		switch (state)
		{
		case DriveForwardFast:
			drive.Drive(0.0, -fastSpeed, 0.0055, false);
			
			if (elapsedTime >= driveFastTime)
			{
				state = State.DriveForwardSlow;
				elapsedTime = 0.0;
			}
			break;
			
		case DriveForwardSlow:
			drive.Drive(0.0, -slowSpeed, 0.0055 * slowSpeed / fastSpeed, false);
			
			if (elapsedTime >= driveSlowTime)
			{
				state = State.PauseBeforeHang;
				elapsedTime = 0.0;
			}
			break;
			
		case PauseBeforeHang:
			drive.Drive(0.0, 0.0, 0.0, false);
			
			if (elapsedTime >= pauseTime)
			{
				state = State.HangGear;
				elapsedTime = 0.0;
			}
			break;
			
		case HangGear:
			drive.Drive(0.0, 0.0, 0.0, false);
			gearHanger.gearHangAuto(hookSpeed);
			
			if (gearHanger.hookIsDown())
			{
				state = State.PauseAfterHang;
				elapsedTime = 0.0;
			}
			break;
			
		case PauseAfterHang:
			drive.Drive(0.0, 0.0, 0.0, false);
			
			if (elapsedTime >= pauseTime)
			{
				state = State.Reverse;
				elapsedTime = 0.0;
			}
			break;
			
		case Reverse:
			drive.Drive(0.0, fastSpeed, 0.0, false);
			
			if (elapsedTime >= reverseTime)
				state = State.Stop;
			break;
			
		case Stop:
			drive.Drive(0.0, 0.0, 0.0, false);
			gearHanger.gearHangAuto(-hookSpeed);
			break;
		}
		
		elapsedTime += RobotConfiguration.timeStep;
	}
	
	public void resetTime() {
		elapsedTime = 0.0;
		state = State.DriveForwardFast;
	}
}