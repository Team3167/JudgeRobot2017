package org.usfirst.frc.team3167.autonomous;

import org.usfirst.frc.team3167.objectcontrol.GearHanger;
import org.usfirst.frc.team3167.robot.RobotConfiguration;
import org.usfirst.frc.team3167.robot.drive.HolonomicDrive;
import org.usfirst.frc.team3167.robot.drive.SimpleMecanumDrive;

public class DriveStraightAuto {
	private final double autoSpeed = 0.5;
	//private final double slowSpeed = 0.3;
	private final double driveFastTime = 0.85;// [sec] TODO: MODIFY THIS
	//private final double driveSlowTime = 0.7;// [sec]
	private final double pauseTime = 0.5;// [sec]
	private final double reverseTime = 0.4;// [sec]
	private final double rotateAdjustmentSpeed = 0.0095; 
	
	private final SimpleMecanumDrive drive;
	private final GearHanger gearHanger; 
	private final HolonomicDrive holoDrive;
	
	// Can't use a standard java timer in a real-time application
	// We'll keep track of time on our own
	private double elapsedTime;// [sec]
	private final double hookSpeed = 0.7; 
	
	public DriveStraightAuto(SimpleMecanumDrive drive, GearHanger gearHanger, HolonomicDrive holoDrive) {
		this.drive = drive;
		this.gearHanger = gearHanger;
		this.holoDrive = holoDrive;
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
	private State lastState;

	public void execute() {	
		switch (state)
		{
		case DriveForwardFast:
			drive.Drive(0.0, -autoSpeed, rotateAdjustmentSpeed, false);
			
			if (elapsedTime >= driveFastTime)
			{
				state = State.PauseBeforeHang;
			}
			break;
			
		/* case DriveForwardSlow:
			drive.Drive(0.0, -slowSpeed, rotateAdjustmentSpeed * slowSpeed / fastSpeed, false);
			
			if (elapsedTime >= driveSlowTime)
			{
				state = State.PauseBeforeHang;
			}
			break; */
			
		case PauseBeforeHang:
			drive.Drive(0.0, 0.0, 0.0, false);
			
			if (elapsedTime >= pauseTime)
			{
				state = State.HangGear;
			}
			break;
			
		case HangGear:
			drive.Drive(0.0, 0.0, 0.0, false);
			gearHanger.gearHangAuto(hookSpeed);
			
			if (gearHanger.hookIsDown())
			{
				state = State.PauseAfterHang;
			}
			break;
			
		case PauseAfterHang:
			drive.Drive(0.0, 0.0, 0.0, false);
			
			if (elapsedTime >= pauseTime)
			{
				state = State.Reverse;
			}
			break;
			
		case Reverse:
			drive.Drive(0.0, autoSpeed, 0.0, false);
			
			if (elapsedTime >= reverseTime)
				state = State.Stop;
			break;
			
		case Stop:
			drive.Drive(0.0, 0.0, 0.0, false);
			gearHanger.gearHangAuto(-hookSpeed);
			break;
		}
		
		// Reset timer for all state changes
		if (state != lastState)
		{
			elapsedTime = 0.0;
		}
		
		lastState = state;
		elapsedTime += RobotConfiguration.timeStep;
		System.out.println("et = " + elapsedTime + "; state = " + state + "; y = " + holoDrive.GetYPosition());
	}
	
	public void resetTime() {
		elapsedTime = 0.0;
		state = State.DriveForwardFast;
		lastState = state;
		holoDrive.Reset();
	}
}