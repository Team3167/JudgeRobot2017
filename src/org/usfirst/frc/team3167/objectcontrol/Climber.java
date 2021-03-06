package org.usfirst.frc.team3167.objectcontrol;

import org.usfirst.frc.team3167.robot.RobotConfiguration;

import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber {
	private Jaguar motor;
	private Joystick stick; 
	private Joystick stick2; 
	
	RobotConfiguration robotConfig = new RobotConfiguration(); 
	
	public Climber(int joystickPort, int joystick2Port, int motorPort) {
		motor = new Jaguar(motorPort);
		stick = new Joystick(joystickPort);
		stick2 = new Joystick(joystick2Port); 
	}
	
	//modify motor speeds
	public void slowSpin() {
		motor.set(-robotConfig.climberSlowSpeed);
	}
	public void slowSpinReverse() {
		motor.set(robotConfig.climberSlowSpeed);
	}
	public void mediumSpin() {
		motor.set(-robotConfig.climberMediumSpeed);
	}
	public void mediumSpinReverse() {
		motor.set(robotConfig.climberMediumSpeed);
	}
	public void fullSpin() {
		motor.set(-robotConfig.climberTopSpeed);
	}
	public void fullSpinReverse() {
		motor.set(robotConfig.climberTopSpeed);
	}
	public void haltMotor() {
		motor.set(0.0);
	}
	
	//accessors
	public double getSlowSpeed() {
		return robotConfig.climberSlowSpeed;
	}
	public double getMediumSpeed() {
		return robotConfig.climberMediumSpeed;
	}
	public double getFullSpeed() {
		return robotConfig.climberTopSpeed;
	}
	
	public void operate() {
		String msg = "";
    	if(stick.getRawButton(1) || stick2.getRawButton(1)) {
    		slowSpin();
    		msg = "Slow spin";
    	}
    	//BUTTON TWO NOW USED FOR SLIDE LOCK
    	/*else if(stick.getRawButton(2) || stick2.getRawButton(2)) {
    		slowSpinReverse();
    		msg = "Slow spin (R)";
    	}*/
    	else if(stick.getRawButton(5) || stick2.getRawButton(5)) {
    		mediumSpin();
    		msg = "Medium spin";
    	}
    	else if(stick.getRawButton(3) || stick2.getRawButton(3)) {
    		mediumSpinReverse();
    		msg = "Medium spin (R)";
    	}
    	else if(stick.getRawButton(6) || stick2.getRawButton(6)) {
    		fullSpin();
    		msg = "Full spin";
    	}
    	else if(stick.getRawButton(4) || stick2.getRawButton(4)) {
    		fullSpinReverse();
    		msg = "Full spin (R)";
    	} else {
    		haltMotor();
    		msg = "Halted";
    	}
    	SmartDashboard.putString("Climber function: ", msg);	
	}
}
