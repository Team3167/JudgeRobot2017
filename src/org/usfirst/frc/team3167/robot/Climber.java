package org.usfirst.frc.team3167.robot;

import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber {
	private Jaguar motor;
	private Joystick stick; 
	
	RobotConfiguration robotConfig = new RobotConfiguration(); 
	
	public Climber(int joystickPort, int motorPort) {
		motor = new Jaguar(motorPort);
		stick = new Joystick(joystickPort);
	}
	
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
    	if(stick.getRawButton(1)) {
    		slowSpin();
    		SmartDashboard.putString("Climber function: ", robotConfig.slowSpinMSG);
    		msg = "Slow spin";
    	}
    	else if(stick.getRawButton(2)) {
    		slowSpinReverse();
    		msg = "Slow spin (R)";
    	}
    	else if(stick.getRawButton(5)) {
    		mediumSpin();
    		//SmartDashboard.putString("Climber function: ", robotConfig.mediumSpinMSG);
    		msg = "Medium spin";
    	}
    	else if(stick.getRawButton(3)) {
    		mediumSpinReverse();
    		msg = "Medium spin (R)";
    	}
    	else if(stick.getRawButton(6)) {
    		fullSpin();
    		//SmartDashboard.putString("Climber function: ", robotConfig.fullSpinMSG);
    		msg = "Full spin";
    	}
    	else if(stick.getRawButton(4)) {
    		fullSpinReverse();
    		msg = "Full spin (R)";
    	} else {
    		haltMotor(); 
    		msg = "Halted";
    	}
    	SmartDashboard.putString("Climber function: ", msg);
		
	}
}
