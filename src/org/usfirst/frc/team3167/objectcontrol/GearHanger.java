package org.usfirst.frc.team3167.objectcontrol;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GearHanger {
	
	/*
	 * TODO: 
	 * 	- Remove joysticks from this class. Structure so this can be used in both autonomous and teleop. 
	 */
	
	private Jaguar gearMotor;
	private Joystick stick, stick2;  
	private DigitalInput limitSwitchHigh, limitSwitchLow; 
	private boolean checkSwitchPosition;
	
	public GearHanger(int joystickPort, int joystick2Port, int motorPort, int limitHighPort, int limitLowPort) {
		stick = new Joystick(joystickPort);
		stick2 = new Joystick(joystick2Port);
		gearMotor = new Jaguar(motorPort); 		
		limitSwitchHigh = new DigitalInput(limitHighPort); 
		limitSwitchLow = new DigitalInput(limitLowPort); 
		checkSwitchPosition = false; 
	}
	
	public void hangGear(double hookSpeed) {
		double liftSpeed = -hookSpeed;
		double lowerSpeed = hookSpeed; 
		double stop = 0.0; 
		
		int forwardPos = 0;
		int reversePos = 180; 
		
		//used for diagnostic 	
		String hookMsg = "";
		String switchMsg = "";
		String povMsg = ""; 
		
		checkStatus(); 
		
		if(stick.getPOV() == forwardPos || stick2.getPOV() == forwardPos) {
			gearMotor.set(liftSpeed);
			hookMsg = "Lifting hook";
			povMsg = "Forward";
			checkSwitchPosition = false; 
			
			if(hookIsUp()) {
				gearMotor.set(stop);
				switchMsg = "High pressed";
				
				if(stick.getPOV() == reversePos || stick2.getPOV() == reversePos) {
					gearMotor.set(lowerSpeed);
					hookMsg = "Lowering hook";
					checkSwitchPosition = true; 
					povMsg = "Reverse";
				}
			}
		}
		else if(stick.getPOV() == reversePos || stick2.getPOV() == reversePos) {
			gearMotor.set(lowerSpeed);
			hookMsg = "Lowering hook";
			povMsg = "Reverse";
			checkSwitchPosition = true; 
			
			if(hookIsDown()) {
				gearMotor.set(stop);
				switchMsg = "Low pressed";
				
				if(stick.getPOV() == forwardPos || stick2.getPOV() == forwardPos) {
					gearMotor.set(liftSpeed);
					hookMsg = "Lifting hook";
					povMsg = "Forward";
					checkSwitchPosition = false; 
				}
			}
		} else {
			gearMotor.set(stop);
			hookMsg = "Neutral position";
			povMsg = "Centered";
		}
		
		SmartDashboard.putString("Gear function: ", hookMsg);
		SmartDashboard.putString("Microswitch Function: ", switchMsg);
		SmartDashboard.putString("POV Position: ", povMsg);
		SmartDashboard.putBoolean("gearDown ", checkSwitchPosition);
	}
	
	private int hookDownCount = 0;
	private int hookUpCount = 0; 
	private final int switchCountLimit = 5;
	private boolean hookIsUp = false;
	private boolean hookIsDown = false; 
	
	public  void gearHangAuto(double hookSpeed) {
		double lowerSpeed = hookSpeed;
		String autoStr = ""; 		
		
		checkStatus(); 
		if(hookIsDown() && hookSpeed > 0) {
			gearMotor.set(0.0);
			autoStr = "at low point";
		} 
		else if(hookIsUp() && hookSpeed < 0) {
			gearMotor.set(0.0);
			autoStr = "at high point";
		} else {
			gearMotor.set(lowerSpeed);
			autoStr = "lowering hook";
		}
		
		SmartDashboard.putBoolean("limitSwitchLow: ", !limitSwitchLow.get());
		SmartDashboard.putString("auto hook: ", autoStr);
	}
	
	public boolean hookIsDown() {
		return hookIsDown; 
	}
	
	public boolean hookIsUp() { 
		return hookIsUp; 
	}
	
	private void checkStatus() {
		// Increment counter if switch is NOT depressed
		if(limitSwitchLow.get()) {
			hookDownCount++;
		}
		else
			hookDownCount = 0;
		
		if (hookDownCount == 0)
			hookIsDown = true;
		else if (hookDownCount > switchCountLimit)
			hookIsDown = false;
		
		// Increment counter if switch is NOT depressed
		if(limitSwitchHigh.get()) {
			hookUpCount++;
		}
		else
			hookUpCount = 0;
		
		if (hookUpCount == 0)
			hookIsUp = true;
		else if (hookUpCount > switchCountLimit)
			hookIsUp = false;
	}
}