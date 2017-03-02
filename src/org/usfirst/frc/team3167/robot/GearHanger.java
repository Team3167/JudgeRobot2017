package org.usfirst.frc.team3167.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GearHanger {
	
	private Jaguar gearMotor;
	private Joystick stick, stick2; 
	private DigitalInput limitSwitchHigh, limitSwitchLow; 
	
	public GearHanger(int joystickPort, int joystick2Port, int motorPort, int limitHighPort, int limitLowPort) {
		stick = new Joystick(joystickPort);
		stick2 = new Joystick(joystick2Port);
		gearMotor = new Jaguar(motorPort); 		
		limitSwitchHigh = new DigitalInput(limitHighPort); 
		limitSwitchLow = new DigitalInput(limitLowPort); 
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
		
		if(stick.getPOV() == forwardPos || stick2.getPOV() == forwardPos) {
			gearMotor.set(liftSpeed);
			hookMsg = "Lifting hook";
			povMsg = "Forward";
			
			if(!limitSwitchHigh.get()) {
				gearMotor.set(stop);
				switchMsg = "High pressed";
				
				if(stick.getPOV() == reversePos || stick2.getPOV() == reversePos) {
					gearMotor.set(lowerSpeed);
					hookMsg = "Lowering hook";
					povMsg = "Reverse";
				}
			}
		}
		else if(stick.getPOV() == reversePos || stick2.getPOV() == reversePos) {
			gearMotor.set(lowerSpeed);
			hookMsg = "Lowering hook";
			povMsg = "Reverse";
			
			if(!limitSwitchLow.get()) {
				gearMotor.set(stop);
				switchMsg = "Low pressed";
				
				if(stick.getPOV() == forwardPos || stick2.getPOV() == forwardPos) {
					gearMotor.set(liftSpeed);
					hookMsg = "Lifting hook";
					povMsg = "Forward";
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
	}

}
