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
		
		String hookMsg = "";
		String switchMsg = "";
		
		if(stick.getPOV() == 0 || stick2.getPOV() == 0) {
			gearMotor.set(liftSpeed);
			hookMsg = "Lifting hook";
			
			if(!limitSwitchHigh.get()) {
				gearMotor.set(stop);
				switchMsg = "High pressed";
				
				if(stick.getPOV() == 180 || stick2.getPOV() == 180) {
					gearMotor.set(lowerSpeed);
					hookMsg = "Lowering hook";
				}
			}
		}
		else if(stick.getPOV() == 180 || stick2.getPOV() == 180) {
			gearMotor.set(lowerSpeed);
			hookMsg = "Lowering hook";
			
			if(!limitSwitchLow.get()) {
				gearMotor.set(stop);
				switchMsg = "Low pressed";
				
				if(stick.getPOV() == 0 || stick2.getPOV() == 0) {
					gearMotor.set(liftSpeed);
					hookMsg = "Lifting hook";
				}
			}
		} else {
			gearMotor.set(stop);
			hookMsg = "Neutral position";
		}
		
		SmartDashboard.putString("Gear function: ", hookMsg);
		SmartDashboard.putString("Microswitch Function: ", switchMsg);
	}

}
