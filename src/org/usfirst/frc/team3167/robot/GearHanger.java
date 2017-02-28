package org.usfirst.frc.team3167.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GearHanger {
	
	private Jaguar gearMotor;
	private Joystick stick; 
	private DigitalInput limitSwitchHigh, limitSwitchLow; 
	
	public GearHanger(int joystickPort, int motorPort, int limitHighPort, int limitLowPort) {
		stick = new Joystick(joystickPort);
		gearMotor = new Jaguar(motorPort); 		
		limitSwitchHigh = new DigitalInput(limitHighPort); 
		limitSwitchLow = new DigitalInput(limitLowPort); 
	}
	
	public void hangGear () {
		double liftSpeed = -0.4;
		double lowerSpeed = 0.4; 
		double stop = 0.0; 
		
		String hookMsg = "";
		String switchMsg = "";
		
		if(stick.getPOV() == 0) {
			gearMotor.set(liftSpeed);
			hookMsg = "Lifting hook";
			
			if(!limitSwitchHigh.get()) {
				gearMotor.set(stop);
				switchMsg = "High pressed";
				
				if(stick.getPOV() == 180) {
					gearMotor.set(lowerSpeed);
					hookMsg = "Lowering hook";
				}
			}
		}
		else if(stick.getPOV() == 180) {
			gearMotor.set(lowerSpeed);
			hookMsg = "Lowering hook";
			
			if(!limitSwitchLow.get()) {
				gearMotor.set(stop);
				switchMsg = "Low pressed";
				
				if(stick.getPOV() == 0) {
					gearMotor.set(liftSpeed);
					hookMsg = "Lifting hook";
				}
			}
		} else {
			gearMotor.set(stop);
			hookMsg = "Hook in neutral position";
		}
		
		SmartDashboard.putString("Gear function: ", hookMsg);
		SmartDashboard.putString("Microswitch Function: ", switchMsg);
	}

}
