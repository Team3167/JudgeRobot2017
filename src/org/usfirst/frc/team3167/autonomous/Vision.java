package org.usfirst.frc.team3167.autonomous;

import java.awt.Image;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Joystick;

public class Vision {
	private Joystick stick; 
	private CameraServer gearServer;
	private CameraServer climberServer;
	private int gearCamChannel;
	private int climberCamChannel; 
	
	//CameraServer climberCamera, gearCamera; 
	//String climberLocation, gearLocation;
	
	public Vision(Joystick stick, int gearCamChannel, int climberCamChannel) {
		this.stick = stick;
		this.gearCamChannel = gearCamChannel;
		this.climberCamChannel = climberCamChannel; 
		
		gearServer = CameraServer.getInstance();
		climberServer = CameraServer.getInstance();
	}
	
	public void enable() {
		setDetails();
		
		gearServer.startAutomaticCapture(gearCamChannel);
		climberServer.startAutomaticCapture(climberCamChannel);
		
	}
	//OUTDATED
	/*public void setClimberDetails() {
		climberCamera.setQuality(20); 
		climberCamera.setSize(70);
	}
	public void setGearDetails () {
		gearCamera.setQuality(20);
		gearCamera.setSize(70);
	} */
	public void setDetails() {
		//camServer.setSize(65);
	}

}
