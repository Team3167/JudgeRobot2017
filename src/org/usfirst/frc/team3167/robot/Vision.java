package org.usfirst.frc.team3167.robot;

import edu.wpi.first.wpilibj.CameraServer;

public class Vision {
	
	CameraServer camServer;
	String camLocation; 
	
	public Vision(String cameraLocation) {
		camServer = CameraServer.getInstance();
		camLocation = cameraLocation;
	}
	
	public void enable() {
		setDetails();
		camServer.startAutomaticCapture(camLocation);
	}
	public void setDetails() {
		camServer.setQuality(25); 
		camServer.setSize(75);
	}

}
