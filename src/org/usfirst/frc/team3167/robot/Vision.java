package org.usfirst.frc.team3167.robot;

import edu.wpi.first.wpilibj.CameraServer;

public class Vision {
	
	CameraServer camServer = CameraServer.getInstance();
	
	public void enable() {
		setDetails();
		camServer.startAutomaticCapture("cam0");
	}
	
	public void setDetails() {
		camServer.setQuality(25); 
		camServer.setSize(75);
	}

}
