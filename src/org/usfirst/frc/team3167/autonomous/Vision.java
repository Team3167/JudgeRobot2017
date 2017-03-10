package org.usfirst.frc.team3167.autonomous;

import edu.wpi.first.wpilibj.CameraServer;

public class Vision {
	
	CameraServer camServer;
	//CameraServer climberCamera, gearCamera; 
	String climberLocation, gearLocation;
	
	public Vision() {
		camServer = CameraServer.getInstance(); 
		
		/*climberLocation = climberPort;
		gearLocation = gearPort;*/
	}
	
	public void enable(String cameraLocation) {
		//setClimberDetails();
		//setGearDetails(); 
		setDetails(); 
		
		//camServer = CameraServer.getInstance();
		
		camServer.startAutomaticCapture(cameraLocation);
		
	}
	/*public void setClimberDetails() {
		climberCamera.setQuality(20); 
		climberCamera.setSize(70);
	}
	public void setGearDetails () {
		gearCamera.setQuality(20);
		gearCamera.setSize(70);
	} */
	public void setDetails() {
		camServer.setQuality(15);
		camServer.setSize(65);
	}

}
