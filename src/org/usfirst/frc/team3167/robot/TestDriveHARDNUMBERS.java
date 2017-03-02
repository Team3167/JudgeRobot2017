package org.usfirst.frc.team3167.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TestDriveHARDNUMBERS {
	
	private Joystick stick = new Joystick(1);
	private Talon motor1;
	private Talon motor2;
	private Talon motor3;
	private Talon motor4;
	
	private Encoder encoder1;
    private Encoder encoder2;
    private Encoder encoder3;
    private Encoder encoder4;
	
	/**
	 * Simple method to feed hard numbers to the wheels so we know exactly what speed values
	 * are being used. From this, we can test that the encoders are giving similar readouts to
	 * determine not only their functionality but the functionality of the wheels/motors themselves.
	 * 
	 * IMPORTANT TO NOTE: Motor 2 and Motor 1 are wired opposite internally.
	 */
    
    public TestDriveHARDNUMBERS(int motor1P, int motor2P, int motor3P, int motor4P, int encoder1A, int encoder1B,
    		int encoder2A, int encoder2B, int encoder3A, int encoder3B, int encoder4A, int encoder4B) {
    	
    	motor1 = new Talon(motor1P); 
    	motor2 = new Talon(motor2P);
    	motor3 = new Talon(motor3P);
    	motor4 = new Talon(motor4P);
    	
    	encoder1 = new Encoder(encoder1A, encoder1B, false, EncodingType.k4X); //MOTOR 1 (10, 11)
        encoder2 = new Encoder(encoder2A, encoder2B, false, EncodingType.k4X); //MOTOR 2 (16, 17) 
        encoder3 = new Encoder(encoder3A, encoder3B, false, EncodingType.k4X); //MOTOR 3 (12, 13) 
        encoder4 = new Encoder(encoder4A, encoder4B, false, EncodingType.k4X); //MOTOR 4 (14, 15)
                
        encoder1.setDistancePerPulse(360.0 / 1024.0);
        encoder2.setDistancePerPulse(360.0 / 1024.0);
        encoder3.setDistancePerPulse(360.0 / 1024.0);
        encoder4.setDistancePerPulse(360.0 / 1024.0);
    }
    
    public TestDriveHARDNUMBERS(int motor1P, int motor2P, int motor3P, int motor4P) {
    	motor1 = new Talon(motor1P);
    	motor2 = new Talon(motor2P); 
    	motor3 = new Talon(motor3P); 
    	motor4 = new Talon(motor4P); 
    }
    
    public void setDistPerPulse() {
    	
    }
	
	//straight
	public void hardNumDrive(double speedCmd) {		
		double speed = speedCmd;
		String msg = ""; 
		
    	//strafe left
    	if(stick.getRawButton(7)) {
    		motor2.set(speed);
    		motor3.set(speed);
    		motor1.set(-speed);
    		motor4.set(-speed);
    		
    		msg = "Strafe left";
    	}
    	//strafe right
    	else if(stick.getRawButton(8)) {
    		motor2.set(-speed);
    		motor3.set(-speed);
    		motor1.set(speed);
    		motor4.set(speed);
    		
    		msg = "Strafe right";
    	}
    	//straight (forward)
    	else if(stick.getRawButton(9)) {
    		motor2.set(speed);
    		motor3.set(-speed);
    		motor1.set(-speed);
    		motor4.set(speed);
    		
    		msg = "Forward";
    	}
    	//reverse
    	else if(stick.getRawButton(10)) {
    		motor2.set(-speed);
    		motor3.set(speed);
    		motor1.set(speed);
    		motor4.set(-speed);
    		
    		msg = "Reverse";
    	}
    	//twist left
    	else if(stick.getRawButton(11)) {
    		motor2.set(speed);
    		motor3.set(-speed);
    		motor1.set(speed);
    		motor4.set(-speed);
    		
    		msg = "Twist left";
    	}
    	//twist right
    	else if(stick.getRawButton(12)) {
    		motor2.set(-speed);
    		motor3.set(speed);
    		motor1.set(-speed);
    		motor4.set(speed);
    		
    		msg = "Twist right"; 
    	} else {
    		motor1.set(0.0);
    		motor2.set(0.0);
    		motor3.set(0.0);
    		motor4.set(0.0);
    		
    		msg = "Stopped";
    	}
    	
    	SmartDashboard.putString("drive function: ", msg);
    	
    	SmartDashboard.putNumber("motor1: ", motor1.getSpeed());
    	//SmartDashboard.putNumber("encoder1: ", encoder1.getRate() / 360.0 * 60);
    	
    	SmartDashboard.putNumber("motor2: ", motor2.getSpeed());
    	//SmartDashboard.putNumber("encoder2: ", encoder2.getRate() / 360.0 * 60);
    	
    	SmartDashboard.putNumber("motor3: ", motor3.getSpeed());
    	//SmartDashboard.putNumber("encoder3: ", encoder3.getRate() / 360.0 * 60);
    	
    	SmartDashboard.putNumber("motor4: ", motor4.getSpeed());
    	//SmartDashboard.putNumber("encoder4: ", encoder4.getRate() / 360.0 * 60);
	}
	
	public void resetEncoders() {
		encoder1.reset();
		encoder2.reset();
		encoder3.reset();
		encoder4.reset();
	}
	
	public void sendDistToDash()
	{
    	SmartDashboard.putNumber("encoder1: ", encoder1.getDistance());
    	SmartDashboard.putNumber("encoder2: ", encoder2.getDistance());
    	SmartDashboard.putNumber("encoder3: ", encoder3.getDistance());
    	SmartDashboard.putNumber("encoder4: ", encoder4.getDistance());
	}
}
