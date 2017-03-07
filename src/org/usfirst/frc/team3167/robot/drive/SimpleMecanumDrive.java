package org.usfirst.frc.team3167.robot.drive;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SimpleMecanumDrive {
	
	private final SpeedController leftFront;
	private final SpeedController leftRear;
	private final SpeedController rightFront;
	private final SpeedController rightRear;
	
	public SimpleMecanumDrive(SpeedController _leftFront, SpeedController _leftRear,
		SpeedController _rightFront, SpeedController _rightRear)
	{
		leftFront = _leftFront;
		leftRear = _leftRear;
		rightFront = _rightFront;
		rightRear = _rightRear;
	}
	
	//private static final double deadband = Preferences.getInstance().getDouble("deadband: ", 0.14);
	private static final double deadband = 0.1; 
	
	private static final double slideLockingFactor = 0.0;
	private static final double speedReductionFactor = 0.7;
	private static final double twistSpeedReductionFactor = 0.85;
	
	public void Drive(double x, double y, double rotation, boolean slideLock)
	{
		if(slideLock) {
			y = (y * slideLockingFactor) * speedReductionFactor;
			rotation = rotation * twistSpeedReductionFactor;
		}
		else if(!slideLock)
			x = (x * slideLockingFactor) * speedReductionFactor; 
			
		x = x * speedReductionFactor;
		y = y * speedReductionFactor;
		rotation = rotation * speedReductionFactor;
		
		if (Math.abs(x) < deadband)
			x = 0.0;
		
		if (Math.abs(y) < (deadband + 0.03))
			y = 0.0;
		
		//twist deadband is awfully high
		if (Math.abs(rotation) < (deadband + 0.05))
			rotation = 0.0; 
		
		double wheelSpeeds[] = new double[4];
	    wheelSpeeds[0] = x + y + rotation;
	    wheelSpeeds[1] = -x + y - rotation;
	    wheelSpeeds[2] = -x + y + rotation;
	    wheelSpeeds[3] = x + y - rotation;

	    normalize(wheelSpeeds);
	    leftFront.set(-wheelSpeeds[0]);
	    rightFront.set(wheelSpeeds[1]);
	    leftRear.set(-wheelSpeeds[2]);
	    rightRear.set(wheelSpeeds[3]);
	    
	    /* SmartDashboard.putNumber("jX: ", x);
	    SmartDashboard.putNumber("jY: ", y);
	    SmartDashboard.putNumber("rotation: ", rotation); */
	    
	    SmartDashboard.putNumber("jX Scaled: ", x * speedReductionFactor);
	    SmartDashboard.putNumber("jY Scaled: ", y * speedReductionFactor);
	    SmartDashboard.putNumber("rotation sc: ", rotation * speedReductionFactor); 
	}

	protected static void normalize(double wheelSpeeds[]) {
	    double maxMagnitude = Math.abs(wheelSpeeds[0]);
	    int i;
	    for (i = 1; i < 4; i++) {
	      double temp = Math.abs(wheelSpeeds[i]);
	      if (maxMagnitude < temp)
	        maxMagnitude = temp;
	    }
	    if (maxMagnitude > 1.0) {
	      for (i = 0; i < 4; i++) {
	        wheelSpeeds[i] = wheelSpeeds[i] / maxMagnitude;
	      }
	    }
	  }
}
