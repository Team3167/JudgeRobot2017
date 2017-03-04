package org.usfirst.frc.team3167.robot.drive;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;

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
	
	private static final double deadband = 0.05;
	public void Drive(double x, double y, double rotation)
	{
		if (Math.abs(x) < deadband)
			x = 0.0;
		
		if (Math.abs(y) < deadband)
			y = 0.0;
		
		if (Math.abs(rotation) < deadband)
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
