/*******************************************************************************
* File:  HolonomicPositioner.java
* Date:  3/20/2011
* Auth:  K. Loux
* Desc:  Class for controlling the position of the robot.
*******************************************************************************/

// Declare our package
package org.usfirst.frc.team3167.robot.drive;

// Local imports
import org.usfirst.frc.team3167.robot.util.PIDControllerII;
import org.usfirst.frc.team3167.robot.util.SecondOrderLimiter;
import org.usfirst.frc.team3167.autonomous.Networking;

/**
 * Position controller for a robot with holonomic motion.  Closes independent
 * loops for x, y and theta.
 *
 * @author K. Loux
 */
public class HolonomicPositioner
{
	// Local fields
	private final HolonomicDrive drive;
	private final double frequency;// [Hz]

	private final PIDControllerII xController;
	private final PIDControllerII yController;
	private final PIDControllerII thetaController;

	private double xTarget, yTarget, thetaTarget;
	
	private final SecondOrderLimiter xLimiter;
	private final SecondOrderLimiter yLimiter;
	private final SecondOrderLimiter thetaLimiter;
	
	private final double xTolerance = 0.5;// [in]
	private final double yTolerance = 1.0;// [in]
	private final double thetaTolerance = 5.0 * Math.PI / 180.0;// [rad]

	// Methods
	/**
	 * Constructor.
	 *
	 * @param _drive	Drive object that closes the velocity loops
	 * @param freq		Fixed frequency at which the loops are closed [Hz]
	 */
	public HolonomicPositioner(HolonomicDrive _drive, double freq)
	{
		drive = _drive;
		frequency = freq;
		
		final double kpLinear = 5.0;
		final double tiLinear = 1.0;
		final double kpRotary = 0.1;
		final double tiRotary = 1.0;

		double integralTime = 2.0;// [sec]
		xController = new PIDControllerII(kpLinear, tiLinear, 0.0, freq);
		yController = new PIDControllerII(kpLinear, tiLinear, 0.0, freq);
		thetaController = new PIDControllerII(kpRotary, tiRotary, 0.0, freq);
		
		final double linearMaxVel = 10.0;// [in/sec]
		final double linearMaxAccel = 15.0;// [in/sec^2]
		final double rotaryMaxVel = 20.0 * Math.PI / 180.0;// [rad/sec]
		final double rotaryMaxAccel = 20.0 * Math.PI / 180.0;// [rad/sec^2]
		
		xLimiter = new SecondOrderLimiter(linearMaxVel, linearMaxAccel, freq);
		yLimiter = new SecondOrderLimiter(linearMaxVel, linearMaxAccel, freq);
		thetaLimiter = new SecondOrderLimiter(rotaryMaxVel, rotaryMaxAccel, freq);
		
		final double cutoffFrequency = 2.0;// [Hz]
		final double zeta = 1.0;
		
		xLimiter.EnableFiltering(cutoffFrequency, zeta);
		yLimiter.EnableFiltering(cutoffFrequency, zeta);
		thetaLimiter.EnableFiltering(cutoffFrequency, zeta);
	}

	/**
	 * Resets the integral of the error signals to zero for all three
	 * controllers.
	 */
	public void ResetControllers()
	{
		xController.ResetError();
		yController.ResetError();
		thetaController.ResetError();
		
		lastX = 0.0;
		lastY = 0.0;
		lastTheta = 0.0;
		
		drive.ResetPositions(lastX, lastY, lastTheta);
		drive.Reset();
	}

	/**
	 * Sets the position reference for each of the three loops.  This must be called
	 * once per loop, even if we don't have new data from the vision processor.  In
	 * that case, this method should be called with the old data, until new estimates
	 * are available.
	 *
	 * @param x		X position reference
	 * @param y		Y position reference
	 * @param theta	Theta position reference
	 */
	private double lastX, lastY, lastTheta;
	public void SetTargetPosition(double x, double y, double theta)
	{
		double xVel = (lastX - x) * frequency;
		double yVel = (lastY - y) * frequency;
		double thetaVel = (lastTheta - theta) * frequency;
		
		xVel = xLimiter.Process(xVel);
		yVel = yLimiter.Process(yVel);
		thetaVel = thetaLimiter.Process(thetaVel);
		
		xTarget += xVel / frequency;
		yTarget += yVel / frequency;
		thetaTarget += thetaVel / frequency;
	}

	/**
	 * Closes each position loop independently.  Returns drive commands.
	 */
	public RobotPosition Update()
	{
		drive.UpdateEstimates();
		
		RobotPosition command = new RobotPosition();
		command.x = xController.DoControl(xTarget, drive.GetXPosition());
		command.y = yController.DoControl(yTarget, drive.GetYPosition());
		command.theta = thetaController.DoControl(thetaTarget,
				drive.GetThetaPosition());

		// Limit the commands to the max allowed by the drive object to reduce
		// the effect of one large error resulting in other commands being
		// ignored
		/*if (xCommand > drive.GetMaxXVel())
			xCommand = drive.GetMaxXVel();
		else if (xCommand < -drive.GetMaxXVel())
			xCommand = -drive.GetMaxXVel();

		if (yCommand > drive.GetMaxYVel())
			yCommand = drive.GetMaxYVel();
		else if (yCommand < -drive.GetMaxYVel())
			yCommand = -drive.GetMaxXVel();

		if (omegaCommand > drive.GetMaxOmega())
			omegaCommand = drive.GetMaxOmega();
		else if (omegaCommand < -drive.GetMaxOmega())
			omegaCommand = -drive.GetMaxOmega();*/

		return command;
	}

	/**
	 * Returns true if the magnitude of all of the three position errors are
	 * less than the allowable tolerance.
	 *
	 * @return True if the robot is at the commanded position
	 */
	public boolean AtTargetPosition()
	{
		// If all axes are within the specified tolerance range, return true
		if (Math.abs(drive.GetXPosition() - xTarget) < xTolerance &&
				Math.abs(drive.GetYPosition() - yTarget) < yTolerance &&
				Math.abs(drive.GetThetaPosition() - thetaTarget) < thetaTolerance)
			return true;

		return false;
	}

	/**
	 * Returns the difference between the actual and commanded X positions.
	 *
	 * @return Error in X position
	 */
	public double GetDeltaX()
	{
		return drive.GetXPosition() - xTarget;
	}

	/**
	 * Returns the difference between the actual and commanded Y positions.
	 *
	 * @return Error in Y position
	 */
	public double GetDeltaY()
	{
		return drive.GetYPosition() - yTarget;
	}

	/**
	 * Returns the difference between the actual and commanded theta positions.
	 *
	 * @return Error in theta position
	 */
	public double GetDeltaTheta()
	{
		return drive.GetThetaPosition() - thetaTarget;
	}
}

