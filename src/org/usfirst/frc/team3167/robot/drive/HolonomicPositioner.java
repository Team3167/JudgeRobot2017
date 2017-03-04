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

	private final PIDControllerII xController;
	private final PIDControllerII yController;
	private final PIDControllerII thetaController;

	private double xTarget, yTarget, thetaTarget;
	private final double xTolerance = 1.0;// [in]
	private final double yTolerance = 1.0;// [in]
	private final double thetaTolerance = 2.5;// [deg]

	private final double kpLinearFine = 5.0;
	private final double tiLinearFine = 1.0;
	private final double kpRotaryFine = 0.1;
	private final double tiRotaryFine = 0.01;

	private final double kpLinearCoarse = 5.0;
	private final double tiLinearCoarse = 0.02;
	private final double kpRotaryCoarse = 0.02;
	private final double tiRotaryCoarse = 0.01;

	// Methods
	/**
	 * Constructor.
	 *
	 * @param _drive	Drive object that closes the velocity loops
	 * @param freq		Fixed frequency at which the loops are closed [Hz]
	 */
	public HolonomicPositioner(HolonomicDrive _drive, double freq)
	{
		// Assign local fields
		drive = _drive;

		// Create the controllers
		double integralTime = 2.0;// [sec]
		xController = new PIDControllerII(0.0, 0.0,
				(int)(integralTime * freq), freq);
		yController = new PIDControllerII(0.0, 0.0,
				(int)(integralTime * freq), freq);
		thetaController = new PIDControllerII(0.0, 0.0,
				(int)(integralTime * freq), freq);
		SetCoarseGains();
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
	}

	/**
	 * Sets the position reference for each of the three loops.
	 *
	 * @param x		X position reference
	 * @param y		Y position reference
	 * @param theta	Theta positon reference
	 */
	public void SetTargetPosition(double x, double y, double theta)
	{
		xTarget = x;
		yTarget = y;
		thetaTarget = theta;
	}

	/**
	 * Closes each position loop independently.
	 */
	public void Update()
	{
		double xCommand = xController.DoControl(xTarget, drive.GetXPosition());
		double yCommand = yController.DoControl(yTarget, drive.GetYPosition());
		double omegaCommand = thetaController.DoControl(thetaTarget,
				drive.GetThetaPosition());

		// Limit the commands to the max allowed by the drive object to reduce
		// the effect of one large error resulting in other commands being
		// ignored
		if (xCommand > drive.GetMaxXVel())
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
			omegaCommand = -drive.GetMaxOmega();

		System.out.println("xCmd: " + xCommand + "  yCmd: " + yCommand
				+ "  thetaCmd: " + omegaCommand);

		drive.Drive(xCommand, yCommand, omegaCommand);
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
	 * Prints the position command to the console for debugging.
	 */
    public void PrintCommand()
    {
        System.out.println("cmd x: " + xTarget + "  cmd y: " + yTarget
                + "  cmd theta: " + thetaTarget);
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

	/**
	 * Sets controller gains to the set used for rough positioning (large
	 * errors).
	 */
	public final void SetCoarseGains()
	{
		xController.SetKp(kpLinearCoarse);
		xController.SetTi(tiLinearCoarse);

		yController.SetKp(kpLinearCoarse);
		yController.SetTi(tiLinearCoarse);

		thetaController.SetKp(kpRotaryCoarse);
		thetaController.SetTi(tiRotaryCoarse);
	}

	/**
	 * Sets the controller gains to the set used for fine positioning (small
	 * errors).
	 */
	public final void SetFineGains()
	{
		xController.SetKp(kpLinearFine);
		xController.SetTi(tiLinearFine);
		
		yController.SetKp(kpLinearFine);
		yController.SetTi(tiLinearFine);

		thetaController.SetKp(kpRotaryFine);
		thetaController.SetTi(tiRotaryFine);
	}
}

