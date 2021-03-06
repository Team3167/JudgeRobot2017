/*******************************************************************************
* File:  HolonomicRobotDrive.java
* Date:  1/9/2011
* Auth:  K. Loux
* Desc:  Holonomic drive class for use with mecanum wheels.
*******************************************************************************/

// Declare our package
package org.usfirst.frc.team3167.robot.drive;

// WPI imports
import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.ADXL345_I2C;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Judge imports
import org.usfirst.frc.team3167.robot.math.Matrix;
import org.usfirst.frc.team3167.robot.util.SecondOrderLimiter;
import org.usfirst.frc.team3167.robot.util.JoystickWrapper;
//import org.usfirst.frc.team3167.robot.util.PIDControllerII;

/**
 * Holonomic drive class for use with independently driven mecanum or omni
 * wheels.  The math is generalized to work with any number of wheels in any
 * position/orientation.  Assumption is that there are at least three wheels and
 * that they rotate in a vertical plane.  The wheels are assumed to all sit on
 * the same horizontal plane with their axes also on horizontal planes.
 *
 * The math in this class is carried out assuming the following coordinate
 * system:
 * <p>
 * X -> positive right<br>
 * Y -> positive forward<br>
 * Z -> positive up<br>
 * <p>
 *
 * The origin of this coordinate system is used as the default center of
 * rotation, but has no other relevance to the operation of the robot.
 *
 * This class operates on velocity control.  For position control, use this
 * class with a
 * {@link HolonomicPositioner} object.
 *
 * <p>
 * This object must be enabled by calling it's Initialize() method.  Not calling
 * this method will throw an exception and the robot will not move.
 *
 * @author K. Loux
 */
public class HolonomicDrive
{
	// Fields
    // Frequency at which this object is updated
    private double freq;

    // Velocity and Acceleration Limits
    private SecondOrderLimiter wheelSpeedLimiter[];
    private double friction = 1.0;// [-]
    private double vXMax = 1.0, vYMax = 1.0;// [in/sec]
	private double omegaMax = 1.0;// [rad/sec]

	// Deadband to apply when using joystick
	private double deadband = 0.03;// units are % / 100
	private double minimumOutput = 0.2;// units are % / 100 (open-loop only)
	private double dbSlope;// [-]
	private double dbIntercept;// [-]

    // Array of wheel objects
    private WheelList wheelList;

    // Initialization parameters
    private boolean initialized = false;

    // Objects used in the driving calculations (created during initialization)
    private Matrix robotMatrix;
    private Matrix cmdWheelVelocity;
    private Matrix cmdRobotMotion = new Matrix(3,1);
    private Matrix inverseRobotMatrix;
    private Matrix measuredWheelVelocity;
    private Matrix estimatedRobotVelocity = new Matrix(3,1);
    private Matrix estimatedRobotPosition = new Matrix(3,1);

	// Objects used in the modeling of robot motion
	private Matrix predictedRobotVelocity = new Matrix(3,1);

    // Methods
    /**
	 * Constructor.
	 *
	 * @param _freq					Fixed frequency at which this object must be
	 * updated (and loops will be closed) [Hz]
	 * @param digitalSideCarSlot	cRIO slot into which the digital sidecar is
	 * connected (designed for all PWM outputs and encoder inputs to be handled
	 * by the same digital sidecar)
	 * @param analogInputSlot		cRIO slot into which the analog input module
	 * handling the gyroscopes and the IR distance sensors is connected
	 */
    public HolonomicDrive(double _freq)
    {
        freq = _freq;
        wheelList = new WheelList();
    }

    /**
	 * Adds wheel to drive object using open-loop control.
	 *
	 * @param posX			X-location of the wheel [in]
	 * @param posY			Y-location of the wheel [in]
	 * @param axisX			X-component of unit vector describing the wheel's
	 * axis of rotation (direction of this vector defines positive direction for
	 * wheel rotation according to right hand rule)
	 * @param axisY			Y-component of unit vector describing the wheel's
	 * axis of rotation (direction of this vector defines positive direction for
	 * wheel rotation according to right hand rule)
	 * @param rollerAngle	Angle between vector defining the axis of rotation
	 * and the rolling element in contact with the ground [deg]
	 * @param radius		Radius of the wheel [in]
	 * @param motorSlot		cRIO slot into which the digital sidecar sending the
	 * PWM signal to the motor controller is connected
	 * @param motorChannel	PWM channel on the digital sidecar to which the
	 * motor controller is connected
     * @param maxSpeed      Maximum speed achievable at this wheel [rad/sec]
	 */
    public void AddWheel(double posX, double posY, double axisX, double axisY,
                double rollerAngle, double radius,
                SpeedController motor, double maxSpeed)
    {
        // Create the wheel object and add it to the array
        Wheel newWheel = new Wheel(posX, posY, axisX, axisY, rollerAngle,
                radius, motor, maxSpeed);

        wheelList.Add(newWheel);
    }

	/**
	 * Adds a wheel to drive object using PI-control with integral saturation to
	 * control windup.
	 *
	 * @param posX				X-location of the wheel [in]
	 * @param posY				Y-location of the wheel [in]
	 * @param axisX				X-component of unit vector describing the
	 * wheel's axis of rotation (direction of this vector defines positive
	 * direction for wheel rotation according to right hand rule)
	 * @param axisY				Y-component of unit vector describing the
	 * wheel's axis of rotation (direction of this vector defines positive
	 * direction for wheel rotation according to right hand rule)
	 * @param rollerAngle		Angle between vector defining the axis of
	 * rotation and the rolling element in contact with the ground [deg]
	 * @param radius			Radius of the wheel [in]
	 * @param gearRatioWheel	Gear ratio between the wheel and the encoder
	 * (not the motor) [encoder revs/wheel rev]
	 * @param motorSlot			cRIO slot into which the digital sidecar sending
	 * the PWM signal to the motor controller is connected
	 * @param motorChannel		PWM channel on the digital sidecar to which the
	 * motor controller is connected
	 * @param maxSpeed			Maximum allowable speed of the wheel [rad/sec]
	 * @param encSlotA			cRIO slot into which the digital sidecar reading
	 * the encoder A channel is connected
	 * @param encChanA			Digital input channel into which the encoder A
	 * pulse signal is connected
	 * @param encSlotB			cRIO slot into which the digital sidecar reading
	 * the encoder B channel is connected
	 * @param encChanB			Digital input channel into which the encoder B
	 * pulse signal is connected
	 * @param encPPR			Encoder pulses per revolution per channel
	 * @param reverseEncoder	Flag indicating wheter or not the encoder's
	 * positive direction should be swapped
	 * @param kp				Proportional gain
	 * @param ti				Integral gain
	 * @param saturation		Maximum allowable magnitude of the integral of
	 * the error signal
	 * @param dFiltOmega		Cutoff frequency for the filter on the feedback
	 * signal [Hz]
	 * @param dFiltZeta			Damping ratio for the filter on the feedback
	 * signal [-]
	 */
    public void AddWheel(double posX, double posY, double axisX, double axisY,
                double rollerAngle, double radius, double gearRatioWheel,
				SpeedController motor, double maxSpeed,
                Encoder encoder, double kp, double ti,
                double saturation, double dFiltOmega, double dFiltZeta)
    {
        // Create the wheel object and add it to the array
        Wheel newWheel;
		newWheel = new Wheel(posX, posY, axisX, axisY, rollerAngle,
			radius, gearRatioWheel, motor, maxSpeed,
			kp, ti, saturation, dFiltOmega, dFiltZeta, freq,
			encoder);

        wheelList.Add(newWheel);
    }

	/**
	 * Adds a wheel to drive object using PI-control with a fixed number of
	 * samples contributing to the integral of the error signal.
	 *
	 * @param posX				X-location of the wheel [in]
	 * @param posY				Y-location of the wheel [in]
	 * @param axisX				X-component of unit vector describing the
	 * wheel's axis of rotation (direction of this vector defines positive
	 * direction for wheel rotation according to right hand rule)
	 * @param axisY				Y-component of unit vector describing the
	 * wheel's axis of rotation (direction of this vector defines positive
	 * direction for wheel rotation according to right hand rule)
	 * @param rollerAngle		Angle between vector defining the axis of
	 * rotation and the rolling element in contact with the ground [deg]
	 * @param radius			Radius of the wheel [in]
	 * @param gearRatioWheel	Gear ratio between the wheel and the encoder
	 * (not the motor) [encoder revs/wheel rev]
	 * @param motorSlot			cRIO slot into which the digital sidecar sending
	 * the PWM signal to the motor controller is connected
	 * @param motorChannel		PWM channel on the digital sidecar to which the
	 * motor controller is connected
	 * @param maxSpeed			Maximum allowable speed of the wheel [rad/sec]
	 * @param encSlotA			cRIO slot into which the digital sidecar reading
	 * the encoder A channel is connected
	 * @param encChanA			Digital input channel into which the encoder A
	 * pulse signal is connected
	 * @param encSlotB			cRIO slot into which the digital sidecar reading
	 * the encoder B channel is connected
	 * @param encChanB			Digital input channel into which the encoder B
	 * pulse signal is connected
	 * @param encPPR			Encoder pulses per revolution per channel
	 * @param reverseEncoder	Flag indicating wheter or not the encoder's
	 * positive direction should be swapped
	 * @param kp				Proportional gain
	 * @param ki				Integral gain
	 * @param queueSize			Number of error samples to include when
	 * calculating the integral of the error signal [-]
	 * @param dFiltOmega		Cutoff frequency for the filter on the feedback
	 * signal [Hz]
	 * @param dFiltZeta			Damping ratio for the filter on the feedback
	 * signal [-]
	 */
    public void AddWheel(double posX, double posY, double axisX, double axisY,
                double rollerAngle, double radius, double gearRatioWheel,
				SpeedController motor, double maxSpeed,
                Encoder encoder, double kp, double ti,
                int queueSize, double dFiltOmega, double dFiltZeta)
    {
        // Create the wheel object and add it to the array
        Wheel newWheel;
		newWheel = new Wheel(posX, posY, axisX, axisY, rollerAngle,
			radius, gearRatioWheel, motor, maxSpeed,
			kp, ti, queueSize, dFiltOmega, dFiltZeta, freq,
			encoder);

        wheelList.Add(newWheel);
    }

	/**
	 * Adds a wheel to the drive object using PID-control and limiting windup by
	 * limiting the magnitude of the integral of the error signal.
	 *
	 * @param posX				X-location of the wheel [in]
	 * @param posY				Y-location of the wheel [in]
	 * @param axisX				X-component of unit vector describing the
	 * wheel's axis of rotation (direction of this vector defines positive
	 * direction for wheel rotation according to right hand rule)
	 * @param axisY				Y-component of unit vector describing the
	 * wheel's axis of rotation (direction of this vector defines positive
	 * direction for wheel rotation according to right hand rule)
	 * @param rollerAngle		Angle between vector defining the axis of
	 * rotation and the rolling element in contact with the ground [deg]
	 * @param radius			Radius of the wheel [in]
	 * @param gearRatioWheel	Gear ratio between the wheel and the encoder
	 * (not the motor) [encoder revs/wheel rev]
	 * @param motorSlot			cRIO slot into which the digital sidecar sending
	 * the PWM signal to the motor controller is connected
	 * @param motorChannel		PWM channel on the digital sidecar to which the
	 * motor controller is connected
	 * @param maxSpeed			Maximum allowable speed of the wheel [rad/sec]
	 * @param encSlotA			cRIO slot into which the digital sidecar reading
	 * the encoder A channel is connected
	 * @param encChanA			Digital input channel into which the encoder A
	 * pulse signal is connected
	 * @param encSlotB			cRIO slot into which the digital sidecar reading
	 * the encoder B channel is connected
	 * @param encChanB			Digital input channel into which the encoder B
	 * pulse signal is connected
	 * @param encPPR			Encoder pulses per revolution per channel
	 * @param reverseEncoder	Flag indicating wheter or not the encoder's
	 * positive direction should be swapped
	 * @param kp				Proportional gain
	 * @param ki				Integral gain
	 * @param kd				Derivative gain
	 * @param saturation		Maximum allowable magnitude of the integral of
	 * the error signal
	 * @param dFiltOmega		Cutoff frequency for the filter on the feedback
	 * signal [Hz]
	 * @param dFiltZeta			Damping ratio for the filter on the feedback
	 * signal [-]
	 */
    public void AddWheel(double posX, double posY, double axisX, double axisY,
                double rollerAngle, double radius, double gearRatioWheel,
				SpeedController motor, double maxSpeed,
                Encoder encoder,
                double kp, double ti, double kd, double saturation,
                double dFiltOmega, double dFiltZeta)
    {
        // Create the wheel object and add it to the array
        Wheel newWheel;
		newWheel = new Wheel(posX, posY, axisX, axisY, rollerAngle,
			    radius, gearRatioWheel, motor,
				kp, ti, kd, saturation, dFiltOmega, dFiltZeta, freq,
				encoder);

        wheelList.Add(newWheel);
    }

	/**
	 * Adds a wheel to the drive object using PID-control and limiting windup by
	 * using a fixed number of error samples to calculate the integral of the
	 * error signal.
	 *
	 * @param posX				X-location of the wheel [in]
	 * @param posY				Y-location of the wheel [in]
	 * @param axisX				X-component of unit vector describing the
	 * wheel's axis of rotation (direction of this vector defines positive
	 * direction for wheel rotation according to right hand rule)
	 * @param axisY				Y-component of unit vector describing the
	 * wheel's axis of rotation (direction of this vector defines positive
	 * direction for wheel rotation according to right hand rule)
	 * @param rollerAngle		Angle between vector defining the axis of
	 * rotation and the rolling element in contact with the ground [deg]
	 * @param radius			Radius of the wheel [in]
	 * @param gearRatioWheel	Gear ratio between the wheel and the encoder
	 * (not the motor) [encoder revs/wheel rev]
	 * @param motorSlot			cRIO slot into which the digital sidecar sending
	 * the PWM signal to the motor controller is connected
	 * @param motorChannel		PWM channel on the digital sidecar to which the
	 * motor controller is connected
	 * @param maxSpeed			Maximum allowable speed of the wheel [rad/sec]
	 * @param encSlotA			cRIO slot into which the digital sidecar reading
	 * the encoder A channel is connected
	 * @param encChanA			Digital input channel into which the encoder A
	 * pulse signal is connected
	 * @param encSlotB			cRIO slot into which the digital sidecar reading
	 * the encoder B channel is connected
	 * @param encChanB			Digital input channel into which the encoder B
	 * pulse signal is connected
	 * @param encPPR			Encoder pulses per revolution per channel
	 * @param reverseEncoder	Flag indicating wheter or not the encoder's
	 * positive direction should be swapped
	 * @param kp				Proportional gain
	 * @param ki				Integral gain
	 * @param kd				Derivative gain
	 * @param queueSize			Number of error samples to include when
	 * calculating the integral of the error signal [-]
	 * @param dFiltOmega		Cutoff frequency for the filter on the feedback
	 * signal [Hz]
	 * @param dFiltZeta			Damping ratio for the filter on the feedback
	 * signal [-]
	 */
    public void AddWheel(double posX, double posY, double axisX, double axisY,
                double rollerAngle, double radius, double gearRatioWheel,
				SpeedController motor, double maxSpeed,
                Encoder encoder,
                double kp, double ti, double kd, int queueSize,
				double dFiltOmega, double dFiltZeta)
    {
        // Create the wheel object and add it to the array
        Wheel newWheel;
		newWheel = new Wheel(posX, posY, axisX, axisY, rollerAngle,
			    radius, gearRatioWheel,
				motor,
				kp, ti, kd, queueSize, dFiltOmega, dFiltZeta, freq,
				encoder);

        wheelList.Add(newWheel);
    }

	/**
	 * Initialization method for the HolonomicRobotDrive object.  MUST be called
	 * prior to using this class to control robot motion.  Failing to do so will
	 * result in an exception being thrown.
	 * <p>
	 * Prior to calling this method, all of the wheels being controlled by this
	 * object must have been added with the appropriate AddWheel() method.
	 * Minimum of three wheels must be added, possibly more depending on the
	 * position and orientation of each wheel.
	 * <p>
	 * Returns true if initialization was successful, false otherwise.
	 *
	 * @return True if initialization was successful, false otherwise
	 *
	 * @throws ArithmeticException
	 */
    // Initialization method to validate all input parameters and create
    // objects used during the driving calculations
    public boolean Initialize() throws ArithmeticException
    {
        // Create the n x 3 robot matrix
        double robotArray[][] = new double[wheelList.Size()][3];

        int i;
        double beta;
        for (i = 0; i < wheelList.Size(); i++)
        {
            beta = wheelList.Get(i).GetRollerAngle() * Math.PI / 180.0;
            robotArray[i][0] =
                    (wheelList.Get(i).GetRotationAxisY() * Math.sin(beta) -
                    wheelList.Get(i).GetRotationAxisX() * Math.cos(beta)) /
                    (wheelList.Get(i).GetRadius() * Math.sin(beta));
            robotArray[i][1] =
                    -(wheelList.Get(i).GetRotationAxisX() * Math.sin(beta) +
                    wheelList.Get(i).GetRotationAxisY() * Math.cos(beta)) /
                    (wheelList.Get(i).GetRadius() * Math.sin(beta));
            robotArray[i][2] = wheelList.Get(i).GetXPos() * robotArray[i][1] -
                    wheelList.Get(i).GetYPos() * robotArray[i][0];
        }

        robotMatrix = new Matrix(robotArray);
		//System.out.println(robotMatrix.Print());
		/*robotMatrix.SetElement(2, 2, -robotMatrix.GetElement(0, 2));
		robotMatrix.SetElement(3, 2, -robotMatrix.GetElement(1, 2));*/

        // Allocate the n x 1 velocity (actual and command) matrices
        cmdWheelVelocity = new Matrix(wheelList.Size(), 1);
        measuredWheelVelocity = new Matrix(wheelList.Size(), 1);

        // We also need the inverse of the robot matrix for estimating velocity
        // based on feedback
        inverseRobotMatrix = robotMatrix.GetPsuedoinverse();

        // Create the limiter objects for each wheel
        wheelSpeedLimiter = new SecondOrderLimiter[wheelList.Size()];
        double wheelMaxAccel;// [rad/sec^2]
        for (i = 0; i < wheelList.Size(); i++)
        {
            wheelMaxAccel = friction * 12 * 32.174
                    / wheelList.Get(i).GetRadius();
            wheelSpeedLimiter[i] = new SecondOrderLimiter(
					wheelList.Get(i).GetMaxRotationRate(), wheelMaxAccel, freq);
        }

        // If we have full column rank, we have enough constraints defined to
        // control the motion of the robot.
        initialized = robotMatrix.GetRank() >= 3 ? true : false;

		// Tell the user if initialization failed
		if (!initialized)
		{
			System.err.println("HolonomicRobotDrive not initialized!");
			System.err.println("    Check that AddWheel has been called for" +
					" each wheel");
			System.err.println("    There must be at least three wheels");
			System.err.println("    The wheel's orientation and position must"+
					" be sufficient to control the robot in all three DOF");

			throw new ArithmeticException("Initialization of holonimic drive "
					+ "failed!");
		}
        else
        {
            // Determine the maximum allowed velocities based on max allowed
            // wheel speed

            // Assign a unit velocity in the x-direction to the commanded motion
            // matrix and solve for the wheel velocities
            cmdRobotMotion.SetElement(0, 0, 1.0);
            cmdWheelVelocity = robotMatrix.Multiply(cmdRobotMotion);

            // Find the wheel that requires the smallest scaling factor
			// (smallest allowable speed divided by the largest required speed)
            double minScalingFactor = 0.0;
            for (i = 0; i < wheelList.Size(); i++)
            {
                if (wheelList.Get(i).GetMaxRotationRate() /
						Math.abs(cmdWheelVelocity.GetElement(i, 0)) <
                        minScalingFactor || minScalingFactor == 0.0)
                    minScalingFactor =
							wheelList.Get(i).GetMaxRotationRate() /
							Math.abs(cmdWheelVelocity.GetElement(i, 0));
            }

            // Scale the applied unit velocity by the ratio of the maximum
            // required wheel speed to the maximum allowable wheel speed to
            // determine the maximum velocity in this direction
            vXMax = minScalingFactor;

            // Repeat the process of the y-direction
            cmdRobotMotion.SetElement(0, 0, 0.0);
            cmdRobotMotion.SetElement(1, 0, 1.0);
            cmdWheelVelocity = robotMatrix.Multiply(cmdRobotMotion);

            minScalingFactor = 0.0;
            for (i = 0; i < wheelList.Size(); i++)
            {
                if (wheelList.Get(i).GetMaxRotationRate() /
						Math.abs(cmdWheelVelocity.GetElement(i, 0)) <
                        minScalingFactor || minScalingFactor == 0.0)
                    minScalingFactor =
							wheelList.Get(i).GetMaxRotationRate() /
							Math.abs(cmdWheelVelocity.GetElement(i, 0));
            }

            vYMax = minScalingFactor;

            // And again for rotation
            cmdRobotMotion.SetElement(1, 0, 0.0);
            cmdRobotMotion.SetElement(2, 0, 1.0);
            cmdWheelVelocity = robotMatrix.Multiply(cmdRobotMotion);

            minScalingFactor = 0.0;
            for (i = 0; i < wheelList.Size(); i++)
            {
                if (wheelList.Get(i).GetMaxRotationRate() /
						Math.abs(cmdWheelVelocity.GetElement(i, 0)) <
                        minScalingFactor || minScalingFactor == 0.0)
                    minScalingFactor =
							wheelList.Get(i).GetMaxRotationRate() /
							Math.abs(cmdWheelVelocity.GetElement(i, 0));
            }

            omegaMax = minScalingFactor;

            // Re-zero the commanded motion matrix
            cmdRobotMotion.SetElement(2, 0, 0.0);

			// Print the maximum velocities for debugging purposes
			/*System.out.println("Max X velocity: " + vXMax);
			System.out.println("Max Y velocity: " + vYMax);
			System.out.println("Max Omega velocity: " + omegaMax);*/
        }

        return initialized;
    }

    /**
	 * Main driving method MUST be called at the rate specified when
	 * constructed.  This method is called by the other Drive() and DriveCoR()
	 * methods.  Generates wheel speed commands for each wheel and calls the
	 * appropriate methods to close the loops for each wheel.  Also updates the
	 * robot's velocity estimate based on wheel encoder feedback.
	 *
	 * @param vX	Desired x-velocity of the robot [in/sec]
	 * @param vY	Desired y-velocity of the robot [in/sec]
	 * @param omega	Desired rotational velocity of the robot [rad/sec]
	 *
	 * @throws IllegalStateException
	 */
    public void Drive(double vX, double vY, double omega)
            throws IllegalStateException
    {
        // Make sure we're ready to rock and roll
        if (!initialized)
            throw new IllegalStateException(
					"Error:  Object is not initialized!" +
					"  Use HolonomicRobotDrive.Initialize() prior to use." +
					"  Look for initialization errors in System.err");

        // Assign values to the commanded motion vector
        cmdRobotMotion.SetElement(0, 0, vX);
        cmdRobotMotion.SetElement(1, 0, vY);
        cmdRobotMotion.SetElement(2, 0, omega);

        // Solve for the required wheel velocities
        cmdWheelVelocity = robotMatrix.Multiply(cmdRobotMotion);// [rad/sec]

        // Check to see if the command needs to be scaled to avoid commanding
        // a speed in excess of the max allowable wheel speed
        if (RescaleVelocityCommand())
            // Recalculate the required wheel speeds
            cmdWheelVelocity = robotMatrix.Multiply(cmdRobotMotion);

        // Issue the velocity commands for each wheel
        // Velocities are in rad/sec at the wheel
        int i;
        for (i = 0; i < cmdWheelVelocity.GetRowCount(); i++)
        {
            // Apply velocity and acceleration limits to the wheel speed
            cmdWheelVelocity.SetElement(i, 0,
                    wheelSpeedLimiter[i].Process(
                    cmdWheelVelocity.GetElement(i, 0)));

            // Close the loop (or just issue the command if we're in open-loop
            // mode)
            ((Wheel)wheelList.Get(i)).DoControl(
                    cmdWheelVelocity.GetElement(i, 0));

            // Get the velocity feedback from the wheel's encoder and store it
            measuredWheelVelocity.SetElement(i, 0,
                    wheelList.Get(i).GetWheelVelocity());
        }

        // Calculate the estimated robot velocity based on the feedback from all
        // of the encoders
        estimatedRobotVelocity = inverseRobotMatrix.Multiply(
				measuredWheelVelocity);
    }

    /**
	 * Calls the main Drive() method with parameters adjusted to achieve the
	 * specified rotation rate about the specified point.
	 *
	 * @param corX	X-location of the center-of-rotation [in]
	 * @param corY	Y-location of the center-of-rotation [in]
	 * @param omega	Desired rotation rate [rad/sec]
	 *
	 * @throws IllegalStateException
	 */
    public void DriveCoR(double corX, double corY, double omega)
            throws IllegalStateException
    {
        // Transform into vX, Vy and omega with CoR = <0,0>
        Drive(omega * corY, -omega * corX, omega);
    }

    /**
	 * Calls the main Drive() method with speed commands adapted from the
	 * specified joystick inputs (single joystick version).
	 *
	 * @param stick	Joystick from which the X, Y and twist measurements are to
	 * be read
	 *
	 * @throws IllegalStateException
	 */
    public void Drive(JoystickWrapper stick)
            throws IllegalStateException
    {
		// NOTE:  Added yaw control 2/23/2012 to help fix weight distribution
		// problem.  Modifies yaw cmd prior to calculating wheel commands
		//double yawCmd = ApplyDeadband(stick.getTwist() * omegaMax);
		/*yawCmd = yawController.DoControl(yawCmd,
				-yawGyro.getRate() * Math.PI / 180.0);*/

        // Call the normal drive method with the joystick values
		// Joystick X axis is left-right, positive right (no change necessary)
		// Joystick Y axis if fore-aft, positive aft (requires sign change)
		// Joystick twist is positive nose right (requires sign change)
        /*Drive(yawCmd, ApplyDeadband(-stick.getY()) * vYMax,
				ApplyDeadband(-stick.getX()) * vXMax);*/
    	Drive(stick.GetRight() * vXMax, stick.GetForward() * vYMax, stick.GetTwist() * omegaMax);
    }

    /**
	 * Calls the main Drive() method with speed commands adapted from the
	 * specified joystick inputs (dual joystick version).
	 *
	 * @param leftStick		Joystick in the operator's left hand
	 * @param rightStick	Joystick in the operator's right hand
	 *
	 * @throws IllegalStateException
	 */
    public void Drive(Joystick leftStick, Joystick rightStick)
            throws IllegalStateException
    {
		// Compute the deadband-ed values from each stick
		// Joystick X axis is left-right, positive right (no change necessary)
		// Joystick Y axis if fore-aft, positive aft (requires sign change)
		double leftX = ApplyDeadband(leftStick.getX());
		double leftY = ApplyDeadband(-leftStick.getY());
		double rightX = ApplyDeadband(rightStick.getX());
		double rightY = ApplyDeadband(-rightStick.getY());

        // Call the normal drive method with the adapted joystick values
        Drive((leftX + rightX) / 2.0 * vXMax,
				(leftY + rightY) / 2.0 * vYMax,
                (rightY - leftY) / 2.0 * omegaMax);
    }

    /**
	 * Sets the surface friction coefficient (creates an acceleration limit for
	 * the robot).
	 *
	 * @param mu Coefficient of friction [-]
	 */
    public void SetFrictionCoefficient(double mu)
    {
        friction = mu;
    }

	/**
	 * Returns a 3x1 vector (Matrix) containing the robot's estimated velocity,
	 * as determined from wheel speed sensors.  Vector is <vX, vY, omega>.
	 *
	 * @return 3x1 Robot speed vector [in/sec] and [rad/sec]
	 */
	public Matrix GetEstimatedRobotVelocity()
	{
		return estimatedRobotVelocity;
	}
	
	/**
	 * Updates the robot position and veloctiy estimates without closing loops.
	 * Useful for "feedback-only" mode when using higher-level controller.
	 */
	public void UpdateEstimates()
	{
		if (!initialized)
            throw new IllegalStateException(
					"Error:  Object is not initialized!" +
					"  Use HolonomicRobotDrive.Initialize() prior to use.");
		
		// Issue the velocity commands for each wheel
        // Velocities are in rad/sec at the wheel
        int i;
        for (i = 0; i < measuredWheelVelocity.GetRowCount(); i++)
        {
        	wheelList.Get(i).UpdateWheelVelocity();
        	
            // Get the velocity feedback from the wheel's encoder and store it
            measuredWheelVelocity.SetElement(i, 0,
                    wheelList.Get(i).GetWheelVelocity());
        }

        // Calculate the estimated robot velocity based on the feedback from all
        // of the encoders
        estimatedRobotVelocity = inverseRobotMatrix.Multiply(
				measuredWheelVelocity);
        
        estimatedRobotPosition = estimatedRobotPosition.Add(estimatedRobotVelocity.Multiply(1.0 / freq));
	}
	
	public void ResetPositions(double x, double y, double theta)
	{
		estimatedRobotPosition.SetElement(0, 0, x);
		estimatedRobotPosition.SetElement(1, 0, y);
		estimatedRobotPosition.SetElement(2, 0, theta);
	}

	/**
	 * NOT YET IMPLEMENTED
	 * <p>
	 * The intent is to implement a dynamic model of the robot so predictions
	 * about the robot's state can be made by knowing only the PWM signals being
	 * sent to each wheel's Jaguar.
	 *
	 * @return NOT YET IMPLEMENTED
	 */
	public Matrix GetPredictedRobotVelocity()
	{
		return predictedRobotVelocity;
	}

	/**
	 * Sets a deadband for each joystick axis around the center position.
	 *
	 * @param _deadband	Value between 0 and 1 specifying the percent of stick
	 * travel (or twist) to set to zero [%]
	 *
	 * @throws IllegalArgumentException
	 */
	public void SetDeadband(double _deadband) throws IllegalArgumentException
	{
		if (_deadband < 0.0 || _deadband > 1.0)
			throw new IllegalArgumentException("Deadband must have value " +
					"between 0 and 1");

		deadband = _deadband;

		SetDBSlopeIntercept();
	}

	/**
	 * Sets a minimum output level for each motor (jump from zero to this level)
	 * For closed-loop systems, it is recommended that this be ZERO.
	 *
	 * @param _deadband	Value between 0 and 1 specifying the minimum voltage
	 * command [%]
	 *
	 * @throws IllegalArgumentException
	 */
	public void SetMinimumOutput(double _minOut) throws IllegalArgumentException
	{
		if (_minOut < 0.0 || _minOut > 1.0)
			throw new IllegalArgumentException("Deadband must have value " +
					"between 0 and 1");

		minimumOutput = _minOut;

		SetDBSlopeIntercept();
	}

	private void SetDBSlopeIntercept()
	{
		dbSlope = (1.0 - minimumOutput) / (1.0 - deadband);
		dbIntercept = 1.0 - dbSlope;
	}

	/**
	 * Returns the signal with the deadband applied.  Deadband is applied
	 * smoothly so that instead of a step at the edge of the deadband, the
	 * signal is interpolated linearly from the first "active" reading to the
	 * maximum joystick value.
	 *
	 * @param signal	Raw, unmodified signal [-1..1]
	 *
	 * @return Signal with deadband applied
	 */
	private double ApplyDeadband(double signal)
	{
		// Linearize the command so it is zero at +/- deadband, but changes
		// linearly to the extremes of travel (don't create a jump as we exit
		// the deadband)

		// This method is intended for use ONLY with variables varying from
		// -1.0 to 1.0

		if (Math.abs(signal) < deadband)
			return 0.0;

		/*if (signal > 0.0)
			return (signal - deadband) / (1.0 - deadband);
		else
			return (signal + deadband) / (1.0 - deadband);*/
		if (signal > 0.0)
			return dbSlope * signal + dbIntercept;
		else
			return dbSlope * signal - dbIntercept;
	}

    /**
	 * Rescales the velocity command vector so that no velocity exceeds the
	 * allowable command for the robot, but the relative proportions of the
	 * commands are preserved.
	 *
	 * @return True if the scaling factor was applied, false if application was
	 * unnecessary
	 */
    private boolean RescaleVelocityCommand()
    {
        double f = 1.0, speed;
        int i;

        // Determine the required scaling factor
        for (i = 0; i < wheelList.Size(); i++)
        {
            speed = Math.abs(cmdWheelVelocity.GetElement(i, 0));
            if (wheelList.Get(i).GetMaxRotationRate() / speed < f)
                f = wheelList.Get(i).GetMaxRotationRate() / speed;
        }

        // If the scaling factor less than one, apply it and return true
        if (f < 1.0)
        {
            cmdRobotMotion.SetElement(0, 0, cmdRobotMotion.GetElement(0,0) * f);
            cmdRobotMotion.SetElement(1, 0, cmdRobotMotion.GetElement(1,0) * f);
            cmdRobotMotion.SetElement(2, 0, cmdRobotMotion.GetElement(2,0) * f);

            return true;
        }

        // Otherwise, return false
        return false;
    }

	/**
	 * Returns the object representing the specified wheel.  May be useful for
	 * some tuning strategies.
	 *
	 * @param i	Index of the desired wheel
	 *
	 * @return Wheel at the specified location in the list
	 */
	public Wheel GetWheel(int i)
	{
		return wheelList.Get(i);
	}

	/**
	 * Resets the controllers for all of the wheels and resets the position
	 * estimate to zero.
	 */
    public void Reset()
    {
        int i;
        for (i = 0; i < wheelList.Size(); i++)
            wheelList.Get(i).Reset();

		SetPosition(0.0, 0.0, 0.0);
		ResetPositions(0.0, 0.0, 0.0);
    }

	/**
	 * Sets the current position estimate to the specified values.
	 *
	 * @param x		X-position [in]
	 * @param y		Y-position [in]
	 * @param theta	Theta position [deg]
	 */
	public void SetPosition(double x, double y, double theta)
	{
		//navigator.SetPosition(x, y, theta);
	}

	/**
	 * Returns the current X position estimate.
	 *
	 * @return Estimated X position [in]
	 */
	public double GetXPosition()
	{
		return estimatedRobotPosition.GetElement(0, 0);
	}

	/**
	 * Returns the current Y position estimate.
	 *
	 * @return Estimated Y position [in]
	 */
	public double GetYPosition()
	{
		return estimatedRobotPosition.GetElement(1, 0);
	}

	/**
	 * Returns the current theta estimate.
	 *
	 * @return Estimated theta [deg]
	 */
	public double GetThetaPosition()
	{
		return estimatedRobotPosition.GetElement(2, 0);
	}

	/**
	 * Returns the maximum allowable X velocity, calculated based on maximum
	 * allowable wheel speeds and geometry.
	 *
	 * @return Maximum allowable magnitude of robot X velocity [in/sec]
	 */
	public double GetMaxXVel()
	{
		return vXMax;
	}

	/**
	 * Returns the maximum allowable Y velocity, calculated based on maximum
	 * allowable wheel speeds and geometry.
	 *
	 * @return Maximum allowable magnitude of robot Y velocity [in/sec]
	 */
	public double GetMaxYVel()
	{
		return vYMax;
	}

	/**
	 * Returns the maximum allowable omega, calculated based on maximum
	 * allowable wheel speeds and geometry.
	 *
	 * @return Maximum allowable magnitude of robot rotation rate [deg/sec]
	 */
	public double GetMaxOmega()
	{
		return omegaMax;
	}

	/**
	 * Returns true if the robot's wheels are all below the speed at which they
	 * are to be considered stopped.
	 *
	 * @return True if the robot is not moving, false otherwise
	 */
	public boolean IsStopped()
	{
		int i;
		for (i = 0; i < wheelList.Size(); i++)
		{
			// Check each wheel speed agains the threshold
			if (!wheelList.Get(i).IsStopped())
				return false;
		}

		return true;
	}
	
	/**
	 * 
	 */
	public void DoSmartDashboardWheelPositions()
	{
		int i;
		for (i = 0; i < wheelList.Size(); i++)
			SmartDashboard.putNumber("angle (" + wheelList.Get(i).GetXPos() + ", " +
					wheelList.Get(i).GetYPos() + ") ", wheelList.Get(i).GetWheelPosition());
	}
	
	/**
	 * 
	 */
	public void DoSmartDashboardWheelVelocities()
	{
		int i;
		for (i = 0; i < wheelList.Size(); i++)
			SmartDashboard.putNumber("speed (" + wheelList.Get(i).GetXPos() + ", " +
					wheelList.Get(i).GetYPos() + ")", wheelList.Get(i).GetWheelVelocity());
	}
	
	public void UpdateGains(double kp, double ti)
	{
		int i;
		for (i = 0; i < wheelList.Size(); i++)
			wheelList.Get(i).UpdateGains(kp, ti);
	}
}