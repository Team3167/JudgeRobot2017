
package org.usfirst.frc.team3167.robot;

import org.usfirst.frc.team3167.autonomous.DriveStraightAuto;
import org.usfirst.frc.team3167.autonomous.JudgeCommand;
import org.usfirst.frc.team3167.autonomous.Networking;
import org.usfirst.frc.team3167.autonomous.Vision;
import org.usfirst.frc.team3167.objectcontrol.Climber;
import org.usfirst.frc.team3167.objectcontrol.GearHanger;
import org.usfirst.frc.team3167.robot.drive.HolonomicDrive;
import org.usfirst.frc.team3167.robot.drive.HolonomicPositioner;
import org.usfirst.frc.team3167.robot.drive.RobotPosition;
import org.usfirst.frc.team3167.robot.drive.SimpleMecanumDrive;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
    //String climberCamLoc, gearCamLoc, finalCam;
    SendableChooser<Command> chooser;
    
    static final private double robotFrequency = 50.0;// [Hz]
    
    private final boolean useSimpleDrive = true;
    private SimpleMecanumDrive mecanumDrive;
    //private RobotDrive d;
    
    private final int listenForRPiOnPort = 5801;
    private Networking rpiInterface;
    
    private HolonomicPositioner positioner;
    private final HolonomicDrive drive = new HolonomicDrive(robotFrequency);
    
    //private final JoystickWrapper stick = new JoystickWrapper(1); 
    private Joystick stick;
    private Joystick stick2; 
    
    private Climber climber;
    private RobotConfiguration robotConfig;
    private Vision vision; 
    private GearHanger gearHanger;
    private DriveStraightAuto auto; 
    
    private JudgeCommand straightAuto;
    private JudgeCommand gearAuto;
    
    static final private int encoderLeftFrontA = 16;
    static final private int encoderLeftFrontB = 17;
    static final private int encoderRightFrontA = 14;
    static final private int encoderRightFrontB = 15;
    static final private int encoderLeftRearA = 10;
    static final private int encoderLeftRearB = 11;
    static final private int encoderRightRearA = 12;
    static final private int encoderRightRearB = 13;
    
    static final private int motorLeftFrontChannel = 2;
    static final private int motorLeftRearChannel = 1;
    static final private int motorRightFrontChannel = 4;
    static final private int motorRightRearChannel = 3; 
    
    static final private int pulsesPerRev = 1024; 
    
    private boolean slideLocked = false; 
    
    private boolean autoState; 
	
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
        chooser = new SendableChooser<Command>();
        
        straightAuto = new JudgeCommand("straightAuto");
        gearAuto = new JudgeCommand("gearAuto");
        
        chooser.addDefault("straightAuto", straightAuto);
        chooser.addObject("gearAuto", gearAuto); 
    	
    	stick = new Joystick(1); 
    	stick2 = new Joystick(2); 
    	
    	climber = new Climber(1, 2, 5); 
    	robotConfig = new RobotConfiguration(); 
 
    	if (useSimpleDrive)
    		mecanumDrive = new SimpleMecanumDrive(
    				new Talon(motorLeftFrontChannel),
    				new Talon(motorLeftRearChannel),
    				new Talon(motorRightFrontChannel),
    				new Talon(motorRightRearChannel));

    	try
    	{
    		rpiInterface = new Networking(listenForRPiOnPort);
    	}
    	catch (Exception e)
    	{
    		System.out.println(e.toString());
    	}
    	
    	InitializeHolonomicDrive();
    	positioner = new HolonomicPositioner(drive, robotFrequency);
    	
    	gearHanger = new GearHanger(1, 2, 6, 8, 9); 
    	
    	auto = new DriveStraightAuto(mecanumDrive, gearHanger, drive); 
    			
    	
    	//joystick, gearPort, climberPort
    	vision = new Vision(stick, 0, 1);
    	vision.enable();
    }
    
	/**
	 * This autonomous (along with the chooser code above) shows how to select between different autonomous modes
	 * using the dashboard. The sendable chooser code works with the Java SmartDashboard. If you prefer the LabVIEW
	 * Dashboard, remove all of the chooser code and uncomment the getString line to get the auto name from the text box
	 * below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the switch structure below with additional strings.
	 * If using the SendableChooser make sure to add them to the chooser code above as well.
	 */
    public void autonomousInit() {
    	/*autoSelected = (String) chooser.getSelected();
		//autoSelected = SmartDashboard.getString("Auto Selector", defaultAuto);
		System.out.println("Auto selected: " + autoSelected);*/
    	
    	//auto = new DriveStraightAuto(mecanumDrive, autoDriveSpeed, autoDriveTime);
    	
    	Command autoCmd = (Command) chooser.getSelected();
    	
    	if(autoCmd.getName() == "straightAuto") {
    		autoState = true; 
    	}
    	else if(autoCmd.getName() == "gearAuto") {
    		autoState = false; 
    	}
    	
    	auto.resetTime();
    	
    	//autoState = chooser.getSelected(); 
    	//if(autoState)
    }

    /**
     * This function is called periodically during autonomous.
     */
    public void autonomousPeriodic() {
    	/*switch(autoSelected) {
    	case customAuto:
        //Put custom auto code here   
            break;
    	case defaultAuto:
    	default:
    	//Put default auto code here
            break;
    	}*/
    	
    	drive.UpdateEstimates();
    	
    	auto.execute(autoState); 
    }
    
    public void teleopInit() {
    	drive.Reset();
    	//vision.enable(0);
    }

    /**
     * This function is called periodically during operator control
     */
/*<<<<<<< HEAD53
    public void teleopPeriodic() {
        if (rpiInterface.GotPositionUpdate())
        {
            // TODO:  This is where we need to put code to automatically steer
            // based on the position data we get back
            Networking.RobotPosition pos = rpiInterface.GetLatestPosition();
            SmartDashboard.putNumber("image x: ", pos.x);
        	SmartDashboard.putNumber("image y: ", pos.y);
        	SmartDashboard.putNumber("image theta: ", pos.theta);
        }
        
        // This stuff could be put in autoInit() and autoPeriodic() as well
        if (stick.AutoDriveToGearPeg())
        	BeginDriveToTarget();
        
        if (driveToTarget)
        	DriveToTarget();
        else
        {
	    	if (useSimpleDrive)
	    	{
	    		//SmartDashboard.putNumber("Right: ", stick.GetRight());
	        	//SmartDashboard.putNumber("Forward: ", stick.GetForward());
	        	//SmartDashboard.putNumber("Twist: ", stick.GetTwist());
	    		mecanumDrive.Drive(-stick.GetRight(), -stick.GetForward(), stick.GetTwist());
	    	}
	    	else
	    	{
	    		// We should maybe use SecondOrderLimiters to prevent inputs from being too aggressive
	    		drive.Drive(stick);
	    		drive.DoSmartDashboardWheelVelocities();
	    	
	    		drive.UpdateGains(Preferences.getInstance().getDouble("kp", 0.02),
	    			Preferences.getInstance().getDouble("ti", 1.0));
	    		//prefs.getDouble("filter", 5.0);
	    	}
        }
=======*/
    public void teleopPeriodic() {    	
    	if (useSimpleDrive)
    	{
    		/*SmartDashboard.putNumber("Right: ", stick.GetRight());
        	SmartDashboard.putNumber("Forward: ", stick.GetForward());
        	SmartDashboard.putNumber("Twist: ", stick.GetTwist());//*/
    		
    		if(stick.getRawButton(2))
    			slideLocked = true; 
    		else
    			slideLocked = false; 
    			
    		//mecanumDrive.Drive(-stick.GetRight(), -stick.GetForward(), stick.GetTwist(), slideLocked);	
    		mecanumDrive.Drive(-stick.getX(), stick.getY(), -stick.getTwist(), slideLocked);
    	}
    	else
    	{
    		// We should maybe use SecondOrderLimiters to prevent inputs from being too aggressive
    		//drive.Drive(stick);
    		drive.DoSmartDashboardWheelVelocities();
    	
    		drive.UpdateGains(Preferences.getInstance().getDouble("kp", 0.02),
    			Preferences.getInstance().getDouble("ti", 1.0));
    		//prefs.getDouble("filter", 5.0);
    	}
//>>>>>>> Westtown
    	
    	//handle climber (with multiple speeds)
    	//could remove reverse spins (currently just a fail-safe)
    	climber.operate(); 
    	
    	gearHanger.hangGear(0.7);
    	    	
    	//testDrive.sendDistToDash();
    }
    
    // TODO:  Clean this up, make a new class
    boolean driveToTarget = false;
    RobotPosition targetPosition = new RobotPosition();
    
    private void BeginDriveToTarget()
    {
    	// TODO:  We should check to make sure we can see the target, otherwise we don't know how to start...
    	driveToTarget = true;
    	rpiInterface.FlushBuffer();
    	
    	targetPosition.x = 0.0;
    	targetPosition.y = 0.0;
    	targetPosition.theta = 0.0;
    	
    	positioner.ResetControllers();
    }
     
    private void DriveToTarget()
    {
    	if (!driveToTarget)
    		return;
    	
    	if (rpiInterface.GotPositionUpdate())
    		targetPosition = rpiInterface.GetLatestPosition();
    	
    	positioner.SetTargetPosition(targetPosition.x, targetPosition.y, targetPosition.theta);
    	RobotPosition command = positioner.Update();
    	
    	if (useSimpleDrive)
    		mecanumDrive.Drive(-command.x, -command.y, command.theta, false);
    	else
    		drive.Drive(command.x, command.y, command.theta);
    	
    	if (positioner.AtTargetPosition())
    		driveToTarget = false;
    }
    ////////////// End move to new class
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    
    }
    
    public void disabledInit() {
    	System.out.println("The robot is ready to rock and roll!");
    }
    
    public void disabledPeriodic() {
    	// Uncomment for testing the encoders - prints wheel angles to dashboard
    	// Wheels are identified with (x,y) position; angles are in degrees
    	// Can manually move each wheel to ensure:
    	// 1.  Encoders are associated with the correct wheel
    	// 2.  Positive direction for each encoder is correct
    	// 3.  Gear ratios are correct
    	//drive.DoSmartDashboardWheelPositions();
    	
    	// For checking signs for joystick inputs:
    	/*SmartDashboard.putNumber("Right: ", stick.GetRight());
    	SmartDashboard.putNumber("Forward: ", stick.GetForward());
    	SmartDashboard.putNumber("Twist: ", stick.GetTwist());//*/
    }
    
    private static Encoder CreateNewEncoder(int aChannel, int bChannel,
    		boolean reverse, EncodingType encoding, int pulsesPerRev)
    {
    	Encoder encoder = new Encoder(aChannel, bChannel, reverse, encoding);
    	encoder.setDistancePerPulse(360.0 / pulsesPerRev);
    	//encoder.setSamplesToAverage(5);
    	return encoder;
    }
    
    private Talon CreateNewMotor(int channel)
    {
    	if (useSimpleDrive)
    		return null;
    	
    	return new Talon(channel);
    }
    
    private boolean InitializeHolonomicDrive()
    {
    	// X is positive right
    	// Y is positive forward
    	final double halfWidth = 22.5 * 0.5;// [in]
    	final double halfLength = 13.0 * 0.5;// [in]
    	final double gearRatio = 9.0;
    	final double rollerAngle = 45.0;// [deg]
    	final double radius = 4.0;// [in]
    	final double maxMotorSpeed = 5000.0;// [RPM]
    	final double maxSpeed = maxMotorSpeed / gearRatio * 2.0 * Math.PI / 60.0;// [rad/sec]
    	
    	final double kp = 0.02;
    	final double ti = 1.0;
    	final double saturation = 0.0;
    	final double filterOmega = 10.0;// [Hz]
    	final double filterZeta = 1.0;
    	
    	final EncodingType encoding = EncodingType.k4X;
    	final int encoderPPR = 1024;
    	
    	// Left front (motor 2)
    	drive.AddWheel(-halfWidth, halfLength, -1.0, 0.0,
    			rollerAngle, radius, gearRatio, CreateNewMotor(motorLeftFrontChannel),
    			maxSpeed, CreateNewEncoder(encoderLeftFrontA, encoderLeftFrontB, false, encoding, encoderPPR),
    			kp, ti, saturation, filterOmega, filterZeta);
    	
    	// Right front (motor 4)
    	drive.AddWheel(halfWidth, halfLength, 1.0, 0.0,
    			-rollerAngle, radius, gearRatio, CreateNewMotor(motorRightFrontChannel),
    			maxSpeed, CreateNewEncoder(encoderRightFrontA, encoderRightFrontB, false, encoding, encoderPPR),
    			kp, ti, saturation, filterOmega, filterZeta);
    	
    	// Left rear (motor 1)
    	drive.AddWheel(-halfWidth, -halfLength, -1.0, 0.0,
    			-rollerAngle, radius, gearRatio, CreateNewMotor(motorLeftRearChannel),
    			maxSpeed, CreateNewEncoder(encoderLeftRearA, encoderLeftRearB, false, encoding, encoderPPR),
    			kp, ti, saturation, filterOmega, filterZeta);
    	
    	// Right rear (motor 3)
    	drive.AddWheel(halfWidth, -halfLength, 1.0, 0.0,
    			rollerAngle, radius, gearRatio, CreateNewMotor(motorRightRearChannel),
    			maxSpeed, CreateNewEncoder(encoderRightRearA, encoderRightRearB, false, encoding, encoderPPR),
    			kp, ti, saturation, filterOmega, filterZeta);
    	
    	drive.SetDeadband(0.05);
    	drive.SetFrictionCoefficient(1.0);
    	drive.SetMinimumOutput(0.0);
    	drive.SetPosition(0.0, 0.0, 0.0);
    			
    	return drive.Initialize();
    }
    
}