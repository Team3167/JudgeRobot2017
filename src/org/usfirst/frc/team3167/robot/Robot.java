
package org.usfirst.frc.team3167.robot;

import org.usfirst.frc.team3167.autonomous.DriveStraightAuto;
import org.usfirst.frc.team3167.autonomous.Vision;
import org.usfirst.frc.team3167.objectcontrol.Climber;
import org.usfirst.frc.team3167.objectcontrol.GearHanger;
import org.usfirst.frc.team3167.robot.drive.HolonomicDrive;
import org.usfirst.frc.team3167.robot.drive.SimpleMecanumDrive;
import org.usfirst.frc.team3167.robot.util.JoystickButton;
import org.usfirst.frc.team3167.robot.util.JoystickWrapper;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Talon;
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
    //SendableChooser chooser;
    
    static final private double robotFrequency = 50.0;// [Hz]
    
    private final boolean useSimpleDrive = true;
    private SimpleMecanumDrive mecanumDrive;
    //private RobotDrive d;
    
    private final HolonomicDrive drive = new HolonomicDrive(robotFrequency);
    
    //private final JoystickWrapper stick = new JoystickWrapper(1); 
    private Joystick stick;
    private Joystick stick2; 
    
    private Climber climber;
    private RobotConfiguration robotConfig;
    private Vision vision; 
    private GearHanger gearHanger;
    private DriveStraightAuto auto; 
    
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
    
    static final private double autoDriveTime = -4.0; //[sec]
    static final private double autoDriveSpeed = 0.85;  
    
    private boolean slideLocked = false; 
	
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
        /*chooser = new SendableChooser();
        chooser.addDefault("gearCam", gearCamLoc);
        chooser.addObject("climberCam", climberCamLoc); */
    	
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
    	else
    		InitializeHolonomicDrive();
    	
    	gearHanger = new GearHanger(1, 2, 6, 8, 9); 
    	
    	auto = new DriveStraightAuto(mecanumDrive, autoDriveSpeed, autoDriveTime);
    	
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
    	auto.resetTime();
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
    	
    	auto.execute(); 
    }
    
    public void teleopInit() {
    	drive.Reset();
    	//vision.enable(0);
    }

    /**
     * This function is called periodically during operator control
     */
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
    	
    	//handle climber (with multiple speeds)
    	//could remove reverse spins (currently just a fail-safe)
    	climber.operate(); 
    	
    	gearHanger.hangGear(0.7);
    	    	
    	//testDrive.sendDistToDash();
    }
    
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
    			rollerAngle, radius, gearRatio, new Talon(motorLeftFrontChannel),
    			maxSpeed, CreateNewEncoder(encoderLeftFrontA, encoderLeftFrontB, false, encoding, encoderPPR),
    			kp, ti, saturation, filterOmega, filterZeta);
    	
    	// Right front (motor 4)
    	drive.AddWheel(halfWidth, halfLength, 1.0, 0.0,
    			-rollerAngle, radius, gearRatio, new Talon(motorRightFrontChannel),
    			maxSpeed, CreateNewEncoder(encoderRightFrontA, encoderRightFrontB, false, encoding, encoderPPR),
    			kp, ti, saturation, filterOmega, filterZeta);
    	
    	// Left rear (motor 1)
    	drive.AddWheel(-halfWidth, -halfLength, -1.0, 0.0,
    			-rollerAngle, radius, gearRatio, new Talon(motorLeftRearChannel),
    			maxSpeed, CreateNewEncoder(encoderLeftRearA, encoderLeftRearB, false, encoding, encoderPPR),
    			kp, ti, saturation, filterOmega, filterZeta);
    	
    	// Right rear (motor 3)
    	drive.AddWheel(halfWidth, -halfLength, 1.0, 0.0,
    			rollerAngle, radius, gearRatio, new Talon(motorRightRearChannel),
    			maxSpeed, CreateNewEncoder(encoderRightRearA, encoderRightRearB, false, encoding, encoderPPR),
    			kp, ti, saturation, filterOmega, filterZeta);
    	
    	drive.SetDeadband(0.05);
    	drive.SetFrictionCoefficient(1.0);
    	drive.SetMinimumOutput(0.0);
    	drive.SetPosition(0.0, 0.0, 0.0);
    			
    	return drive.Initialize();
    }
    
}