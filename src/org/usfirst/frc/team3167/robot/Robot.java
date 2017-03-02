
package org.usfirst.frc.team3167.robot;

import org.usfirst.frc.team3167.robot.drive.HolonomicDrive;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
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
    final String defaultAuto = "Default";
    final String customAuto = "My Auto";
    String autoSelected;
    SendableChooser chooser;
    
    static final private double robotFrequency = 50.0;// [Hz]
    
    private final HolonomicDrive drive = new HolonomicDrive(robotFrequency);
    
    private final Joystick stick = new Joystick(1);
    
    private Climber climber;
    private RobotConfiguration robotConfig;
    private Vision vision; 
    //private TestDriveHARDNUMBERS testDrive;
    private GearHanger gearHanger;
    
    private RobotDrive mDrive;
    
    static final private int encoderLeftRearA = 10;
    static final private int encoderLeftRearB = 11;
    static final private int encoderRightFrontA = 16;
    static final private int encoderRightFrontB = 17;
    static final private int encoderRightRearA = 12;
    static final private int encoderRightRearB = 13;
    static final private int encoderLeftFrontA = 14;
    static final private int encoderLeftFrontB = 15;
    
    static final private int motorLeftRearChannel = 1;
    static final private int motorRightFrontChannel = 2;
    static final private int motorRightRearChannel = 3;
    static final private int motorLeftFrontChannel = 4;
	
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
        /*chooser = new SendableChooser();
        chooser.addDefault("Default Auto", defaultAuto);
        chooser.addObject("My Auto", customAuto);
        SmartDashboard.putData("Auto choices", chooser);*/
   
    	climber = new Climber(1, 2, 5); 
    	robotConfig = new RobotConfiguration(); 
    	/*testDrive = new TestDriveHARDNUMBERS(1, 2, 3, 4,
    			motor1EncoderA, motor1EncoderB,
    			motor2EncoderA, motor2EncoderB,
    			motor3EncoderA, motor3EncoderB,
    			motor4EncoderA, motor4EncoderB);//*/
    	
    	mDrive = new RobotDrive(new Talon(motorLeftFrontChannel),
    			new Talon(motorLeftRearChannel),
    			new Talon(motorRightFrontChannel),
    			new Talon(motorRightRearChannel));
    	//InitializeHolonomicDrive();
    	gearHanger = new GearHanger(1, 2, 6, 8, 9); 
    	vision = new Vision("cam0");
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
//		autoSelected = SmartDashboard.getString("Auto Selector", defaultAuto);
		System.out.println("Auto selected: " + autoSelected);*/
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
    }
    
    public void teleopInit() {
    	drive.Reset(); 
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
    	//hard number drive based on button input (encoder testing)
    	//testDrive.hardNumDrive(0.8);
    	
    	
    	//mDrive.mecanumDrive_Cartesian(stick.getX() * 0.5, stick.getY() * 0.5, stick.getTwist() * 0.5, 0);
    	
    	// We should maybe use SecondOrderLimiters to prevent inputs from being too aggressive
    	//drive.Drive(stick.getX(), stick.getY(), stick.getTwist());
    	
    	//handle climber (with multiple speeds)
    	//could remove reverse spins (currently just a fail-safe)
    	climber.operate(); 
    	
    	gearHanger.hangGear(0.6);
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
    }
    
    private static Encoder CreateNewEncoder(int aChannel, int bChannel,
    		boolean reverse, EncodingType encoding, int pulsesPerRev)
    {
    	Encoder encoder = new Encoder(aChannel, bChannel, reverse, encoding);
    	encoder.setDistancePerPulse(360.0 / pulsesPerRev);
    	encoder.setSamplesToAverage(5);
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
    	
    	final double kp = 1.0;
    	final double ki = 0.0;
    	final double saturation = 0.0;
    	final double filterOmega = 10.0;// [Hz]
    	final double filterZeta = 1.0;
    	
    	final EncodingType encoding = EncodingType.k4X;
    	final int encoderPPR = 1024;
    	
    	// Left front (motor 4)
    	drive.AddWheel(-halfWidth, halfLength, -1.0, 0.0,
    			rollerAngle, radius, gearRatio, new Talon(motorLeftFrontChannel),
    			maxSpeed, CreateNewEncoder(encoderLeftFrontA, encoderLeftFrontB, false, encoding, encoderPPR),
    			kp, ki, saturation, filterOmega, filterZeta);
    	
    	// Right front (motor 2)
    	drive.AddWheel(halfWidth, halfLength, 1.0, 0.0,
    			-rollerAngle, radius, gearRatio, new Talon(motorRightFrontChannel),
    			maxSpeed, CreateNewEncoder(encoderRightFrontA, encoderRightFrontB, false, encoding, encoderPPR),
    			kp, ki, saturation, filterOmega, filterZeta);
    	
    	// Left rear (motor 1)
    	drive.AddWheel(-halfWidth, -halfLength, -1.0, 0.0,
    			-rollerAngle, radius, gearRatio, new Talon(motorLeftRearChannel),
    			maxSpeed, CreateNewEncoder(encoderLeftRearA, encoderLeftRearB, true, encoding, encoderPPR),
    			kp, ki, saturation, filterOmega, filterZeta);
    	
    	// Right rear (motor 3)
    	drive.AddWheel(halfWidth, -halfLength, 1.0, 0.0,
    			rollerAngle, radius, gearRatio, new Talon(motorRightRearChannel),
    			maxSpeed, CreateNewEncoder(encoderRightRearA, encoderRightRearB, true, encoding, encoderPPR),
    			kp, ki, saturation, filterOmega, filterZeta);
    			
    	return drive.Initialize();
    }
    
}