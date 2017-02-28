
package org.usfirst.frc.team3167.robot;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
    
    //private Joystick stick;
    
    private Climber climber;
    private RobotConfiguration robotConfig;
    private TestDriveHARDNUMBERS testDrive;
    private GearHanger gearHanger;
    
    static final private int motor1EncoderA = 10;
    static final private int motor1EncoderB = 11;
    static final private int motor2EncoderA = 16;
    static final private int motor2EncoderB = 17;
    static final private int motor3EncoderA = 12;
    static final private int motor3EncoderB = 13;
    static final private int motor4EncoderA = 14;
    static final private int motor4EncoderB = 15;
	
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
        /*chooser = new SendableChooser();
        chooser.addDefault("Default Auto", defaultAuto);
        chooser.addObject("My Auto", customAuto);
        SmartDashboard.putData("Auto choices", chooser);*/
    	
    	//stick = new Joystick(1);
    	climber = new Climber(1, 5); 
    	robotConfig = new RobotConfiguration(); 
    	testDrive = new TestDriveHARDNUMBERS(1, 2, 3, 4,
    			motor1EncoderA, motor1EncoderB,
    			motor2EncoderA, motor2EncoderB,
    			motor3EncoderA, motor3EncoderB,
    			motor4EncoderA, motor4EncoderB);
    	gearHanger = new GearHanger(1, 6, 8, 9); 
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
    	testDrive.resetEncoders(); 
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {     	
    	//hard number drive based on button input (encoder testing)
    	testDrive.hardNumDrive(0.8);
    	
    	//drive.mecanumDrive_Cartesian(stick.getX(), stick.getY(), stick.getTwist(), 0);
    	
    	//handle climber (with multiple speeds)
    	//could remove reverse spins (currently just a fail-safe)
    	climber.operate(); 
    	
    	gearHanger.hangGear();
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
    
}