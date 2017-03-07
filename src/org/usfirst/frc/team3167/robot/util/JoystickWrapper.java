package org.usfirst.frc.team3167.robot.util;

import edu.wpi.first.wpilibj.Joystick;

// This class provides a means of giving joystick inputs more meaning.
public class JoystickWrapper {
	
	private final Joystick stick;
	private final JoystickButton driveToGearPegButton;
	
	public JoystickWrapper(int stickID)
	{
		stick = new Joystick(stickID);
		driveToGearPegButton = new JoystickButton(stick, 8);
	}
	
	public double GetForward()
	{
		return -stick.getY();
	}
	
	public double GetRight()
	{
		return stick.getX();
	}
	
	public double GetTwist()
	{
		return -stick.getTwist();
	}
	
	public boolean AutoDriveToGearPeg()
	{
		driveToGearPegButton.HasJustBeenPressed();
	}
}
