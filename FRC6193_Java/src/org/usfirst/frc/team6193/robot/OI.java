package org.usfirst.frc.team6193.robot;


import edu.wpi.first.wpilibj.Joystick;


/** Operator Interface 
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	public Joystick stickXbox = new Joystick(0);
	
	public boolean isPIDTuningDrivelineMove = true;
	public boolean isPIDTuningDrivelineRotate = false;
	public boolean isPIDTuningDrivelineMagCurve = false;
	
	
}

