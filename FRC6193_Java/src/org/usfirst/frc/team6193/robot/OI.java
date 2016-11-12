package org.usfirst.frc.team6193.robot;


import org.usfirst.frc.team6193.robot.commands.SpinnersDefaultCommand;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;


/** Operator Interface 
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	public Joystick stickXbox = new Joystick(0);
	public Joystick stickFlight = new Joystick(0);
	
	public boolean isPIDTuningDrivelineMove = false;
	public boolean isPIDTuningDrivelineRotate = false;
	public boolean isPIDTuningDrivelineMagCurve = true;
	
	public JoystickButton flightButton_1;
	public JoystickButton flightButton_2;
	public JoystickButton flightButton_3;
	public JoystickButton flightButton_4;
	OI(){
		flightButton_1 = new JoystickButton(stickFlight,1);
		flightButton_1.whenPressed(new SpinnersDefaultCommand("OUT_HI",1));
		
		flightButton_2 = new JoystickButton(stickFlight,2);
		flightButton_2.whenPressed(new SpinnersDefaultCommand("OFF",0));
		
		flightButton_3 = new JoystickButton(stickFlight,3);
		flightButton_3.whenPressed(new SpinnersDefaultCommand("IN_HI",-1));
		
		flightButton_4 = new JoystickButton(stickFlight,4);
		flightButton_4.whenPressed(new SpinnersDefaultCommand("IN_HI1",-1));
	}

	
}

