package org.usfirst.frc.team6193.robot.subsystems;

import org.usfirst.frc.team6193.robot.Robot;
import org.usfirst.frc.team6193.robot.RobotMap;
import org.usfirst.frc.team6193.robot.commands.DrivelineDefaultCommand;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.command.PIDSubsystem;

/**
 *
 */
public class Driveline extends PIDSubsystem {

	private RobotDrive m_robotDrive;
	private CANTalon m_leftFrontMotorController;
	private CANTalon m_rightFrontMotorController;
	private CANTalon m_rightRearMotorController;
	private CANTalon m_leftRearMotorController;
    // Initialize your subsystem here
    public Driveline() 
    {
    	super(1.0,0.0,0.0,0.02,0.0);

		m_leftFrontMotorController = new CANTalon(RobotMap.LEFT_FRONT_CONTROLLER_CANID);
		m_leftRearMotorController = new CANTalon(RobotMap.LEFT_REAR_CONTROLLER_CANID);
		m_rightRearMotorController = new CANTalon(RobotMap.RIGHT_REAR_CONTROLLER_CANID);
		m_rightFrontMotorController = new CANTalon(RobotMap.RIGHT_FRONT_CONTROLLER_CANID);
		
		m_robotDrive = new RobotDrive(m_leftRearMotorController, m_leftFrontMotorController, m_rightRearMotorController,m_rightFrontMotorController);

		m_leftFrontMotorController.setInverted(true);
		m_leftRearMotorController.setInverted(true);
		m_rightFrontMotorController.setInverted(true);
		m_rightRearMotorController.setInverted(true);
    }
    
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        setDefaultCommand(new DrivelineDefaultCommand());
    }
	public void Drive() {
		m_robotDrive.arcadeDrive(getY(Robot.oi.stickXbox), Robot.oi.stickXbox.getX() * RobotMap.DRIVELINE_TURN_SCALAR);
	}
    protected double returnPIDInput() {

    	return 0.0;
    }
    
    protected void usePIDOutput(double output) {

    }
	private double getY(Joystick stick){
		double Left = stick.getRawAxis(3);
		double Right = stick.getRawAxis(2);
		
		return Right - Left;
	}
}
