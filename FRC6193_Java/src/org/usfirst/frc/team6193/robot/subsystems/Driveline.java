package org.usfirst.frc.team6193.robot.subsystems;

import org.usfirst.frc.team6193.robot.Robot;
import org.usfirst.frc.team6193.robot.RobotMap;
import org.usfirst.frc.team6193.robot.commands.DrivelineDefaultCommand;
import org.usfirst.frc.team6193.robot.lib.DrivelinePIDMode;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
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
	
	private ADXRS450_Gyro gyro;
	private DrivelinePIDMode m_drivelinePIDMode;
    // Initialize your subsystem here
    public Driveline() 
    {
    	super(1.0,0.0,0.0,0.02,0.0);

		m_leftFrontMotorController = new CANTalon(RobotMap.DRIVELINE_LEFT_FRONT_MOTORCONTROLLER_CANID);
		m_leftRearMotorController = new CANTalon(RobotMap.DRIVELINE_LEFT_REAR_MOTORCONTROLLER_CANID);
		m_rightRearMotorController = new CANTalon(RobotMap.DRIVELINE_RIGHT_REAR_MOTORCONTROLLER_CANID);
		m_rightFrontMotorController = new CANTalon(RobotMap.DRIVELINE_RIGHT_FRONT_MOTORCONTROLLER_CANID);
		
		m_robotDrive = new RobotDrive(m_leftRearMotorController, m_leftFrontMotorController, m_rightRearMotorController,m_rightFrontMotorController);

		m_leftFrontMotorController.setInverted(true);
		m_leftRearMotorController.setInverted(true);
		m_rightFrontMotorController.setInverted(true);
		m_rightRearMotorController.setInverted(true);
		
		gyro = new ADXRS450_Gyro();
		
		m_drivelinePIDMode = DrivelinePIDMode.MOVE; 
    }
    
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        setDefaultCommand(new DrivelineDefaultCommand());
    }
	public void Drive() {
		m_robotDrive.arcadeDrive(getY(Robot.oi.stickXbox), Robot.oi.stickXbox.getX() * RobotMap.DRIVELINE_TURN_SCALAR);
	}
	/** Drive the robot with move and rotate commands
	 * 
	 * @param move
	 * @param rotate
	 */
	public void Drive(double move, double rotate) {
		m_robotDrive.arcadeDrive(move,rotate);
	}
	public void DriveMagnitudeCurve(double magnitude, double curve){
		m_robotDrive.drive(magnitude, curve);
	}
    protected double returnPIDInput() {
    	if(m_drivelinePIDMode == DrivelinePIDMode.MOVE){
    		return m_rightFrontMotorController.getEncPosition() * RobotMap.DRIVELINE_ENCODER_INCHPERCNT;
    	}else {
    		gyro.getAngle();
    	}
    	return 0.0;
    }
    
    protected void usePIDOutput(double output) {
    	if(m_drivelinePIDMode == DrivelinePIDMode.MOVE){
    		Drive(output,0.0);
    	}else {
    		Drive(0.0,output);
    	}
    }
    public void setPIDMode(DrivelinePIDMode mode, double p, double i, double d, double f)
    {
    	m_drivelinePIDMode = mode;
    	getPIDController().reset();
    	resetDrivelineDistance();
    	getPIDController().setPID(p,i,d,f);
    	getPIDController().setToleranceBuffer(4);
    	enable();
    }
	private double getY(Joystick stick){
		double Left = stick.getRawAxis(3);
		double Right = stick.getRawAxis(2);
		
		return Right - Left;
	}
	private void resetDrivelineDistance()
	{
		m_rightFrontMotorController.setPosition(0.0);
	}
}
