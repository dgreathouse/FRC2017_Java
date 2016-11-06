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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is responsible for all calls that will eventually make the
 * 4 motors driving the robot turn. In other words this class holds all functionality
 * that makes the robot move in any direction.
 */
public class Driveline extends PIDSubsystem {

	private RobotDrive m_robotDrive;
	private CANTalon m_leftFrontMotorController;
	private CANTalon m_rightFrontMotorController;
	private CANTalon m_rightRearMotorController;
	private CANTalon m_leftRearMotorController;
	
	private ADXRS450_Gyro gyro;
	private DrivelinePIDMode m_drivelinePIDMode;
    // The main constructor for the Driveline class
    public Driveline() 
    {
    	/* Since this class was extended from PIDSubsystem and PIDSubsystem does not 
    	 * have a default constructor with no arguments, we must call one of the PIDSubsystems
    	 * constructors. This call sets the PID control loop to 20ms */
    	super(1.0,0.0,0.0,0.02,0.0);

    	// Create new motor control objects
		m_leftFrontMotorController = new CANTalon(RobotMap.DRIVELINE_LEFT_FRONT_MOTORCONTROLLER_CANID);
		m_leftRearMotorController = new CANTalon(RobotMap.DRIVELINE_LEFT_REAR_MOTORCONTROLLER_CANID);
		m_rightRearMotorController = new CANTalon(RobotMap.DRIVELINE_RIGHT_REAR_MOTORCONTROLLER_CANID);
		m_rightFrontMotorController = new CANTalon(RobotMap.DRIVELINE_RIGHT_FRONT_MOTORCONTROLLER_CANID);
		
		// Invert the motors since all motors were wired backwards
		m_leftFrontMotorController.setInverted(true);
		m_leftRearMotorController.setInverted(true);
		m_rightFrontMotorController.setInverted(true);
		m_rightRearMotorController.setInverted(true);
		
		// Set the encoder counts per revolution
		//m_leftFrontMotorController.configEncoderCodesPerRev(20);
		m_rightFrontMotorController.configEncoderCodesPerRev(20);
		
		// Create a RobotDrive object with the new motor control objects
		m_robotDrive = new RobotDrive(m_leftRearMotorController, m_leftFrontMotorController, m_rightRearMotorController,m_rightFrontMotorController);
		m_robotDrive.setSafetyEnabled(false);
		
		//m_leftFrontMotorController.setEncPosition(0);
		m_rightFrontMotorController.setEncPosition(0);
		
		m_drivelinePIDMode = DrivelinePIDMode.MOVE; 
		
		// Create a new Gyro to get angle.
		// This is the gyro plugged into the SPI port in the top right of the RoboRIO
		gyro = new ADXRS450_Gyro();
    }
    
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        setDefaultCommand(new DrivelineDefaultCommand());
    }
    /**
     * Arcade drive style with the Left Trigger as brake, Right Trigger as throttle and left thumb stick as rotate
     * Default scalar on rotate is applied to stop the robot from rotating quickly. 
     */
	public void Drive() {
		SmartDashboard.putNumber("DrivelineDistance", getDrivelineDistance());
		m_robotDrive.arcadeDrive(getY(Robot.oi.stickXbox), Robot.oi.stickXbox.getX() * RobotMap.DRIVELINE_TURN_SCALAR);
		SmartDashboard.putNumber("DrivelineRotateOutput", Robot.oi.stickXbox.getX());
		SmartDashboard.putNumber("DrivelineMoveOutput", getY(Robot.oi.stickXbox));
		SmartDashboard.putNumber("DrivelineAngle", gyro.getAngle());
	}
	/** Drive the robot with move and rotate commands in arcadeDrive mode
	 * 
	 * @param move
	 * @param rotate
	 */
	public void Drive(double move, double rotate) {
		SmartDashboard.putNumber("DrivelineDistance", getDrivelineDistance());
		SmartDashboard.putNumber("DrivelineAngle", gyro.getAngle());
		SmartDashboard.putNumber("DrivelineRotateOutput", rotate);
		SmartDashboard.putNumber("DrivelineMoveOutput", move);
		m_robotDrive.arcadeDrive(move,rotate);
	}
	/** Drive the robot at a set magnitude and curve
	 * 
	 * @param magnitude
	 * @param curve
	 */
	public void DriveMagnitudeCurve(double magnitude, double curve){
		m_robotDrive.drive(magnitude, curve);
	}
	/** Return the PID sensor value that is used for control
	 * This method is called periodically by the PIDController to get the 
	 * value of the sensor that is being used for the PIDController.
	 * 
	 */
    protected double returnPIDInput() {
    	if(m_drivelinePIDMode == DrivelinePIDMode.MOVE){
    		return getDrivelineDistance();
    	}else {
    		return gyro.getAngle();
    	}
    
    }
    /** Use the PIDController output value to drive the robot.
     * Once the PIDController has calculated a value the output must be 
     * used to move or rotate the robot
     * 
     */
    protected void usePIDOutput(double output) {
    	if(m_drivelinePIDMode == DrivelinePIDMode.MOVE){
    		Drive(-output,0.0);
    	}else {
    		Drive(0.0,output);
    	}
    }
    /** Set the PIDMode, configure PID settings and enable the PIDController
     * 
     * @param mode
     * @param p
     * @param i
     * @param d
     * @param f
     */
    public void setPIDMode(DrivelinePIDMode mode, double p, double i, double d)
    {
    
    	m_drivelinePIDMode = mode;					// Set the local PID Mode value
    	getPIDController().reset();					// Clear any PIDs that might be running on the driveline
    	resetDrivelineDistance();					// Reset Driveline distance to 0.0. All Commands assume starting at 0.0
    	gyro.reset();								// Reset Gyro so the heading is 0.0. All repeated Rotate commands start from current as 0.0
    	getPIDController().setPID(p,i,d);			// Set the PID controller PID values
    	getPIDController().setToleranceBuffer(24);	// Use a average buffer size of 4 to calculate OnTarget
    	enable();									// Enable the PID to start running.
    }
    /** Get the move value from the joystick
     * 
     * @param stick
     * @return
     */
	private double getY(Joystick stick){
		double Left = stick.getRawAxis(3);
		double Right = stick.getRawAxis(2);
		
		return Right - Left;
	}
	/**
	 * Reset the Driveline encoders to 0
	 * 
	 */
	private void resetDrivelineDistance()
	{
		m_rightFrontMotorController.setPosition(0.0);
	}
	public double getDrivelineDistance()
	{
		return m_rightFrontMotorController.getEncPosition() * RobotMap.DRIVELINE_ENCODER_INCHPERCNT;
	}
	private double getDrivelineSpeed(){
		return m_leftFrontMotorController.getSpeed();
	}
	private double getDrivelineCurrent()
	{
		return m_rightFrontMotorController.getOutputCurrent() + m_leftFrontMotorController.getOutputCurrent()
				+ m_rightRearMotorController.getOutputCurrent() + m_leftRearMotorController.getOutputCurrent();
	}
}
