package org.usfirst.frc.team6193.robot.commands;

import org.usfirst.frc.team6193.robot.Robot;
import org.usfirst.frc.team6193.robot.lib.DrivelinePIDMode;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class will rotate the robot using a PID control method.
 */
public class DrivelineRotateCommand extends Command {
	private double m_angle = 0.0;
	private double m_maxTimeout = 0.0;
	private double m_speed = 0.0;
	private double m_percentTolerance = 0.0;
	private double m_p = 0.0;
	private double m_i = 0.0;
	private double m_d = 0.0;
	private double m_f = 0.0;
	/** Rotate the robot
	 * 
	 * @param angle The angle to rotate
	 * @param speed The speed of rotation
	 * @param timeout Timeout in seconds if target not reached
	 * @param percentTolerance Percent of tolerance of angle
	 * @param p
	 * @param i
	 * @param d
	 * @param f
	 */
    public DrivelineRotateCommand(double angle, double speed, double timeout, double percentTolerance, double p, double i, double d, double f) {
    	m_angle = angle;
    	m_maxTimeout = timeout;
    	m_speed = speed;
    	m_percentTolerance = percentTolerance;
    	m_p = p;
    	m_i = i;
    	m_d = d;
    	m_f = f;
        // Use requires() here to declare subsystem dependencies
        requires(Robot.driveline);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	if(Robot.oi.isPIDTuningDrivelineRotate){
    		m_angle = SmartDashboard.getNumber("DrivelineRotatePIDAngle", 0.0);
    		m_speed = SmartDashboard.getNumber("DrivelineRotatePIDSpeed", 0.0);
    		m_percentTolerance = SmartDashboard.getNumber("DrivelineRotatePIDPercentTolerance", 0.0);
    		m_maxTimeout = SmartDashboard.getNumber("DrivelineRotatePIDTimeout", 0.0);
    		m_p = SmartDashboard.getNumber("DrivelineRotatePID/P", 0.0);
    		m_i = SmartDashboard.getNumber("DrivelineRotatePID/I", 0.0);
    		m_d = SmartDashboard.getNumber("DrivelineRotatePID/D", 0.0);
    	}
    	Robot.driveline.setInputRange(0.0, m_angle);
    	Robot.driveline.setOutputRange(-m_speed,m_speed);
    	Robot.driveline.setPercentTolerance(m_percentTolerance);
    	Robot.driveline.setSetpoint(m_angle);
    	Robot.driveline.setPIDMode(DrivelinePIDMode.ROTATE, m_p, m_i, m_d, m_f);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if(timeSinceInitialized() > m_maxTimeout || Robot.driveline.onTarget()){
    		return true;
    	}else {
    		return false;
    	}
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.driveline.disable();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
