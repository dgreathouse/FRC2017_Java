package org.usfirst.frc.team6193.robot.commands;

import org.usfirst.frc.team6193.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This command will drive the robot for a set time at a certain speed while curving
 * See RobotDrive.drive for more information
 */
public class DrivelineMagnitudeCurveCommand extends Command {
	private double m_maxTimeout = 0.0;
	private double m_magnitude = 0.0;
	private double m_curve = 0.0;
    public DrivelineMagnitudeCurveCommand(double magnitude, double curve, double timeout) {
    	m_maxTimeout = timeout;
    	m_curve = curve;
    	m_magnitude = magnitude;
        // Use requires() here to declare subsystem dependencies
        requires(Robot.driveline);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	if(Robot.oi.isPIDTuningDrivelineMagCurve){
    		m_maxTimeout = SmartDashboard.getNumber("PIDDrivelineMagCurveTimeout",0.0);
    		m_magnitude = SmartDashboard.getNumber("PIDDrivelineMagCurveMagnitude",0.0);
    		m_curve = SmartDashboard.getNumber("PIDDrivelineMagCurveCurve",0.0);
    	}
    }
    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.driveline.DriveMagnitudeCurve(m_magnitude, m_curve);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if(timeSinceInitialized() >= m_maxTimeout){
    		return false;
    	}else {
    		return true;
    	}
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.driveline.DriveMagnitudeCurve(0.0, 0.0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
