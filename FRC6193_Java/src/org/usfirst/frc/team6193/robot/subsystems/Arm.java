package org.usfirst.frc.team6193.robot.subsystems;

import org.usfirst.frc.team6193.robot.RobotMap;
import org.usfirst.frc.team6193.robot.commands.ArmDefaultCommand;
import org.usfirst.frc.team6193.robot.lib.SpeedLimiter;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.command.PIDSubsystem;

/**
 *
 */
public class Arm extends PIDSubsystem {
	private CANTalon m_ArmMotorController;
	public Arm() {
		super ("Arm",1,0,0);
		m_ArmMotorController = new CANTalon(RobotMap.ARM_MOTORCONTROLLER_CANID);
		
	}

    // Initialize your subsystem here
 
    public void initDefaultCommand() {
    	setDefaultCommand (new ArmDefaultCommand());
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    protected double returnPIDInput() {
        // Return your input value for the PID loop
        // e.g. a sensor, like a potentiometer:
        // yourPot.getAverageVoltage() / kYourMaxVoltage;
    	return 0.0;
    }
    
    protected void usePIDOutput(double output) {
        // Use output to drive your system, like a motor
        // e.g. yourMotor.set(output);
    }

	public void Move(double armSpeed){
		//armSpeed = SpeedLimiter.limit (armSpeed);
		armSpeed = limitArmAngle (armSpeed);
		m_ArmMotorController.set(armSpeed);
		//RobotMap.armController.set (armSpeed);
		// TODO Auto-generated method stub
		
	}
	double limitArmAngle(double val){
		double angle = m_ArmMotorController.getEncPosition();
		if(angle>120||angle<65){
			return 0;
		}else{
			return val;
		}
		
	}
}
