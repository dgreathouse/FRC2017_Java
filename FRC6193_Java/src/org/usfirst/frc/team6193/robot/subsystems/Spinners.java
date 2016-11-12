package org.usfirst.frc.team6193.robot.subsystems;

import org.usfirst.frc.team6193.robot.RobotMap;
import org.usfirst.frc.team6193.robot.commands.SpinnersDefaultCommand;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.command.PIDSubsystem;

/**
 *
 */
public class Spinners extends PIDSubsystem {
	private CANTalon m_LeftSpinnersMotorController;
	private CANTalon m_RightSpinnersMotorController;
    public Spinners() {
    	super("Spinners", 0, 0, 0);
    	m_LeftSpinnersMotorController = new CANTalon(RobotMap.LEFTSPINNER_MOTORCONTROLLER_CANID);
    	m_RightSpinnersMotorController = new CANTalon(RobotMap.RIGHTSPINNER_MOTORCONTROLLER_CANID);
    	m_RightSpinnersMotorController.setInverted(true);
        // Use these to get going:
        // setSetpoint() -  Sets where the PID controller should move the system
        //                  to
        // enable() - Enables the PID controller.
    }
    
    public void initDefaultCommand() {
    	//setDefaultCommand (new SpinnersDefaultCommand());
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public void Spin(double spinSpeed){
    	m_LeftSpinnersMotorController.set(spinSpeed);
    	m_RightSpinnersMotorController.set(spinSpeed);
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
}
