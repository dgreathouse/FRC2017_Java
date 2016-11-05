package org.usfirst.frc.team6193.robot.commandGroups;

import org.usfirst.frc.team6193.robot.commands.DrivelineMagnitudeCurveCommand;
import org.usfirst.frc.team6193.robot.commands.DrivelineMoveCommand;
import org.usfirst.frc.team6193.robot.commands.DrivelineRotateCommand;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * This class is responsible for adding a set of commands to the 
 * scheduler based on a simple index in the constructor. The robot can
 * ask for a SmartDashboard value, get the value from hardware, or hard code
 * the value when creating the object.
 */
public class AutonomousCommandGroup extends CommandGroup {
    
    public  AutonomousCommandGroup(int index) {
    	switch(index){
    	case 0: // Move 4', Rotate 45 Degrees, Drive a curve for 2 seconds.
    		addSequential(new DrivelineMoveCommand(48, 0.85, 5, 0.5, 1.0, 0, 0, 0));
    		addSequential(new DrivelineRotateCommand(45, 0.85, 2, 1, 1, 0, 0, 0));
    		addSequential(new DrivelineMagnitudeCurveCommand(0.75, 45, 2));	
    		break;
    	case 1: // Move
    		addSequential(new DrivelineMoveCommand(48, 0.85, 5, 0.5, 1.0, 0, 0, 0));
    		break;
    	case 2: // Rotate
    		addSequential(new DrivelineRotateCommand(45, 0.85, 2, 1, 1, 0, 0, 0));
    		break;
    	case 3: // Magnitude and Curve
    		addSequential(new DrivelineMagnitudeCurveCommand(0.75, 45, 2));	
    		break;
    	}


    }
}
