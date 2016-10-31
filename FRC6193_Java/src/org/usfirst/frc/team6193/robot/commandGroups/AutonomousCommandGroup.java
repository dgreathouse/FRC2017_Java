package org.usfirst.frc.team6193.robot.commandGroups;

import org.usfirst.frc.team6193.robot.commands.DrivelineMoveCommand;

import edu.wpi.first.wpilibj.command.CommandGroup;



/**
 *
 */
public class AutonomousCommandGroup extends CommandGroup {
    
    public  AutonomousCommandGroup(int index) {
    	
    	if(index == 0){
    		addSequential(new DrivelineMoveCommand(48, 0.85, 5, 0.5, 1.0, 0, 0, 0));
    	}else if(index == 1){
    		
    	}

    }
}
