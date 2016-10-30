package org.usfirst.frc.team6193.robot;
/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	
    public static int LEFT_FRONT_CONTROLLER_CANID = 7;
    public static int LEFT_REAR_CONTROLLER_CANID = 6;
    public static int RIGHT_FRONT_CONTROLLER_CANID = 11;
    public static int RIGHT_REAR_CONTROLLER_CANID = 10;
    
    public static double DRIVELINE_TURN_SCALAR = 0.8;
}
