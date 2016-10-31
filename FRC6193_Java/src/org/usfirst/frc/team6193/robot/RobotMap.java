package org.usfirst.frc.team6193.robot;

import org.usfirst.frc.team6193.robot.lib.DrivelinePIDMode;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	
	/* Driveline constants */
    public static int DRIVELINE_LEFT_FRONT_MOTORCONTROLLER_CANID = 7;
    public static int DRIVELINE_LEFT_REAR_MOTORCONTROLLER_CANID = 6;
    public static int DRIVELINE_RIGHT_FRONT_MOTORCONTROLLER_CANID = 11;
    public static int DRIVELINE_RIGHT_REAR_MOTORCONTROLLER_CANID = 10;
    
    public static double DRIVELINE_TURN_SCALAR = 0.8;
    public static double DRIVELINE_ENCODER_INCHPERCNT = 0.02934;
    
    public static double DRIVELINE_DRIVE_PID_P = 0.05;
    public static double DRIVELINE_DRIVE_PID_I = 0.0005;
    public static double DRIVELINE_DRIVE_PID_D = 0.05;
    public static double DRIVELINE_DRIVE_PID_F = 0.25;
    
    public static double DRIVELINE_TURN_PID_P = 0.05;
    public static double DRIVELINE_TURN_PID_I = 0.0005;
    public static double DRIVELINE_TURN_PID_D = 0.05;
    public static double DRIVELINE_TURN_PID_F = 0.25;
    
    public static DrivelinePIDMode Driveline_PID_Mode = DrivelinePIDMode.MOVE;

    
}
