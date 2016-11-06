
package org.usfirst.frc.team6193.robot;

import org.usfirst.frc.team6193.robot.commandGroups.AutonomousCommandGroup;
import org.usfirst.frc.team6193.robot.subsystems.Driveline;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.communication.FRCNetworkCommunicationsLibrary;
import edu.wpi.first.wpilibj.communication.UsageReporting;
import edu.wpi.first.wpilibj.communication.FRCNetworkCommunicationsLibrary.tInstances;
import edu.wpi.first.wpilibj.communication.FRCNetworkCommunicationsLibrary.tResourceType;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Main Robot Class
 * The Virtual Machine "VM" is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	/* Static references to singleton objects */
	public static Driveline driveline;
	public static OI oi;

	/* State variables used to determine the state the robot is in */
	private boolean m_disabledInitialized;
	private boolean m_autonomousInitialized;
	private boolean m_teleopInitialized;
	private boolean m_testInitialized;

	/* Stored timestamp for loop time debugging*/
	private double m_prevTimeStamp;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	public void robotInit() {
		/* Create a instance of the Driveline class and call it driveline */
		driveline = new Driveline();
		oi = new OI();

		m_disabledInitialized = false;
		m_autonomousInitialized = false;
		m_teleopInitialized = false;
		m_testInitialized = false;
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	public void disabledInit() {
		Scheduler.getInstance().removeAll();
		driveline.disable();
		SmartDashboard.putNumber("DrivelineMovePID/p", 0.1);
		SmartDashboard.putNumber("DrivelineMovePID/i", 0.0);
		SmartDashboard.putNumber("DrivelineMovePID/d", 0.0);
		SmartDashboard.putNumber("DrivelineMovePID/setpoint", 179);
	}
	/**
	 * This function is called periodically while in the Disable state.
	 * 
	 */
	public void disabledPeriodic() {
		// I see no reason to run the scheduler in Disable state
		//Scheduler.getInstance().run();
	}
	/**
	 * This function is called once when the robot enters the Autonomous state.
	 */
	public void autonomousInit() {
		int index = (int)SmartDashboard.getNumber("AutoplayIndex", 0);
		AutonomousCommandGroup ACG = new AutonomousCommandGroup(index);
		ACG.start();
	}

	/**
	 * This function is called periodically while in the Autonomous state
	 */
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This function is called once when the robot enters TeleOp state
	 */
	public void teleopInit() {

	}

	/**
	 * This function is called periodically when the robot is in TeleOp state
	 */
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This function is called periodically during test mode
	 */
	public void testPeriodic() {
		LiveWindow.run();
	}

	/** Start Competition from IterativeRobot class
	 * Code was copied from the IterativeRobot class for testing of timing issues.
	 */
	public void startCompetition() {
		UsageReporting.report(tResourceType.kResourceType_Framework, tInstances.kFramework_Iterative);

		robotInit();

		// Tell the DS that the robot is ready to be enabled
		FRCNetworkCommunicationsLibrary.FRCNetworkCommunicationObserveUserProgramStarting();

		// loop forever, calling the appropriate mode-dependent function
		LiveWindow.setEnabled(false);
		while (true) {
			// Call the appropriate function depending upon the current robot
			// mode
			if (isDisabled()) {
				// call DisabledInit() if we are now just entering disabled mode
				// from
				// either a different mode or from power-on
				if (!m_disabledInitialized) {
					LiveWindow.setEnabled(false);
					disabledInit();
					m_disabledInitialized = true;
					// reset the initialization flags for the other modes
					m_autonomousInitialized = false;
					m_teleopInitialized = false;
					m_testInitialized = false;
				}
				if (nextPeriodReady()) {
					FRCNetworkCommunicationsLibrary.FRCNetworkCommunicationObserveUserProgramDisabled();
					disabledPeriodic();
				}
			} else if (isTest()) {
				// call TestInit() if we are now just entering test mode from
				// either
				// a different mode or from power-on
				if (!m_testInitialized) {
					LiveWindow.setEnabled(true);
					testInit();
					m_testInitialized = true;
					m_autonomousInitialized = false;
					m_teleopInitialized = false;
					m_disabledInitialized = false;
				}
				if (nextPeriodReady()) {
					FRCNetworkCommunicationsLibrary.FRCNetworkCommunicationObserveUserProgramTest();
					testPeriodic();
				}
			} else if (isAutonomous()) {
				// call Autonomous_Init() if this is the first time
				// we've entered autonomous_mode
				if (!m_autonomousInitialized) {
					LiveWindow.setEnabled(false);
					// KBS NOTE: old code reset all PWMs and relays to "safe
					// values"
					// whenever entering autonomous mode, before calling
					// "Autonomous_Init()"
					autonomousInit();
					m_autonomousInitialized = true;
					m_testInitialized = false;
					m_teleopInitialized = false;
					m_disabledInitialized = false;
				}
				if (nextPeriodReady()) {
					FRCNetworkCommunicationsLibrary.FRCNetworkCommunicationObserveUserProgramAutonomous();
					autonomousPeriodic();
				}
			} else {
				// call Teleop_Init() if this is the first time
				// we've entered teleop_mode
				if (!m_teleopInitialized) {
					LiveWindow.setEnabled(false);
					teleopInit();
					m_teleopInitialized = true;
					m_testInitialized = false;
					m_autonomousInitialized = false;
					m_disabledInitialized = false;
				}
				if (nextPeriodReady()) {
					FRCNetworkCommunicationsLibrary.FRCNetworkCommunicationObserveUserProgramTeleop();
					teleopPeriodic();
				}
			}
			m_ds.waitForData();
			calculateLoopTime();
		}
	}

	/**
	 * Determine if the appropriate next periodic function should be called.
	 * Call the periodic functions whenever a packet is received from the Driver
	 * Station, or about every 20ms.
	 * 
	 * This seems to be redundant with m_ds.waitForData(). After testing, these
	 * method calls may be removed. Include testing for motor safety,
	 * which we disable. :)
	 */
	private boolean nextPeriodReady() {
		return m_ds.isNewControlData();
	}

	/**
	 * This function is used to calculate the time since the last periodic call.
	 * Sending the looptime to the dashboard helps in debugging WIFI lost packets and FMS issues
	 */
	private void calculateLoopTime() {
		// Get the current time stamp from the FPGA. FPGA is the most accurate
		// time that can be captured
		double time = Timer.getFPGATimestamp();
		// Find the loop time by subtracting the previous time from the current
		// time.
		double loopTime = time - m_prevTimeStamp;
		// Store the current time into the previous time for the next
		// calculation
		m_prevTimeStamp = time;
		// Send it to the SmartDashboard for graphing
		// Data is not usable for the FTA, but it helps understand the issues with the FMS
		SmartDashboard.putNumber("LoopTime", loopTime);
	}
}
