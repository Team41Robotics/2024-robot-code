package frc.robot;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The main class representing the robot.
 * Extends the LoggedRobot class to enable AdvantageKit logging.
 */
public class Robot extends LoggedRobot {
	private Command autonomousCommand;

	private LinearFilter v_filter = LinearFilter.movingAverage(10);
	private LinearFilter c_filter = LinearFilter.movingAverage(10);

	PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);

	/**
	 * Initializes the robot.
	 * This method is called once when the robot starts up.
	 * It sets up the robot's subsystems, configures button bindings,
	 * and initializes the logger.
	 *
	 * @see edu.wpi.first.wpilibj.TimedRobot#robotInit()
	 */
	@Override
	public void robotInit() {
		robot = this;
		initSubsystems();
		RobotContainer.configureOtherBindings();

		Logger.recordMetadata("ProjectName", "Robot2024");
		if (isReal()) {
			// Logger.addDataReceiver(new WPILOGWriter("/D/logs"));
			Logger.addDataReceiver(new NT4Publisher());
		} else {
			setUseTiming(false);
			String logPath = LogFileUtil.findReplayLog();
			// Logger.setReplaySource(new WPILOGReader(logPath));

			Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
		}
		// Logger.disableDeterministicTimestamps()
		Logger.start();
	}

	/**
	 * This method is called periodically during the robot operation.
	 * It runs scheduled commands, and records various outputs for logging purposes.
	 *
	 * @see edu.wpi.first.wpilibj.TimedRobot#robotPeriodic()
	 */
	@Override
	public void robotPeriodic() {
		shooter.periodic(); // wtf???
		CommandScheduler.getInstance().run();

		Logger.recordOutput("pdh/MPM/curr", pdh.getCurrent(1));
		Logger.recordOutput("pdh/totalCurr", pdh.getTotalCurrent());
		Logger.recordOutput("pdh/energy", pdh.getTotalEnergy());

		Logger.recordOutput("Battery/Voltage", v_filter.calculate(RobotController.getBatteryVoltage()));
		Logger.recordOutput("Battery/Current", c_filter.calculate(RobotController.getInputCurrent()));
	}

	@Override
	public void disabledInit() {}

	@Override
	public void disabledPeriodic() {}

	/**
	 * This method is called once at the beginning of autonomous mode.
	 * It resets the drive and retrieves the autonomous command from RobotContainer.
	 * If there is a valid autonomous command, it schedules it to run.
	 */
	@Override
	public void autonomousInit() {
		drive.zero();
		autonomousCommand = RobotContainer.getAutonomousCommand();

		if (autonomousCommand != null) autonomousCommand.schedule();
	}

	@Override
	public void autonomousPeriodic() {}

	/**
	 * Called when teleop mode is entered.
	 * Cancels the autonomous command if it is running.
	 */
	@Override
	public void teleopInit() {
		if (autonomousCommand != null) autonomousCommand.cancel();
	}

	@Override
	public void teleopPeriodic() {}

	/**
	 * Called when test mode is entered.
	 * This method resets the yaw of the IMU and retrieves the drive offsets.
	 */
	@Override
	public void testInit() {
		imu.zeroYaw();
		drive.getOffsets();
	}

	@Override
	public void testPeriodic() {}
}
