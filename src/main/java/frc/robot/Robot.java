package frc.robot;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
	private Command autonomousCommand;
	// CANSparkMax intakeMax = new CANSparkMax(20, MotorType.kBrushless);

	@Override
	public void robotInit() {
		robot = this;
		initSubsystems();
		configureButtonBindings();
		Logger.recordMetadata("ProjectName", "OffSeason Swerve"); // Set a metadata value

		if (isReal()) {
			// Logger.addDataReceiver(new WPILOGWriter("/U")); // Log to a USB stick
			Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
		} else {
			setUseTiming(false); // Run as fast as possible
			String logPath =
					LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
			Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
			Logger.addDataReceiver(
					new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
		}

		// Logger.disableDeterministicTimestamps() // See "Deterministic Timestamps" in the "Understanding Data Flow"
		// page
		Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.g
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
		//System.out.println(left_js.getX() + "," + left_js.getY());
	}

	@Override
	public void disabledInit() {}

	@Override
	public void disabledPeriodic() {}

	@Override
	public void autonomousInit() {
		drive.zero();
		autonomousCommand = RobotContainer.getAutonomousCommand();

		if (autonomousCommand != null) {
			autonomousCommand.schedule();
		}
	}

	@Override
	public void autonomousPeriodic() {}

	@Override
	public void teleopInit() {
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}
	}

	@Override
	public void teleopPeriodic() {
		/*if(left_js.button(1).getAsBoolean()){
			intakeMax.set(0.7);
		}else if(right_js.button(1).getAsBoolean()){
			intakeMax.set(-0.7);
		}else{
			intakeMax.set(0);
		}*/
	}

	@Override
	public void testInit() {
		imu.zeroYaw();
		drive.getOffsets();
	}

	@Override
	public void testPeriodic() {}
}
