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

	@Override
	public void robotInit() {
		robot = this;
		initSubsystems();
		configureButtonBindings();

		Logger.recordMetadata("ProjectName", "Robot2024");
		if (isReal()) {
			// Logger.addDataReceiver(new WPILOGWriter("/U"));
			Logger.addDataReceiver(new NT4Publisher());
		} else {
			setUseTiming(false);
			String logPath = LogFileUtil.findReplayLog();
			Logger.setReplaySource(new WPILOGReader(logPath));
			Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
		}
		// Logger.disableDeterministicTimestamps()
		Logger.start();
	}

	@Override
	public void robotPeriodic() {
		shooter.periodic();
		CommandScheduler.getInstance().run();
	}

	@Override
	public void disabledInit() {}

	@Override
	public void disabledPeriodic() {}

	@Override
	public void autonomousInit() {
		drive.zero();
		autonomousCommand = RobotContainer.getAutonomousCommand();

		if (autonomousCommand != null) autonomousCommand.schedule();
	}

	@Override
	public void autonomousPeriodic() {}

	@Override
	public void teleopInit() {
		if (autonomousCommand != null) autonomousCommand.cancel();
	}

	@Override
	public void teleopPeriodic() {}

	@Override
	public void testInit() {
		imu.zeroYaw();
		drive.getOffsets();
	}

	@Override
	public void testPeriodic() {}
}
