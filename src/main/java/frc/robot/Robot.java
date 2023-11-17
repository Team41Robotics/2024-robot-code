package frc.robot;

import static frc.robot.RobotContainer.*;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends LoggedRobot {
	private Command autonomousCommand;

	@Override
	public void robotInit() {
		robot = this;
		initSubsystems();
		configureButtonBindings();
		
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
	}

	@Override
	public void disabledInit() {}

	@Override
	public void disabledPeriodic() {}

	@Override
	public void autonomousInit() {
		drive.zero();
		// autonomousCommand = robotContainer.getAutonomousCommand();

		// if (autonomousCommand != null) {
		// autonomousCommand.schedule();
		// }
	}

	@Override
	public void autonomousPeriodic() {}

	@Override
	public void teleopInit() {
		imu.zeroYaw();
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}
	}

	@Override
	public void teleopPeriodic() {}

	int first = 1;

	@Override
	public void testInit() {
		imu.zeroYaw();
		drive.getOffsets();
	}

	@Override
	public void testPeriodic() {}
}
