package frc.robot;

import static frc.robot.RobotContainer.drive;
import static frc.robot.RobotContainer.initSubsystems;
import static frc.robot.RobotContainer.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
	private Command autonomousCommand;

	@Override
	public void robotInit() {
		robot = this;
		initSubsystems();
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
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}
	}

	@Override
	public void teleopPeriodic() {}

	int first=1;
	@Override
	public void testInit() {
		CommandScheduler.getInstance().cancelAll();
		if(first == 1) {
			SwerveModule module = drive.modules[0];
			Shuffleboard.getTab("Module 0").addDouble("Direction", module::getDirection);
			Shuffleboard.getTab("Module 0").addDouble("Angular Velocity", module::getAngularVelocity);
			Shuffleboard.getTab("Module 0").addDouble("Velocity", module::getVelocity);
			Shuffleboard.getTab("Module 0").addDouble("Drive Voltage", () -> module.drive_v);
			Shuffleboard.getTab("Module 0").addDouble("Turn Voltage", () -> module.turn_v);
		}
		first=0;
	}

	@Override
	public void testPeriodic() {
		double mag = new Joystick(0).getMagnitude();
		double angle = new Joystick(0).getDirectionRadians();
		if(mag > 0.5) drive.modules[0].setState(new SwerveModuleState(0, new Rotation2d(angle)));
	}

	public void configureButtons() {
		;
	}
}
