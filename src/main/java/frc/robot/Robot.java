package frc.robot;

import static frc.robot.RobotContainer.*;
import static frc.robot.constants.Constants.SWERVE_MAXSPEED;
import static java.lang.Math.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
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
	public void teleopPeriodic() {
		//drive.drive(new ChassisSpeeds(
		//MathUtil.applyDeadband(controller.getLeftX(), 0.1) * SWERVE_MAXSPEED,
		//MathUtil.applyDeadband(-controller.getLeftY(), 0.1) * SWERVE_MAXSPEED,
		//MathUtil.applyDeadband(-controller.getRightY(), 0.1) / 5));
		drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
				MathUtil.applyDeadband(controller.getLeftX(), 0.1) * SWERVE_MAXSPEED,
				MathUtil.applyDeadband(-controller.getLeftY(), 0.1) * SWERVE_MAXSPEED,
				MathUtil.applyDeadband(controller.getRightX(), 0.1) ,
				new Rotation2d(-(imu.getAngle() +180 % 360)/180 * PI)));

		//System.out.println(imu.getAngle()%360);
	}

	int first = 1;

	@Override
	public void testInit() {
		imu.zeroYaw();
		drive.getOffsets();
		/*
		CommandScheduler.getInstance().cancelAll();
		if (first == 1) {
			SwerveModule module = drive.modules[0];
			Shuffleboard.getTab("Module 0").addDouble("Direction", module::getDirection);
			Shuffleboard.getTab("Module 0").addDouble("Angular Velocity", module::getAngularVelocity);
			Shuffleboard.getTab("Module 0").addDouble("Velocity", module::getVelocity);
			Shuffleboard.getTab("Module 0").addDouble("Drive Voltage", () -> module.drive_v);
			Shuffleboard.getTab("Module 0").addDouble("Turn Voltage", () -> module.turn_v);
			Shuffleboard.getTab("Module 0").addNumber("Setpoint", () -> module.target_state.speedMetersPerSecond);
			Shuffleboard.getTab("Module 0").addNumber("Setpoint Angle", () -> module.target_state.angle.getDegrees());
		}
		first = 0;
		*/
	}

	@Override
	public void testPeriodic() {
		double x = controller.getRightX();
		double y = controller.getRightY();
		double mag = Math.hypot(x, y);
		double angle = Math.atan2(y, x);
		//if (mag > 0.5) {
		//	drive.modules[1].setState(new SwerveModuleState(0, new Rotation2d(angle)));
		//}
	}
}
