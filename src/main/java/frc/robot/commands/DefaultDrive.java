package frc.robot.commands;

import static frc.robot.RobotContainer.*;
import static frc.robot.constants.Constants.*;
import static java.lang.Math.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class DefaultDrive extends Command {

	public DefaultDrive() {
		addRequirements(drive);
	}

	@Override
	public void execute() {
		double mag = Math.hypot(controller.getLeftX(), controller.getLeftY());
		double ma2 = MathUtil.applyDeadband(mag, 0.1);
		double theta = Math.atan2(controller.getLeftY(), controller.getLeftX());
		drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
				cos(theta) * ma2 * SWERVE_MAXSPEED * SPEED_MULT,
				-sin(theta) * ma2 * SWERVE_MAXSPEED * SPEED_MULT,
				MathUtil.applyDeadband(controller.getRightX(), 0.1),
				new Rotation2d(imu.yaw())));
	}
}
