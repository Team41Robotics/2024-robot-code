package frc.robot.commands;

import static frc.robot.RobotContainer.*;
import static frc.robot.constants.Constants.*;
import static java.lang.Math.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class DefaultDrive extends Command {
	public DefaultDrive() {
		addRequirements(drive);
	}

	public void run(double vx, double vy, double w) {
		double mag = Math.hypot(vx, vy);
		double ma2 = MathUtil.applyDeadband(mag, 0.1);
		double theta = Math.atan2(vy, vx);
		// drive.drive(new ChassisSpeeds(
		// 		cos(theta) * ma2 * SWERVE_MAXSPEED * SPEED_MULT,
		// 		sin(theta) * ma2 * SWERVE_MAXSPEED * SPEED_MULT,
		// 		MathUtil.applyDeadband(w, 0.1) * 2.5));
		drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
				cos(theta) * ma2 * SWERVE_MAXSPEED * SPEED_MULT,
				sin(theta) * ma2 * SWERVE_MAXSPEED * SPEED_MULT,
				MathUtil.applyDeadband(w, 0.1) * 2.5,
				drive.getPose().getRotation()));
	}

	@Override
	public void execute() {
		// double mag = Math.hypot(controller.getLeftY(), controller.getLeftX());
		// double ma2 = MathUtil.applyDeadband(mag, 0.1);
		// double theta = Math.atan2(controller.getLeftX(), controller.getLeftY());
		// drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
		// 		-cos(theta) * ma2 * SWERVE_MAXSPEED * SPEED_MULT,
		// 		-sin(theta) * ma2 * SWERVE_MAXSPEED * SPEED_MULT,
		// 		MathUtil.applyDeadband(controller.getRightX(), 0.1) * 2.5,
		// 		drive.getPose().getRotation()));
		run(-controller.getLeftY(), -controller.getLeftX(), -controller.getRightX());
	}
}
