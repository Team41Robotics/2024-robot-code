package frc.robot.commands;

import static frc.robot.RobotContainer.*;
import static frc.robot.constants.Constants.*;
import static java.lang.Math.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class DefaultDrive extends Command {
	double trot = 0;
	PIDController tpid = new PIDController(1, 0, 0);

	public DefaultDrive() {
		addRequirements(drive);
		trot = imu.yaw();
		tpid.enableContinuousInput(-PI, PI);
	}

	@Override
	public void execute() {
		double mag = Math.hypot(controller.getLeftX(), controller.getLeftY());
		double ma2 = MathUtil.applyDeadband(mag, 0.1);
		double theta = Math.atan2(controller.getLeftY(), controller.getLeftX());
		trot += MathUtil.applyDeadband(controller.getRightX(), 0.1) * 2.5 * 0.05;
		drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
				cos(theta) * ma2 * SWERVE_MAXSPEED * SPEED_MULT,
				-sin(theta) * ma2 * SWERVE_MAXSPEED * SPEED_MULT,
				MathUtil.applyDeadband(controller.getRightX(), 0.1) * 2.5 + 0 * tpid.calculate(imu.yaw(), trot),
				new Rotation2d(imu.yaw())));
	}
}
