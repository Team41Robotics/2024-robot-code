package frc.robot.commands.drive;

import static frc.robot.RobotContainer.drive;
import static frc.robot.RobotContainer.left_js;
import static frc.robot.RobotContainer.right_js;
import static frc.robot.constants.Constants.SPEED_MULT;
import static frc.robot.constants.Constants.SWERVE_MAXSPEED;
import static java.lang.Math.cos;
import static java.lang.Math.sin;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.util.Util;

public class FaceSpeakerDrive extends Command {

	private PIDController wPID = new PIDController(1, 0, 0);

	public FaceSpeakerDrive() {
		addRequirements(drive);
		wPID.setTolerance(Units.degreesToRadians(1));
		wPID.enableContinuousInput(0, Math.PI * 2);
	}

	private double getYVel() {
		ChassisSpeeds velocity = drive.getVelocity();
		double theta = drive.getPose().getRotation().getRadians();
		return Math.cos(theta) * velocity.vyMetersPerSecond + Math.sin(theta * velocity.vxMetersPerSecond);
	}

	@Override
	public void execute() {
		Pose2d currentPose = drive.getPose();

		double cX = currentPose.getX();
		double dx = targetX - cX;
		double flight_time = dx / Constants.NOTE_VELOCITY;
		double cY = currentPose.getY() + getYVel() * flight_time;

		double dy = targetY - cY;

		double targetRotation = Math.atan(dy / dx);
		double currentRotation = currentPose.getRotation().getRadians();
		wPID.setSetpoint(targetRotation);

		double vx = left_js.getY();
		double vy = left_js.getX();
		double mag = Math.hypot(vx, vy);
		double ma2 = MathUtil.clamp(Util.sensCurve(mag * 1.5, 0.1), -1, 1);
		double theta = Math.atan2(vy, vx);
		double sign = (DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Blue) ? 1.0 : -1.0);
		double speed_mult = (right_js.button(1).getAsBoolean() ? 0.9 : SPEED_MULT);

		drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
				cos(theta) * ma2 * SWERVE_MAXSPEED * speed_mult * sign,
				sin(theta) * ma2 * SWERVE_MAXSPEED * speed_mult * sign,
				wPID.calculate(currentRotation) * 2.5,
				drive.getPose().getRotation()));
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
