package frc.robot.commands.drive;

import static frc.robot.RobotContainer.drive;
import static frc.robot.constants.Constants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.util.Util;

public class FaceSpeaker extends Command {

	private PIDController wPID = new PIDController(1, 0.1, 0);

	public FaceSpeaker() {
		addRequirements(drive);
		wPID.setTolerance(Units.degreesToRadians(1));
		wPID.enableContinuousInput(0, Math.PI * 2);
	}

	private double getYVel() {
		ChassisSpeeds velocity = drive.getVelocity();
		double theta = drive.getPose().getRotation().getRadians();
		return Math.cos(theta) * velocity.vyMetersPerSecond + Math.sin(theta) * velocity.vxMetersPerSecond;
	}

	@Override
	public void execute() {
		Pose2d currentPose = drive.getPose();

		double cX = currentPose.getX();
		double dx = Util.getTargetX() - cX;
		double flight_time = Math.abs(dx) / Constants.NOTE_VELOCITY;
		double cY = currentPose.getY() + getYVel() * flight_time * 0;

		double dy = TARGET_Y - cY;

		double targetRotation = Math.atan(dy / dx);
		double currentRotation = currentPose.getRotation().getRadians();
		wPID.setSetpoint(targetRotation);

		drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
				0, 0, wPID.calculate(currentRotation) * 2.5, drive.getPose().getRotation()));
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
