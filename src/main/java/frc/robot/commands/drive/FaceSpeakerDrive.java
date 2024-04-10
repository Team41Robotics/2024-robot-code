package frc.robot.commands.drive;

import static frc.robot.RobotContainer.drive;
import static frc.robot.RobotContainer.left_js;
import static frc.robot.RobotContainer.right_js;
import static frc.robot.constants.Constants.FieldConstants.TARGET_Y;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.util.Util;
import org.littletonrobotics.junction.Logger;

public class FaceSpeakerDrive extends Command {

	private PIDController wPID = new PIDController(1.5, 0.1, 0);

	public FaceSpeakerDrive() {
		addRequirements(drive);
		wPID.setTolerance(Units.degreesToRadians(1));
		wPID.enableContinuousInput(0, Math.PI * 2);
	}

	private double getYVel() {
		ChassisSpeeds velocity = drive.getVelocity();
		double theta = drive.getPose().getRotation().getRadians();
		return Math.cos(theta) * velocity.vyMetersPerSecond + Math.sin(theta) * velocity.vxMetersPerSecond;
	}

	private double getXVel() {
		ChassisSpeeds velocity = drive.getVelocity();
		double theta = drive.getPose().getRotation().getRadians();
		return Math.sin(theta) * velocity.vyMetersPerSecond + Math.cos(theta) * velocity.vxMetersPerSecond;
	}

	@Override
	public void initialize() {
		wPID.reset();
	}

	@Override
	public void execute() {
		Pose2d currentPose = drive.getPose();

		double cX = currentPose.getX();
		double dx = Util.getTargetX() - cX;
		double flight_time = Math.abs(dx) / (Constants.NOTE_VELOCITY + getXVel());
		double cY = currentPose.getY() + getYVel() * flight_time;

		double dy = TARGET_Y - cY;

		double targetRotation = Math.atan(dy / dx);
		double currentRotation = currentPose.getRotation().getRadians();
		wPID.setSetpoint(targetRotation + (!Util.isRed() ? Math.PI : 0));
		Logger.recordOutput("AutoAngle/err", wPID.getPositionError());

		ChassisSpeeds speeds = Util.joystickToSpeeds(
				left_js.getY(),
				left_js.getX(),
				0,
				right_js.button(1).getAsBoolean(),
				drive.getPose().getRotation());
		speeds.omegaRadiansPerSecond = wPID.calculate(currentRotation) * 2.5;
		drive.drive(speeds);
		;
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
