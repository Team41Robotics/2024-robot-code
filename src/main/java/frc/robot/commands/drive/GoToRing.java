package frc.robot.commands.drive;

import static frc.robot.RobotContainer.drive;
import static frc.robot.RobotContainer.photon;
import static frc.robot.RobotContainer.right_js;
import static frc.robot.RobotContainer.xbox;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.Util;
import java.util.Optional;

public class GoToRing extends Command {

	private PIDController wPID = new PIDController(1.25, 0, 0);

	private Translation2d storedNotePose = null;

	public GoToRing() {
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

		wPID.enableContinuousInput(-Math.PI, Math.PI);
	}

	@Override
	public void execute() {

		Optional<Pose2d> nearestNote = photon.getNearestNote();

		if (nearestNote.isPresent()) {
			// Translation2d current_translation = drive.getPose().getTranslation();
			Translation2d target_translation = nearestNote.get().getTranslation();

			Pose2d currentPose = drive.getPose();
			storedNotePose = target_translation.plus(currentPose.getTranslation());
		}

		if (storedNotePose == null) {
			return;
		}

		double targetRotation = storedNotePose
				.minus(drive.getPose().getTranslation())
				.getAngle()
				.getRadians();
		wPID.setSetpoint(targetRotation);

		Pose2d currentPose = drive.getPose();

		double currentRotation = currentPose.getRotation().getRadians();
		// Logger.recordOutput("AutoAngle/err", wPID.getPositionError());
		System.out.println("target rotation: " + wPID.getSetpoint());
		System.out.println("current rotation:" + currentRotation);

		ChassisSpeeds speeds = Util.joystickToSpeeds(
				xbox.getLeftY(),
				xbox.getLeftX(),
				0,
				right_js.button(1).getAsBoolean(),
				Rotation2d.fromRadians(targetRotation));
		speeds.omegaRadiansPerSecond = -wPID.calculate(Math.PI) * 2.5;
		drive.drive(speeds);
	}

	@Override
	public boolean isFinished() {
		return false;
		// return wPID.atSetpoint();
	}

	@Override
	public void end(boolean interrupted) {
		// TODO Auto-generated method stub
		super.end(interrupted);
		drive.drive(new ChassisSpeeds());
		storedNotePose = null;
	}
}
