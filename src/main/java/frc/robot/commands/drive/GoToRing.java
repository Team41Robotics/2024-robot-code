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

	@Override
	public void initialize() {
		wPID.reset();
		wPID.enableContinuousInput(-Math.PI, Math.PI);
	}

	@Override
	public void execute() {

		Optional<Pose2d> nearestNote = photon.getNearestNote(); // gets the pose of the nearest note from photon vision

		if (nearestNote.isPresent()) {

			Translation2d target_translation =
					nearestNote.get().getTranslation(); // gets translation vector for nearest note

			Pose2d currentPose = drive.getPose(); // gets vector of current pose
			storedNotePose = target_translation.plus(currentPose.getTranslation()); // adds pose vector and note vector
		}

		if (storedNotePose == null) {
			return;
		}

		double targetRotation = storedNotePose
				.minus(drive.getPose()
						.getTranslation()) // compare difference in angle between current translation and target
				// translation
				.getAngle()
				.getRadians();
		wPID.setSetpoint(targetRotation); // use PID to adjust to target translation

		Pose2d currentPose = drive.getPose();

		// double currentRotation = currentPose.getRotation().getRadians();
		// Logger.recordOutput("AutoAngle/err", wPID.getPositionError());
		// System.out.println("target rotation: " + wPID.getSetpoint());
		// System.out.println("current rotation:" + currentRotation);

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
