package frc.robot.commands.drive;

import static frc.robot.RobotContainer.drive;
import static frc.robot.RobotContainer.photon;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.Optional;

public class GoToRing extends Command {
	public GoToRing() {
		addRequirements(drive);
	}

	@Override
	public void execute() {
		Optional<Pose2d> target_state = photon.getNearestNote();
		if (target_state.isEmpty()) {
			drive.drive(new ChassisSpeeds());
			return;
		}
		Pose2d targetPose = target_state.get();
		double targetX = targetPose.getX();
		double targetY = targetPose.getY();
		if (Math.abs(targetY) > 0.1) {
			drive.drive(new ChassisSpeeds(0, 0, 2 * targetY));
			return;
		}
		drive.drive(new ChassisSpeeds(-targetX / 3., targetY / 3, 0));
	}

	@Override
	public boolean isFinished() {
		Optional<Pose2d> target_state = photon.getNearestNote();
		if (target_state.isEmpty()) {
			return false;
		}
		if (Math.abs(target_state.get().getX()) < 1.0
				&& Math.abs(target_state.get().getY()) <= 0.05) {
			return true; // TODO termination conditions
		}
		return false;
	}
}
