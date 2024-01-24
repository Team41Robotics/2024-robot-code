package frc.robot.commands;

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
		photon.switchMode(1);
		Optional<Pose2d> target_state = photon.getNearestNote();
		if (target_state.isPresent()) {
			System.out.println(target_state.get().getX());
			if (Math.abs(target_state.get().getY()) <= 0.1) {
				// drive.drive(new ChassisSpeeds());
				drive.drive(new ChassisSpeeds(
						-target_state.get().getX() / 3., target_state.get().getY() / 3, 0));
			} else {
				System.out.println("YAW TOO BIG");
				drive.drive(new ChassisSpeeds(0, 0, 2 * target_state.get().getY()));
			}
		} else {
			System.out.println("NO SIGHT");
			drive.drive(new ChassisSpeeds());
		}
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

	@Override
	public void end(boolean interrupted) {
		photon.switchMode(0);
	}
}
