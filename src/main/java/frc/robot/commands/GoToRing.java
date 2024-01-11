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
				drive.drive(new ChassisSpeeds(-target_state.get().getX() / 3., 0, 0));
			} else {
				drive.drive(new ChassisSpeeds(0, 0, 5 * target_state.get().getY()));
			}
		} else {
			drive.drive(new ChassisSpeeds());
		}
	}

	@Override
	public boolean isFinished() {
		Optional<Pose2d> target_state = photon.getNearestNote();
		if (target_state.isPresent()) { // TODO termination conditions
		}
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		photon.switchMode(0);
	}
}