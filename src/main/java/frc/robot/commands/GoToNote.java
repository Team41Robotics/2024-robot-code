package frc.robot.commands;

import static frc.robot.RobotContainer.drive;
import static frc.robot.RobotContainer.photon;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.Optional;

public class GoToNote extends Command {
	public GoToNote() {
		addRequirements(drive);
	}

	@Override
	public void execute() {
		Optional<Pose2d> target_state = photon.getNearestNote();
		if(target_state.isPresent()){
			drive.drive(new ChassisSpeeds(-target_state.get().getX()/3., 0, 0));
		
		}
	}

	@Override
	public boolean isFinished() {
		Optional<Pose2d> target_state = photon.getNearestNote();
		if(!target_state.isPresent()) return false;
		return Math.abs(target_state.get().getX()) < 1.0;

	}
}
