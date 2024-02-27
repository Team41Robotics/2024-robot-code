package frc.robot.commands.intake;

import static frc.robot.RobotContainer.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class SetPivot extends Command {

	Rotation2d target;

	public SetPivot(double target) {
		addRequirements(intake);
		this.target = (Rotation2d.fromDegrees(target));
	}

	@Override
	public void initialize() {
		intake.setAngle(target);
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
