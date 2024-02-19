package frc.robot.commands.intake;

import static frc.robot.RobotContainer.intake;

import edu.wpi.first.wpilibj2.command.Command;

public class Collect extends Command {

	public Collect() {
		addRequirements(intake);
	}

	@Override
	public void execute() {
		intake.setPivot(0);
		intake.runIntake(0.5);
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
