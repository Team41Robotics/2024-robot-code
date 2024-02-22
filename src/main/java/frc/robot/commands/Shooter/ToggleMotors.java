package frc.robot.commands.Shooter;

import static frc.robot.RobotContainer.shooter;

import edu.wpi.first.wpilibj2.command.Command;

public class ToggleMotors extends Command {

	public ToggleMotors() {
		addRequirements(shooter);
	}

	@Override
	public void execute() {
		shooter.toggleMode();
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
