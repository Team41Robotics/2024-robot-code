package frc.robot.commands.intake;

import static frc.robot.RobotContainer.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

public class SetPivot extends Command {

	public SetPivot(DoubleSupplier target) {
		addRequirements(intake);
		intake.pivotPID.setSetpoint(target.getAsDouble());
	}

	public SetPivot(double target) {
		this(() -> target);
	}

	@Override
	public void execute() {
		intake.setAngle(new Rotation2d());
	}

	@Override
	public boolean isFinished() {
		return intake.angleAtSetpoint();
	}
}
