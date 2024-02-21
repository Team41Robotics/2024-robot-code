package frc.robot.commands.Shooter;

import static frc.robot.RobotContainer.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

public class toAngle extends Command {
	DoubleSupplier target_angle;
	PIDController t1Controller = new PIDController(2, 0, 0);
	// angle is from horizontal
	public toAngle(DoubleSupplier angle) {

		addRequirements(shooter);
		target_angle = () -> Units.degreesToRotations(90 - angle.getAsDouble());
		System.out.println("Converted: " + Units.degreesToRotations(90 - angle.getAsDouble()));
		t1Controller.setSetpoint(target_angle.getAsDouble());
		t1Controller.setTolerance(0.01);
	}

	public toAngle(double angle) {
		this(() -> angle);
	}

	@Override
	public void execute() {
		shooter.setAngle(new Rotation2d());
		// shooter.angleMotor2.set(output);
	}

	@Override
	public boolean isFinished() {
		return shooter.angleAtSetpoint();
		// t1Controller.atSetpoint();
	}
}
