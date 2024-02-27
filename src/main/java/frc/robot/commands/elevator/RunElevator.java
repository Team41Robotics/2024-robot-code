package frc.robot.commands.elevator;

import static frc.robot.RobotContainer.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import java.lang.Math.*;

public class RunElevator extends Command {

	double speed;
	double maxVelocity1;
	double maxVelocity2;

	boolean leftContact;
	boolean rightContanct;

	public RunElevator(double s) {
		addRequirements(elevator);
		speed = s;
		leftContact = false;
		rightContanct = false;
	}

	@Override
	public void initialize() {
		elevator.setMotor(speed);
		maxVelocity1 = 0;
		maxVelocity2 = 0;
	}

	@Override
	public void execute() {
		maxVelocity1 =
				Math.max(maxVelocity1, elevator.climberMotor1.getVelocity().getValueAsDouble());
		maxVelocity2 =
				Math.max(maxVelocity2, elevator.climberMotor2.getVelocity().getValueAsDouble());

		if (elevator.climberMotor1.getVelocity().getValueAsDouble() < maxVelocity1 * 0.75) {
			leftContact = true;
			elevator.climberMotor1.set(0);
		} else if (elevator.climberMotor2.getVelocity().getValueAsDouble() < maxVelocity2 * 0.75) {
			rightContanct = true;
			elevator.climberMotor2.set(0);
		}
	}

	@Override
	public boolean isFinished() {
		return (leftContact && rightContanct);
	}
}
