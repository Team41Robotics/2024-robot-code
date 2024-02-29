package frc.robot.commands.elevator;

import static frc.robot.RobotContainer.elevator;
import static frc.robot.RobotContainer.left_js;
import static frc.robot.RobotContainer.right_js;

import edu.wpi.first.wpilibj2.command.Command;

public class manualElevator extends Command {
	@Override
	public void execute() {
		elevator.setLeft(left_js.getY());
		elevator.setRight(right_js.getY());
	}
}
