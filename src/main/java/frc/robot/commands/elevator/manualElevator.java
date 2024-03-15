package frc.robot.commands.elevator;

import static frc.robot.RobotContainer.elevator;
import static frc.robot.RobotContainer.left_js;
import static frc.robot.RobotContainer.right_js;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;

public class manualElevator extends Command {
	@Override
	public void execute() {
		elevator.setLeftPos(elevator.getLeftPos() - 3 * MathUtil.applyDeadband(left_js.getY(), 0.1));
		elevator.setRightPos(elevator.getRightPos() - 3 * MathUtil.applyDeadband(right_js.getY(), 0.1));
	}
}
