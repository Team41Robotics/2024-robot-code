package frc.robot.commands;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.intake.SetPivot;

public class Handoff extends SequentialCommandGroup {
	public Handoff() {
		super(
				new ParallelCommandGroup(shooter.toAngleCommand(Rotation2d.fromDegrees(65)), new SetPivot(-85)),
				new WaitUntilCommand(() -> intake.angleAtSetpoint()),
				new PrintCommand("waiting for intake"),
				new WaitUntilCommand(() -> shooter.angleAtSetpoint()),
				new InstantCommand(() -> shooter.runMotors(0.45)),
				shooter.runFeeder()
						.deadlineWith(
								new StartEndCommand(() -> intake.runIntakeMotor(-0.5), () -> intake.stopIntakeMotor()))
						.until(ds.button(8)));
		addRequirements(intake);
	}
}
