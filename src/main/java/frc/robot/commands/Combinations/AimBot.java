package frc.robot.commands.Combinations;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.drive.FaceSpeakerDrive;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import java.util.function.BooleanSupplier;

public class AimBot extends ConditionalCommand {
	public AimBot(ShooterSubsystem shooter, BooleanSupplier overide) {
		super(
				new ParallelCommandGroup(new FaceSpeakerDrive(), shooter.autoShoot())
						.until(() -> !shooter.ringLoaded())
						.andThen(shooter.toAngleDegreeCommand(90)),
				shooter.toAngleDegreeCommand(20).andThen(new InstantCommand(() -> shooter.runMotors(.7))),
				overide);
	}
}
