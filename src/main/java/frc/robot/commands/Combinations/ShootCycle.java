package frc.robot.commands.Combinations;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.drive.FaceSpeakerDrive;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShootCycle extends SequentialCommandGroup {
	public ShootCycle(ShooterSubsystem shooter) {
		super(
				shooter.loadNote(),
				new ParallelRaceGroup(
						new WaitCommand(0.5).andThen(new WaitUntilCommand(shooter::isReady).withTimeout(70)),
						new ParallelCommandGroup(new FaceSpeakerDrive(), shooter.autoShoot())
								.until(() -> !shooter.ringLoaded())),
				new WaitCommand(0.5),
				new StartEndCommand(() -> shooter.runFeederMotor(0.4), () -> shooter.runFeederMotor(0))
						.until(() -> !shooter.ringLoaded()));
	}
}
