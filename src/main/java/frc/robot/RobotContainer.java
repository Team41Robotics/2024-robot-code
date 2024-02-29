package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.drive.DefaultDrive;
import frc.robot.commands.drive.FaceSpeakerDrive;
import frc.robot.commands.elevator.manualElevator;
import frc.robot.commands.intake.SetPivot;
import frc.robot.subsystems.LEDS;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.util.LocalADStarAK;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
	// robot
	public static Robot robot;

	// subsystems
	public static SwerveSubsystem drive = new SwerveSubsystem();
	public static ShooterSubsystem shooter = new ShooterSubsystem();
	public static IntakeSubsystem intake = new IntakeSubsystem();
	public static ElevatorSubsystem elevator = new ElevatorSubsystem();

	public static PhotonVision photon = new PhotonVision();
	public static LEDS leds = new LEDS();

	// other
	public static LoggedDashboardChooser<Command> autoChooser;

	public static IMU imu = new IMU();
	public static CommandJoystick left_js = new CommandJoystick(1);
	public static CommandJoystick right_js = new CommandJoystick(2);
	public static CommandJoystick ds = new CommandJoystick(0);

	public static void initSubsystems() {
		drive.setDefaultCommand(new DefaultDrive(() -> left_js.getY(), () -> left_js.getX(), () -> -right_js.getX()));
		drive.init(new Pose2d(1, 1, new Rotation2d(Math.PI)));

		leds.init();

		Pathfinding.setPathfinder(new LocalADStarAK());

		autoChooser = new LoggedDashboardChooser<>("Auto Routine", AutoBuilder.buildAutoChooser());
		Shuffleboard.getTab("Swerve").add("Auto Selector", autoChooser.getSendableChooser());
	}

	public static void configureButtonBindings() {
		// straight up
		ds.button(1).onTrue(shooter.toAngleCommand(Rotation2d.fromDegrees(15)));
		ds.button(2).whileTrue(new manualElevator());
		// handoff syste m
		left_js.button(1).whileTrue(intake.runIntake(-0.4));
		left_js.button(3).onTrue(shooter.runFeeder());

		left_js.button(4)
				.onTrue(shooter.toAngleCommand(Rotation2d.fromDegrees(15))
						.alongWith(shooter.muzzleLoad().andThen(shooter.toAngleCommand(Rotation2d.fromDegrees(90)))));

		right_js.button(1)
				.whileTrue(new ParallelCommandGroup(new FaceSpeakerDrive(), shooter.autoShoot())
						.until(() -> !shooter.ringLoaded())
						.andThen(shooter.toAngleCommand(Rotation2d.fromDegrees(90))));
		// right_js.button(1).whileTrue(intake.runIntake(0.6));
		right_js.button(2).onTrue(new SetPivot(120));
		right_js.button(4)
				.onTrue(new StartEndCommand(() -> shooter.runFeederMotor(0.4), () -> shooter.runFeederMotor(0))
						.until(() -> !shooter.ringLoaded()));
		ds.button(3).onTrue(shooter.toAngleCommand(Rotation2d.fromDegrees(90))); // new Handoff());

		ds.button(12).onTrue(shooter.autoShoot());
		// ds.button(6).onTrue(shooter.toAngleCommand(Rotation2d.fromDegrees(55)));
		ds.button(6).onTrue(shooter.shootSingle(0.15).alongWith(shooter.toAngleCommand(Rotation2d.fromDegrees(20))));

		// .whileTrue(new StartEndCommand(() -> shooter.runFeederMotor(0.4), () -> shooter.runFeederMotor(0)));

	}

	public static Command getAutonomousCommand() {
		return autoChooser.get();
	}
}
