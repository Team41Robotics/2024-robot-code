package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.Combinations.AimBot;
import frc.robot.commands.Combinations.Handoff;
import frc.robot.commands.Combinations.ShootCycle;
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

/**
 * The RobotContainer class is responsible for initializing and configuring the robot's subsystems, sensors, and commands.
 */
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

	/**
	 * Initializes the subsystems of the robot.
	 * This method sets up the default command for the drive subsystem,
	 * initializes the LED subsystem, and registers various commands for
	 * controlling the shooter, intake, and other subsystems.
	 */
	public static void initSubsystems() {
		drive.setDefaultCommand(new DefaultDrive(() -> left_js.getY(), () -> left_js.getX(), () -> -right_js.getX()));
		drive.init(new Pose2d(8, 6, new Rotation2d(Math.PI)));

		leds.init();

		// NamedCommands.registerCommand("GoToRing", new GoToRing());
		NamedCommands.registerCommand("ShooterUp", shooter.toAngleDegreeCommand(15));
		NamedCommands.registerCommand("RunFeeder", shooter.loadNote());
		NamedCommands.registerCommand("RunIntake", (intake.runIntake(0.75).until(() -> !intake.intakeSwitch())));
		NamedCommands.registerCommand(
				"AutoShoot",
				new ParallelCommandGroup(new FaceSpeakerDrive(), shooter.autoShoot())
						.until(() -> !shooter.ringLoaded()));
		NamedCommands.registerCommand("Handoff", new Handoff());

		NamedCommands.registerCommand("StopShooter", new InstantCommand(() -> shooter.runMotors(0)));
		NamedCommands.registerCommand("PreRun", new InstantCommand(() -> shooter.runMotors(0.5)));
		NamedCommands.registerCommand(
				"WaitForShooter",
				new WaitUntilCommand(() -> shooter.isReady()).withTimeout(3).andThen(new PrintCommand("waited doen")));
		NamedCommands.registerCommand(
				"ActuallyShoot",
				new StartEndCommand(() -> shooter.runFeederMotor(0.4), () -> shooter.runFeederMotor(0))
						.until(() -> !shooter.ringLoaded()));
		NamedCommands.registerCommand("IntakeDown", new SetPivot(115));
		NamedCommands.registerCommand("WaitForIntake", new WaitUntilCommand(() -> !intake.intakeSwitch()));
		NamedCommands.registerCommand(
				"ShootCycle",
				new SequentialCommandGroup(
						new InstantCommand(() -> shooter.runMotors(0.5)),
						shooter.loadNote(),
						new ParallelRaceGroup(
								new WaitUntilCommand(() -> shooter.isReady()).withTimeout(70),
								new ParallelCommandGroup(new FaceSpeakerDrive(), shooter.autoShoot())
										.until(() -> !shooter.ringLoaded())),
						new StartEndCommand(() -> shooter.runFeederMotor(0.4), () -> shooter.runFeederMotor(0))
								.until(() -> !shooter.ringLoaded()),
						new InstantCommand(() -> shooter.runMotors(0.5))));
		NamedCommands.registerCommand(
				"ShootCycle2",
				new SequentialCommandGroup(
						new InstantCommand(() -> shooter.runMotors(0.5)),
						shooter.loadNote(),
						new ParallelRaceGroup(
								new WaitUntilCommand(() -> shooter.isReady()).withTimeout(70),
								new ParallelCommandGroup(shooter.autoShoot()).until(() -> !shooter.ringLoaded())),
						new StartEndCommand(() -> shooter.runFeederMotor(0.4), () -> shooter.runFeederMotor(0))
								.until(() -> !shooter.ringLoaded()),
						new InstantCommand(() -> shooter.runMotors(0.5))));

		Pathfinding.setPathfinder(new LocalADStarAK());

		autoChooser = new LoggedDashboardChooser<>("Auto Routine", AutoBuilder.buildAutoChooser());
		Shuffleboard.getTab("Swerve").add("Auto Selector", autoChooser.getSendableChooser());
	}

	/**
	 * Configures the button bindings for the robot.
	 */
	public static void configureButtonBindings() {

		ds.button(1).onTrue(shooter.subwooferShot());
		ds.button(2).whileTrue(new manualElevator());
		ds.button(2).onTrue(shooter.stopMotors());
		// handoff syste m
		right_js.button(1)
				.and(shooter::isReady)
				.and(shooter::ringLoaded)
				.whileTrue(new RunCommand(() -> leds.flashLeds(Color.kGreen))
						.finallyDo(() -> leds.rainbow().schedule()));

		right_js.button(2).onTrue(elevator.zeroEncoders());

		ds.button(3).onTrue(shooter.feederShot());
		left_js.button(1)
				.onTrue(intake.automaticIntake()
						.until(left_js.button(1).negate())
						.andThen(new Handoff().withTimeout(4)));

		left_js.button(2).onTrue(new ShootCycle(shooter));
		left_js.button(3).onTrue(shooter.loadNote());
		left_js.button(4)
				.onTrue(shooter.toAngleDegreeCommand(25)
						.alongWith(shooter.muzzleLoad()
								.andThen(
										shooter.toAngleDegreeCommand(45),
										new InstantCommand(() -> shooter.runMotors(0.3)))));

		right_js.button(1).and(() -> shooter.ringLoaded()).whileTrue(new AimBot(shooter, ds.button(15)));
		ds.button(9).onTrue(new SetPivot(115));
		ds.button(10).onTrue(new Handoff());

		right_js.button(4)
				.onTrue(new StartEndCommand(() -> shooter.runFeederMotor(0.4), () -> shooter.runFeederMotor(0))
						.until(() -> !shooter.ringLoaded()));
		ds.button(12).whileTrue(intake.runIntake(0.75)); // new Handoff());
		ds.button(11).whileTrue(intake.runIntake(-0.75)); // new Handoff());
		right_js.pov(0).onTrue(new InstantCommand(() -> leds.flashLeds(Color.kGreen)));
		right_js.pov(180).onTrue(leds.twinkle(Color.kPink));
		right_js.pov(270).onTrue(leds.twinkle(Color.kYellow));
		right_js.pov(90).onTrue(leds.rainbow());
		right_js.pov(45).onTrue(leds.fade(Color.kBlueViolet));

		ds.button(6).onTrue(shooter.toAngleDegreeCommand(65).andThen(shooter.shootSingle(0.45)));
		ds.button(14).onTrue(new SetPivot(-85).andThen(shooter.toAngleDegreeCommand(55)));

		ds.button(6).onTrue(new SetPivot(-85).andThen(shooter.toAngleCommand(Rotation2d.fromDegrees(45))));

		ds.button(7).onTrue(shooter.toAngleDegreeCommand(20).andThen(shooter.ampShoot()));
	}

	/**
	 * Returns the autonomous command to be executed by the robot.
	 *
	 * @return the autonomous command
	 */
	public static Command getAutonomousCommand() {
		return autoChooser.get();
	}
}
