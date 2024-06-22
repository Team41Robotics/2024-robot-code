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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Combinations.AimBot;
import frc.robot.commands.Combinations.Handoff;
import frc.robot.commands.Combinations.ShootCycle;
import frc.robot.commands.drive.DefaultDrive;
import frc.robot.commands.drive.FaceSpeakerDrive;
import frc.robot.commands.drive.GoToRing;
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

	public static CommandXboxController xbox = new CommandXboxController(2);

	public static PhotonVision photon = new PhotonVision();
	public static LEDS leds = new LEDS();

	// other
	public static LoggedDashboardChooser<Command> autoChooser;

	public static IMU imu = new IMU();
	public static CommandJoystick left_js = new CommandJoystick(0);
	public static CommandJoystick right_js = new CommandJoystick(3);
	public static CommandJoystick ds = new CommandJoystick(4);

	/**
	 * Initializes the subsystems of the robot.
	 * This method sets up the default command for the drive subsystem,
	 * initializes the LED subsystem, and registers various commands for
	 * controlling the shooter, intake, and other subsystems.
	 */
	public static void initSubsystems() {
		drive.setDefaultCommand(new DefaultDrive(xbox::getLeftY, xbox::getLeftX, () -> -xbox.getRightX()));

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
		drive.init(new Pose2d());
		autoChooser = new LoggedDashboardChooser<>("Auto Routine", AutoBuilder.buildAutoChooser());
		Shuffleboard.getTab("Swerve").add("Auto Selector", autoChooser.getSendableChooser());
	}

	/**
	 * Configures the button bindings for the robot.
	 */
	public static void configureButtonBindings() {

		ds.button(1).onTrue(shooter.subwooferShot()); // aims for subwoofer 
		ds.button(2).whileTrue(new manualElevator()); // turns on manual elevator control
		ds.button(2).onTrue(shooter.stopMotors()); // stops the shooter 
		// handoff syste m
		right_js.button(1) // flashes leds green 
				.and(shooter::isReady)
				.and(shooter::ringLoaded)
				.whileTrue(new RunCommand(() -> leds.flashLeds(Color.kGreen))
						.finallyDo(() -> leds.rainbow().schedule()));

		right_js.button(2).onTrue(elevator.zeroEncoders()); // zeroes encoders

		ds.button(3).onTrue(shooter.feederShot()); // 
		left_js.button(1) // auto intake and handoff
				.onTrue(intake.automaticIntake()
						.until(left_js.button(1).negate())
						.andThen(new Handoff().withTimeout(4)));

		left_js.button(2).onTrue(new ShootCycle(shooter)); // aims for speaker and shoots
		left_js.button(3).onTrue(shooter.loadNote()); // runs shooter feed motors until beam brake is triggered
		left_js.button(4) // muzzle load for use at human player station
				.onTrue(shooter.toAngleDegreeCommand(25)
						.alongWith(shooter.muzzleLoad()
								.andThen(
										shooter.toAngleDegreeCommand(45),
										new InstantCommand(() -> shooter.runMotors(0.3)))));

		right_js.button(1).and(() -> shooter.ringLoaded()).whileTrue(new AimBot(shooter, ds.button(15))); // auto aim 
		ds.button(9).onTrue(new SetPivot(115)); // manually lower intake
		ds.button(10).onTrue(new Handoff()); // manually toggle handoff

		right_js.button(4).onTrue(shooter.fireNote()); // fires note
		ds.button(12).whileTrue(intake.runIntake(0.75)); // runs intake motors
		ds.button(11).whileTrue(intake.runIntake(-0.75)); // runs intake motors in reverse (use to spit out notes)
		right_js.pov(0).onTrue(new InstantCommand(() -> leds.flashLeds(Color.kGreen))); // flashes green
		right_js.pov(180).onTrue(leds.twinkle(Color.kPink)); // flashes pink 
		right_js.pov(270).onTrue(leds.twinkle(Color.kYellow));
		right_js.pov(90).onTrue(leds.rainbow());
		right_js.pov(45).onTrue(leds.fade(Color.kBlueViolet));

		ds.button(6).onTrue(shooter.toAngleDegreeCommand(65).andThen(shooter.shootSingle(0.45)));
		ds.button(14).onTrue(new SetPivot(-85).andThen(shooter.toAngleDegreeCommand(55)));

		ds.button(6).onTrue(new SetPivot(-85).andThen(shooter.toAngleCommand(Rotation2d.fromDegrees(45))));

		ds.button(7).onTrue(shooter.toAngleDegreeCommand(20).andThen(shooter.ampShoot())); // amp shot 
		ds.button(15).whileTrue(new GoToRing()); // auto align
		/*
		xbox.leftTrigger(0.4)
				.whileTrue((intake.automaticIntake()
						.until(left_js.button(1).negate())
						.andThen(new Handoff().withTimeout(4))));
		xbox.rightTrigger(0.4).onTrue(shooter.toAngleDegreeCommand(45).andThen(shooter.shootSingle(0.95)));
		xbox.x().onTrue(shooter.fireNote());
		*/
	}

	public static void configureOtherBindings() {
		/*
		left_js.button(1)
				.debounce(.1)
				.onTrue((intake.automaticIntake().until(left_js.button(2)).andThen(new Handoff().withTimeout(4))));
		right_js.button(2).onTrue(shooter.toAngleDegreeCommand(10));
		right_js.button(3).onTrue(shooter.toAngleDegreeCommand(60));
		right_js.button(4).onTrue(shooter.toAngleDegreeCommand(70));
		right_js.button(5).onTrue(shooter.toAngleDegreeCommand(40));
		left_js.button(4)
				.onTrue(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));
		right_js.button(1).onTrue(shooter.shootSingle(0.4));
		right_js.button(1).debounce(0.1).onFalse(shooter.fireNote());
		left_js.button(4).whileTrue(shooter.loadNote());
		right_js.pov(180).onTrue(leds.twinkle(Color.kPink));
		right_js.pov(270).onTrue(leds.twinkle(Color.kYellow));
		right_js.pov(90).onTrue(leds.rainbow());
		right_js.pov(45).onTrue(leds.fade(Color.kBlueViolet));

		left_js.button(5).onTrue(intake.toDegree(120));
		*/

		xbox.leftTrigger(0.4)
				.whileTrue((intake.xBoxIntake().until(xbox.button(4).negate()).andThen(new Handoff().withTimeout(4))));
		xbox.rightTrigger(0.4).onTrue(shooter.toAngleDegreeCommand(35).andThen(shooter.shootSingle(0.3)));
		xbox.button(10).onTrue(shooter.toAngleDegreeCommand(85).andThen(shooter.shootSingle(0.90)));
		xbox.button(1).onTrue(intake.xBoxIntake());
		xbox.button(2).onTrue(shooter.fireNote());
		xbox.button(6).onTrue(new InstantCommand(() -> shooter.runFeederMotor(.7)));
		xbox.button(6).onFalse(shooter.stopFeedMotor());
		xbox.button(5).onTrue(new InstantCommand(() -> intake.runIntakeMotor(0.5)));
		xbox.button(5).onFalse(new InstantCommand(() -> intake.stopIntakeMotor()));
		xbox.y().whileTrue(new GoToRing());
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
