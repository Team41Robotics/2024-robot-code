package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.Handoff;
import frc.robot.commands.drive.DefaultDrive;
import frc.robot.commands.drive.FaceSpeaker;
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
import frc.robot.util.Util;
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
		drive.init(new Pose2d(8, 6, new Rotation2d(Math.PI)));

		leds.init();

		// NamedCommands.registerCommand("GoToRing", new GoToRing());
		NamedCommands.registerCommand("FaceSpeaker", new FaceSpeaker());
		NamedCommands.registerCommand("ShooterUp", shooter.toAngleDegreeCommand(15));
		NamedCommands.registerCommand("RunFeeder", shooter.runFeeder());
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

		NamedCommands.registerCommand(
				"ShootCycle",
				new SequentialCommandGroup(
						new InstantCommand(() -> shooter.runMotors(0.5)),
						shooter.runFeeder(),
						new ParallelRaceGroup(
								new WaitUntilCommand(() -> shooter.isReady()).withTimeout(70),
								new ParallelCommandGroup(new FaceSpeakerDrive(), shooter.autoShoot())
										.until(() -> !shooter.ringLoaded())),
						new StartEndCommand(() -> shooter.runFeederMotor(0.4), () -> shooter.runFeederMotor(0))
								.until(() -> !shooter.ringLoaded()),
						new InstantCommand(() -> shooter.runMotors(0.5))));

		Pathfinding.setPathfinder(new LocalADStarAK());

		autoChooser = new LoggedDashboardChooser<>("Auto Routine", AutoBuilder.buildAutoChooser());
		Shuffleboard.getTab("Swerve").add("Auto Selector", autoChooser.getSendableChooser());
	}

	public static void configureButtonBindings() {
		// straight up
		ds.button(1).onTrue(shooter.toAngleDegreeCommand(25).andThen(shooter.shootSingle(0.7)));
		ds.button(2).whileTrue(new manualElevator());
		// handoff syste m
		ds.button(3).whileTrue(new RunCommand(() -> drive.resetOdom()));
		//	ds.button(3).onTrue((intake.runIntake(0.75).until(() -> !intake.intakeSwitch())).andThen(new Handoff()));
		left_js.button(3).onTrue(shooter.runFeeder());
		left_js.button(2)
				.onTrue(new SequentialCommandGroup(
						new InstantCommand(() -> shooter.runMotors(0.5)),
						shooter.runFeeder(),
						new ParallelRaceGroup(
								new WaitCommand(0.5)
										.andThen(new WaitUntilCommand(() -> shooter.isReady()).withTimeout(70)),
								new ParallelCommandGroup(new FaceSpeakerDrive(), shooter.autoShoot())
										.until(() -> !shooter.ringLoaded())),
						new WaitCommand(0.5),
						new StartEndCommand(() -> shooter.runFeederMotor(0.4), () -> shooter.runFeederMotor(0))
								.until(() -> !shooter.ringLoaded()),
						new InstantCommand(() -> shooter.runMotors(0.5))));
		left_js.button(1)
				.onTrue(new SetPivot(115)
						.andThen(new WaitUntilCommand(() -> intake.getAngle().getDegrees() > 0))
						.andThen((intake.runIntake(0.75)
										.until(() -> !intake.intakeSwitch())
										.until(left_js.button(1).negate()))
								.andThen(new Handoff().withTimeout(4))));
		// left_js.button(2)
		//		.whileTrue(new StartEndCommand(() -> leds.flashLeds(Color.kPink), () -> leds.flashLeds(Color.kBlack)));
		// left_js.button(2).onTrue(new InstantCommand(() -> imu.zeroYaw()));
		left_js.button(4)
				.onTrue(shooter.toAngleDegreeCommand(25)
						.alongWith(shooter.muzzleLoad()
								.andThen(
										shooter.toAngleDegreeCommand(75),
										new InstantCommand(() -> shooter.runMotors(0.6)))));

		right_js.button(1)
				.and(() -> shooter.ringLoaded())
				.whileTrue(new ConditionalCommand(
						new ParallelCommandGroup(new FaceSpeakerDrive(), shooter.autoShoot())
								.until(() -> !shooter.ringLoaded())
								.andThen(shooter.toAngleDegreeCommand(90)),
						shooter.toAngleDegreeCommand(20).andThen(new InstantCommand(() -> shooter.runMotors(.7))),
						ds.button(15)));
		// right_js.button(1).whileTrue(intake.runIntake(0.6));
		ds.button(9).onTrue(new SetPivot(115));
		ds.button(10).onTrue(new Handoff());

		right_js.button(4)
				.onTrue(new StartEndCommand(() -> shooter.runFeederMotor(0.4), () -> shooter.runFeederMotor(0))
						.until(() -> !shooter.ringLoaded()));
		ds.button(12).whileTrue(intake.runIntake(0.75)); // new Handoff());
		ds.button(11).whileTrue(intake.runIntake(-0.75)); // new Handoff());
		right_js.pov(0).onTrue(new InstantCommand(() -> leds.flashLeds(Color.kGreen)));
		right_js.pov(180).onTrue(leds.twinkle((Util.isRed() ? Color.kRed : Color.kBlue)));
		right_js.pov(270).onTrue(leds.RGBFade());
		right_js.pov(90).onTrue(leds.rainbow());
		right_js.pov(45).onTrue(leds.fade(Color.kBlueViolet));

		// ds.button(12).onTrue(shooter.autoShoot());
		// ds.button(6).onTrue(shooter15.toAngleCommand(Rotation2d.fromDegrees(55)));
		ds.button(6).onTrue(shooter.toAngleDegreeCommand(25).andThen(shooter.shootSingle(0.125)));
		ds.button(14).onTrue(new SetPivot(-85).andThen(shooter.toAngleDegreeCommand(55)));
		ds.button(2)
				.whileTrue(new InstantCommand(() -> shooter.runMotors(0))
						.andThen(new InstantCommand(
								() -> CommandScheduler.getInstance().cancelAll())));
		ds.button(3).whileTrue(new InstantCommand(() -> leds.flashLeds(new Color(255, 0, 0))));
		// .whileTrue(new StartEndCommand(() -> shooter.runFeederMotor(0.4), () -> shooter.runFeederMotor(0)));

	}

	public static Command getAutonomousCommand() {
		return autoChooser.get();
	}
}
