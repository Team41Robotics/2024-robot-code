package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.*;
import frc.robot.commands.Shooter.toAngle;
import frc.robot.commands.drive.DefaultDrive;
import frc.robot.commands.drive.FaceSpeaker;
import frc.robot.commands.drive.FaceSpeakerDrive;
import frc.robot.commands.drive.GoToRing;
import frc.robot.subsystems.LEDS;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.SparkFlexIO;
import frc.robot.util.LocalADStarAK;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
	public static Robot robot;
	public static SwerveSubsystem drive;
	public static ShooterSubsystem shooter = new ShooterSubsystem(new SparkFlexIO());
	public static IntakeSubsystem intake = new IntakeSubsystem();
	public static IMU imu = new IMU();
	public static PhotonVision photon = new PhotonVision();
	private static LoggedDashboardChooser<Command> autoChooser;
	private static LEDS leds = new LEDS();
	public static CommandJoystick left_js = new CommandJoystick(1);
	public static CommandJoystick right_js = new CommandJoystick(2);
	public static CommandJoystick ds = new CommandJoystick(0);

	public static void initSubsystems() {
		drive = new SwerveSubsystem();
		drive.setDefaultCommand(new DefaultDrive(() -> left_js.getY(), () -> left_js.getX(), () -> -right_js.getX()));
		drive.init(new Pose2d(1, 1, new Rotation2d(Math.PI)));
		drive.initShuffleboard();
		autoChooser = new LoggedDashboardChooser<>("Auto Routine", AutoBuilder.buildAutoChooser());
		Pathfinding.setPathfinder(new LocalADStarAK());
		Shuffleboard.getTab("Swerve").add("Auto Selector", autoChooser.getSendableChooser());
		NamedCommands.registerCommand("FACERING", new GoToRing());
	}

	public static void configureButtonBindings() {

		ds.button(1).onTrue(new InstantCommand(() -> left_js.button(1).getAsBoolean()));
		//left_js.button(2).onTrue(new GoToRing().until(() -> left_js.button(1).getAsBoolean()));
		left_js.button(2).onTrue(new toAngle(45));//.155));
		right_js.button(2).onTrue(new toAngle(90));
		left_js.button(4).onTrue(new toAngle(() -> shooter.calculateAngle()));
		right_js.button(3)
				.onTrue(new FaceSpeaker().until(() -> left_js.button(1).getAsBoolean()));
		right_js.button(1).whileTrue(new FaceSpeakerDrive());
		left_js.button(3).onTrue(new InstantCommand(()->shooter.zeroAbsoluteEncoder()));
		ds.button(7).onTrue(new InstantCommand(() -> drive.note_vel -= 0.1));
		ds.button(8).onTrue(new InstantCommand(() -> drive.note_vel += 0.1));
		ds.button(6).whileTrue(new StartEndCommand(()->shooter.runIntakeMotor(0.5), ()->shooter.runIntakeMotor(0)));
		ds.button(1).onTrue(new InstantCommand(() -> shooter.setSpeed(0.65)));
		ds.button(3).onTrue(new InstantCommand(() -> shooter.setSpeed(0)));
	}

	public static Command getAutonomousCommand() {
		return autoChooser.get();
	}
}
