package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.*;
import frc.robot.subsystems.LEDS;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.util.LocalADStarAK;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
	public static Robot robot;
	public static SwerveSubsystem drive;
	public static ShooterSubsystem shooter;
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
		// left_js.button(2).onTrue(new GoToRing().until(() -> left_js.button(1).getAsBoolean()));
		right_js.button(2)
				.onTrue(new FaceSpeaker().until(() -> left_js.button(1).getAsBoolean()));
		right_js.button(1).whileTrue(new FaceSpeakerDrive());
	}

	public static Command getAutonomousCommand() {
		return autoChooser.get();
	}
}
