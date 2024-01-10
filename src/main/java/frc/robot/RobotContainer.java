package frc.robot;

import static frc.robot.constants.Constants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.GoToRing;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.util.LocalADStarAK;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
	public static Robot robot;
	public static SwerveSubsystem drive;
	public static IMU imu = new IMU();
	public static PhotonVision photon = new PhotonVision();
	private static LoggedDashboardChooser<Command> autoChooser;

	public static CommandJoystick left_js = new CommandJoystick(1);
	public static CommandJoystick right_js = new CommandJoystick(2);
	public static CommandJoystick ds = new CommandJoystick(0);

	public static void initSubsystems() {
		drive = new SwerveSubsystem();
		drive.setDefaultCommand(new DefaultDrive(
				() -> left_js.getY() * SPEED_MULT, () -> left_js.getX() * SPEED_MULT, () -> -right_js.getX()));
		drive.init(new Pose2d(1, 1, new Rotation2d()));
		drive.initShuffleboard();
		autoChooser = new LoggedDashboardChooser<>("Auto Routine", AutoBuilder.buildAutoChooser());
		Pathfinding.setPathfinder(new LocalADStarAK());
		Shuffleboard.getTab("Swerve").add("Auto Selector", autoChooser.getSendableChooser());
	}

	public static void configureButtonBindings() {
		// left_js.button(3).and(left_js.button(1)).whileTrue(drive.followPath("New Path"));
		ds.button(11).onTrue(new InstantCommand(() -> photon.switchMode(1)));
		ds.button(12).onTrue(new InstantCommand(() -> photon.switchMode(0)));
		// ds.button(1).onTrue(new InstantCommand(()-> photon.getNearestNote(drive.getPose())));
		left_js.button(2).onTrue(new GoToRing().until(() -> right_js.button(2).getAsBoolean()));
	}

	public static Command getAutonomousCommand() {
		return autoChooser.get();
	}
}
