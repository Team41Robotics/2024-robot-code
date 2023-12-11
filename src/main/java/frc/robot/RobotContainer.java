package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DefaultDrive;
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

	public static CommandXboxController controller = new CommandXboxController(0);
	// public static CommandJoystick left_js = new CommandJoystick(1);
	// public static CommandJoystick right_js = new CommandJoystick(2);

	public static void initSubsystems() {
		drive = new SwerveSubsystem();
		drive.setDefaultCommand(new DefaultDrive());
		drive.init(new Pose2d(1, 1, new Rotation2d()));
		drive.initShuffleboard();
		autoChooser = new LoggedDashboardChooser<>("Auto Routine", AutoBuilder.buildAutoChooser());
		Pathfinding.setPathfinder(new LocalADStarAK());
		Shuffleboard.getTab("Swerve").add("Auto Selector", autoChooser.getSendableChooser());
	}

	public static void configureButtonBindings() {
		controller.y().onTrue(new InstantCommand(() -> imu.zeroYaw()));
		controller.x().onTrue(new InstantCommand(() -> drive.getOffsets()));
		// left_js.button(4).onTrue(new InstantCommand(() -> imu.zeroYaw()));
	}

	public static Command getAutonomousCommand() {
		return autoChooser.get();
	}
}
