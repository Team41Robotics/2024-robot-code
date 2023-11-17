package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DefaultDrive;
import frc.robot.subsystems.drive.SwerveSubsystem;

public class RobotContainer {
	public static Robot robot;
	public static SwerveSubsystem drive;
	public static IMU imu = new IMU();

	public static CommandXboxController controller = new CommandXboxController(0);
	// public static CommandJoystick left_js = new CommandJoystick(1);
	// public static CommandJoystick right_js = new CommandJoystick(2);

	public static void initSubsystems() {
		drive = new SwerveSubsystem();
		drive.setDefaultCommand(new DefaultDrive());
		drive.initOdom(new Pose2d());
		drive.initShuffleboard();
	}

	public static void configureButtonBindings() {
		controller.y().onTrue(new InstantCommand(() -> imu.zeroYaw()));
		// left_js.button(5).onTrue(new InstantCommand(() -> drive.getOffsets()));
		// left_js.button(4).onTrue(new InstantCommand(() -> imu.zeroYaw()));
	}
}
