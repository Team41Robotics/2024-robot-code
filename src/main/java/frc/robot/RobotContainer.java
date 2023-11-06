package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
	public static Robot robot;
	public static SwerveSubsystem drive;
	public static AHRS imu = new AHRS();

	public static CommandXboxController controller = new CommandXboxController(0);

	public static void initSubsystems() {
		drive = new SwerveSubsystem();
	}

	public static void configureButtonBindings() {
		controller.y().onTrue(new InstantCommand(() -> imu.zeroYaw()));
	}
}
