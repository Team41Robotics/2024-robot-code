package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
	public static Robot robot;
	public static SwerveSubsystem drive;
	public static XboxController controller = new XboxController(0);

	public static void initSubsystems() {
		drive = new SwerveSubsystem();
	}
}
