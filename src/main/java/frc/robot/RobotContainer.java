package frc.robot;

import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
	public static Robot robot;
	public static SwerveSubsystem drive;

	public static void initSubsystems() {
		drive = new SwerveSubsystem();
	}
}
