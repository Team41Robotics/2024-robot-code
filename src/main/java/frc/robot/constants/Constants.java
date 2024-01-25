package frc.robot.constants;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.util.Units;

public final class Constants {
	public static final double ROBOT_LENGTH = 23.5 * 2.54 / 100;
	public static final double ROBOT_WIDTH = 26.4 * 2.54 / 100;

	public static final double SWERVE_MAXSPEED = 4.42;
	public static final double ANGULAR_SPEED = SWERVE_MAXSPEED / (Math.hypot(ROBOT_LENGTH, ROBOT_WIDTH) / 2);

	public static final double ANGULAR_SPEED_MULT = 0.4;

	public static final double SPEED_MULT = 0.4;
	public static final double SWERVE_WHEEL_RAD = 2 * 2.54 / 100;
	public static final double L2_DRIVE_RATIO = 1 / 6.75; // input RPM * gearing = output RPM
	public static final double L2_TURN_RATIO = 7. / 150;

	public static final double DRIVE_KP = 0.05;
	public static final double DRIVE_KF = 0.23;
	public static final double TURN_KP = 0;
	public static final double TURN_KF = 0;

	public static final double APRILTAG_4_Y = 5.548;

	public static final double cam_height = Units.inchesToMeters(9.65);

	public static final HolonomicPathFollowerConfig PATH_FOLLOWER_CONFIG = new HolonomicPathFollowerConfig(
			new PIDConstants(1.0, 0.0, 0.0), // Translation PID constants
			new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
			4.5, // Max module speed, in m/s
			0.4488, // Drive base radius in meters. Distance from robot center to furthest module.
			new ReplanningConfig() // Default path replanning config. See the API for the options here
			);
}
