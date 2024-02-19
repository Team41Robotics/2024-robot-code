package frc.robot.constants;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.util.Units;

public final class Constants {
	public static final double ROBOT_LENGTH = Units.inchesToMeters(29);
	public static final double ROBOT_WIDTH = Units.inchesToMeters(29);

	public static final double SWERVE_MAXSPEED = 4.42;
	public static final double ANGULAR_MAX_SPEED = SWERVE_MAXSPEED / (Math.hypot(ROBOT_LENGTH, ROBOT_WIDTH) / 2);

	public static final double SPEED_MULT = 0.55;
	public static final double TURBO_SPEED_MULT = 0.9;

	public static final double ANGULAR_SPEED_MULT = 0.5;
	public static final double TURBO_ANGULAR_SPEED_MULT = 0.9;

	public static final double SWERVE_WHEEL_RAD = 2 * 2.54 / 100;
	public static final double L2_DRIVE_RATIO = 1 / 6.75; // input RPM * gearing = output RPM
	public static final double L2_TURN_RATIO = 7. / 150;

	public static final double MODULE_DRIVE_KP = 0.05;
	public static final double MODULE_DRIVE_KF = 0.23;

	public static final double NOTE_VELOCITY = 11.6;

	public static final double DRIVER_TURN_KP = 1.;

	public static final double CAMERA_HEIGHT = Units.inchesToMeters(13.5);

	public static int SHOOTER_MOTOR_1 = 30;
	public static int SHOOTER_MOTOR_2 = 31;
	public static int SHOOTER_PIVOT_MOTOR1 = 13;
	public static int SHOOTER_PIVOT_MOTOR2 = 4;
	public static double SHOOTER_HEIGHT = Units.inchesToMeters(15);
	public static double TARGET_HEIGHT = Units.inchesToMeters(95);
	
	public static final HolonomicPathFollowerConfig PATH_FOLLOWER_CONFIG = new HolonomicPathFollowerConfig(
			new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
			new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
			4.5, // Max module speed, in m/s
			0.4488, // Drive base radius in meters. Distance from robot center to furthest module.
			new ReplanningConfig() // Default path replanning config. See the API for the options here
			);
}
