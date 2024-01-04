package frc.robot.constants;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

public final class Constants {
	public static final double SWERVE_MAXSPEED = 4.42; // TODO
	public static final double SPEED_MULT = 0.6; // TODO
	public static final double SWERVE_WHEEL_RAD = 2 * 2.54 / 100;
	public static final double L2_DRIVE_RATIO = 1 / 6.75; // input RPM * gearing = output RPM
	public static final double L2_TURN_RATIO = 7. / 150;

	// public static final double DRIVE_KS = 0.1; // TODO
	// public static final double DRIVE_KV = 0.2;
	// public static final double TURN_KS = 0.05; // TODO
	// public static final double TURN_KV = 0.1;
	public static final double DRIVE_KP = 0.05;
	public static final double DRIVE_KF = 0.23;
	public static final double TURN_KP = 0;
	public static final double TURN_KF = 0;

	// public static final TrapezoidProfile.Constraints SWERVE_TURN_TRAPEZOID = new TrapezoidProfile.Constraints(1, 2);

	public static final double ROBOT_LENGTH = 23.5 * 2.54 / 100;
	public static final double ROBOT_WIDTH = 26.4 * 2.54 / 100;

	public static final HolonomicPathFollowerConfig PATH_FOLLOWER_CONFIG =
			new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your
					// Constants class
					new PIDConstants(1.0, 0.0, 0.0), // Translation PID constants
					new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
					4.5, // Max module speed, in m/s
					0.4488, // Drive base radius in meters. Distance from robot center to furthest module.
					new ReplanningConfig() // Default path replanning config. See the API for the options here
					);
}
