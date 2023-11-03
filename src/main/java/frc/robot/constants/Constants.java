package frc.robot.constants;

public final class Constants {
	public static final double SWERVE_MAXSPEED = 1; // TODO
	public static final double SWERVE_WHEEL_RAD = 2 * 2.54 / 100;
	public static final double L2_DRIVE_RATIO = 1 / 6.75; // input RPM * gearing = output RPM
	public static final double L2_TURN_RATIO = 7. / 150;

	// public static final double DRIVE_KS = 0.1; // TODO
	// public static final double DRIVE_KV = 0.2;
	// public static final double TURN_KS = 0.05; // TODO
	// public static final double TURN_KV = 0.1;

	// public static final TrapezoidProfile.Constraints SWERVE_TURN_TRAPEZOID = new TrapezoidProfile.Constraints(1, 2);

	public static final double ROBOT_LENGTH = 23.5 * 2.54 / 100;
	public static final double ROBOT_WIDTH = 26.4 * 2.54 / 100;

	public static final double NW_ENCODER_OFFSET = 0.075195;
	public static final double NE_ENCODER_OFFSET = 0.066162;
	public static final double SW_ENCODER_OFFSET = 0.111084;
	public static final double SE_ENCODER_OFFSET = 0.915283;
}
