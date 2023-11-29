package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public final class Constants {
	public static final double SWERVE_MAXSPEED = 4.42; // TODO
	public static final double SPEED_MULT = 0.5; // TODO
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

	// TODO TODO TODO TODO fix offsets
	public static final Rotation2d NW_ENCODER_OFFSET = new Rotation2d(Units.rotationsToRadians(.421630859375));
	public static final Rotation2d NE_ENCODER_OFFSET = new Rotation2d(Units.rotationsToRadians(0.609619140625));
	public static final Rotation2d SW_ENCODER_OFFSET = new Rotation2d(Units.rotationsToRadians(0.56591796875));
	public static final Rotation2d SE_ENCODER_OFFSET = new Rotation2d(Units.rotationsToRadians(0.5732421875));
}
