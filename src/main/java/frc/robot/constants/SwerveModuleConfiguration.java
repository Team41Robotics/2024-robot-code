package frc.robot.constants;

import static java.lang.Math.*;

import edu.wpi.first.math.geometry.Rotation2d;

public enum SwerveModuleConfiguration {
	NW(2, 3, 6, .9216),
	NE(5, 4, 7, 1.1096),
	SW(8, 9, 10, 1.0659),
	SE(11, 12, 13, 1.0732);
	public int ENCODER, TURN_MOTOR, DRIVE_MOTOR;
	public Rotation2d offset;

	SwerveModuleConfiguration(int enc, int tm, int dm, double offset_rot) {
		ENCODER = enc;
		TURN_MOTOR = tm;
		DRIVE_MOTOR = dm;
		offset = new Rotation2d(offset_rot * 2 * PI);
	}
}
