package frc.robot.constants;

import static java.lang.Math.*;

import edu.wpi.first.math.geometry.Rotation2d;

public enum SwerveModuleConfiguration {
        NW(2, 3, 6, .4216),
        NE(5, 4, 7, .6096),
        SW(8, 9, 10, .5659),
        SE(11, 12, 13, .5732);

        public int ENCODER, TURN_MOTOR, DRIVE_MOTOR;
        public Rotation2d offset;

        SwerveModuleConfiguration(int enc, int tm, int dm, double offset_rot) {
                ENCODER = enc;
                TURN_MOTOR = tm;
                DRIVE_MOTOR = dm;
                offset = new Rotation2d(offset_rot * 2 * PI);
        }
}
