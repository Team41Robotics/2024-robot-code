package frc.robot.util;

import static frc.robot.constants.Constants.*;
import static java.lang.Math.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Util {
	private static double a = 3;

	public static double sensCurve(double w, double d) {
		if (abs(w) < d) return 0;
		double wn = (w - d) / (1 - d);

		wn = sigmoid(wn) / sigmoid(1);
		return wn * signum(w);
	}

	public static double sigmoid(double x) {
		return 2 / (1.0 + exp(-x * a)) - 1;
	}

	public static ChassisSpeeds joystickToSpeeds(double vx, double vy, double w, boolean turbo, Rotation2d rot) {
		double mag = Math.hypot(vx, vy);
		double mag_curved = MathUtil.clamp(Util.sensCurve(mag, 0.15) * 1.5, -1, 1);

		double theta = Math.atan2(vy, vx);
		double sign = isRed() ? -1.0 : 1.0;

		double speed_mult = turbo ? TURBO_SPEED_MULT : SPEED_MULT;
		double angular_mult = turbo ? TURBO_ANGULAR_SPEED_MULT : ANGULAR_SPEED_MULT;
		return ChassisSpeeds.fromFieldRelativeSpeeds(
				cos(theta) * mag_curved * SWERVE_MAXSPEED * speed_mult * sign,
				sin(theta) * mag_curved * SWERVE_MAXSPEED * speed_mult * sign,
				MathUtil.applyDeadband(w, 0.1) * ANGULAR_MAX_SPEED * angular_mult,
				rot);
	}

	public static boolean isRed() {
		return DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red);
	}

	public static double getTargetX() {
		return isRed() ? TARGET_X_RED : TARGET_X_BLUE;
	}
}
