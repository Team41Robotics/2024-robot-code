package frc.robot.util;

import static frc.robot.commands.constants.Constants.FieldConstants.*;
import static frc.robot.commands.constants.Constants.RobotConstants.*;
import static java.lang.Math.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Util {
	// tuned value for sigmoid, higher values make the curve steeper, this is what thomas likes. Use desmos to preview
	// curves
	private static double a = 3;

	/**
	 * Applies a sensitivity curve to a given input value.
	 * If the absolute value of the input is less than the deadband value, the output is 0.
	 * Otherwise, the output is calculated using a sigmoid function and the sign of the input.
	 *
	 * @param w the input value
	 * @param d the deadband value
	 * @return the output value after applying the sensitivity curve
	 */
	public static double sensCurve(double w, double d) {
		if (abs(w) < d) return 0;
		double wn = (abs(w) - d) / (1 - d);

		wn = sigmoid(wn) / sigmoid(1);
		return wn * signum(w);
	}

	/**
	 * Calculates the sigmoid function of a given input.
	 * The sigmoid function maps the input to a value between -1 and 1.
	 *
	 * @param x the input value
	 * @return the sigmoid value of the input
	 */
	public static double sigmoid(double x) {
		return 2 / (1.0 + exp(-x * a)) - 1;
	}

	/**
	 * Represents the speeds of a robot chassis in field-relative coordinates.
	 * The speeds include the linear velocity components in the x and y directions,
	 * as well as the angular velocity component around the robot's center of rotation.
	 *
	 * Also, the sensitivity curve is basically a clamped linear function right now :/
	 *
	 * @see <a href="https://www.desmos.com/calculator/mbxig6izt9">Desmos Graph</a>
	 * @param vx The linear velocity component in the x direction.
	 * @param vy The linear velocity component in the y direction.
	 * @param w The angular velocity component around the robot's center of rotation.
	 * @param turbo A boolean indicating whether the robot is in turbo mode.
	 * @param rot The rotation of the robot chassis.
	 * @return The ChassisSpeeds object representing the speeds of the robot chassis.
	 */
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

	/**
	 * Returns whether the robot is on the red alliance.
	 *
	 * @return true if the robot is on the red alliance, false otherwise
	 */
	public static boolean isRed() {
		return DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red);
	}

	/**
	 * Returns the x-coordinate of the target based on the alliance color.
	 *
	 * @return the x-coordinate of the target
	 */
	public static double getTargetX() {
		return isRed() ? TARGET_X_RED : TARGET_X_BLUE;
	}
}
