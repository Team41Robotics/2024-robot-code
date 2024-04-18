package frc.robot;

import static java.lang.Math.*;

import com.kauailabs.navx.frc.AHRS;

/**
 * The IMU class represents an Inertial Measurement Unit that provides
 * orientation and motion sensing capabilities.
 */
public class IMU {
	private AHRS imu;

	/**
	 * Constructs a new IMU object.
	 */
	public IMU() {
		imu = new AHRS();
	}

	/**
	 * Returns the yaw angle in radians.
	 * Z-axis is up, so the yaw angle is positive when the robot turns counterclockwise.
	 *
	 * @return the yaw angle in radians
	 */
	public double yaw() {
		return -imu.getAngle() / 180 * PI;
	}

	/**
	 * Resets the yaw angle to zero. DANGEROUS: CAN MESS WITH ODOMETRY.
	 */
	public void zeroYaw() {
		imu.zeroYaw();
	}
}
