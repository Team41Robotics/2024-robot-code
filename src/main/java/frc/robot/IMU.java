package frc.robot;

import static java.lang.Math.*;

import com.kauailabs.navx.frc.AHRS;

public class IMU {
	private AHRS imu;

	public IMU() {
		imu = new AHRS();
	}

	public double yaw() {
		return -(imu.getAngle() + 180 % 360) / 180 * PI;
	}

	public void zeroYaw() {
		imu.zeroYaw();
	}
}
