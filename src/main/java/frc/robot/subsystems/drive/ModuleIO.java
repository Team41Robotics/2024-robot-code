package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
	@AutoLog
	public static class ModuleIOInputs {
		public double drivePositionRad = 0.0;
		public double driveVelocityMetersPerSec = 0.0;
		public double driveAppliedVolts = 0.0;
		public double[] driveCurrentAmps = new double[] {};

		public Rotation2d turnAbsolutePosition = new Rotation2d();
		public double turnAbsolutePositionRad = 0.0;
		public Rotation2d turnPosition = new Rotation2d();
		public double turnVelocityRadPerSec = 0.0;
		public double turnAppliedVolts = 0.0;
		public double[] turnCurrentAmps = new double[] {};

		public double targetRad = 0.0;
		public double targetVel = 0.0;
		public double compensatedTargetVel = 0.0;
	}

	/** Updates the set of loggable inputs. */
	public default void updateInputs(ModuleIOInputs inputs) {}

	/** Run the drive motor at the specified voltage. */
	public default void setDriveVoltage(double volts) {}

	/** Runs the Drive motor at specified velocity in m/s using integrated PID  controller */
	public default void setDriveVelocity(double mps) {}

	/** Run the turn motor at the specified voltage. */
	public default void setTurnVoltage(double volts) {}

	/** Enable or disable brake mode on the drive motor. */
	public default void setDriveBrakeMode(boolean enable) {}

	/** Enable or disable brake mode on the turn motor. */
	public default void setTurnBrakeMode(boolean enable) {}

	public default void logTargetState(ModuleIOInputs inputs, SwerveModuleState target, double compensatedVel) {}
}
