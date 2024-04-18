package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

/**
 * The ShooterIO interface represents the input/output functionality of the shooter subsystem.
 * It provides methods to set the velocity of the shooter motors, stop the motors, and update the beam brake state.
 * 
 * NOT USED CURRENTLY
 */
public interface ShooterIO {

	@AutoLog
	public class ShooterIOInputs {

		public double currentVelocityMotor1 = 0.0;
		public double currentVelocityMotor2 = 0.0;
		public double currentVoltageMotor1 = 0.0;
		public double currentVoltageMotor2 = 0.0;
		public double[] currentAppliedAmpsMotor1 = {};
		public double[] currentAppliedAmpsMotor2 = {};
		public boolean currentBeamBrakeState = false;
	}

	public default void setVelocity(double speed) {}

	public default void stopMotors() {}

	public default void updateBeamBrake(boolean state) {}
}
