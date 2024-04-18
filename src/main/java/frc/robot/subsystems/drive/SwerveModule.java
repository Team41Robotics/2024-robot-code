package frc.robot.subsystems.drive;

import static frc.robot.constants.Constants.*;
import static frc.robot.constants.Constants.RobotConstants.*;
import static java.lang.Math.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.Logger;

/**
 * Represents a swerve module on a robot's drive system.
 * A swerve module consists of a drive motor and a turning motor.
 * It provides methods to control and monitor the state of the module.
 */
public class SwerveModule {
	public PIDController pidTurn = new PIDController(MODULE_TURN_KP, 0, 0);

	private ModuleIO io;
	private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
	private String name;

	double offset;
	SwerveModuleState target_state = new SwerveModuleState();

	public SwerveModule(ModuleIO io, String name) {
		this.io = io;
		this.name = name;
		pidTurn.enableContinuousInput(-PI, PI);
	}

	public double getDirection() {
		return inputs.turnAbsolutePosition.getRadians();
	}

	public double getVelocity() { // in m/s
		return inputs.driveVelocityMetersPerSec;
	}

	public double getDrivePosition() {
		return inputs.drivePositionRad * SWERVE_WHEEL_RAD;
	}

	/**
	 * Sets the target state of the SwerveModule.
	 * The state is optimized based on the current direction of the module.
	 * 
	 * @param state the desired state of the SwerveModule
	 */
	public void setState(SwerveModuleState state) {
		state = SwerveModuleState.optimize(state, new Rotation2d(getDirection()));
		target_state = state;
	}

	/**
	 * Returns the position of the swerve module.
	 *
	 * @return the position of the swerve module
	 */
	public SwerveModulePosition getPosition() {
		return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getDirection()));
	}

	/**
	 * Returns the target state of the SwerveModule.
	 *
	 * @return the target state of the SwerveModule
	 */
	public SwerveModuleState getTargetState() {
		return target_state;
	}

	/**
	 * Returns the measured state of the swerve module.
	 * 
	 * @return The measured state of the swerve module.
	 */
	public SwerveModuleState getMeasuredState() {
		return new SwerveModuleState(inputs.driveVelocityMetersPerSec, new Rotation2d(getDirection()));
	}

	public void fixOffset() {
		System.out.println("ERROR Offset for Cancoder: " + this.name + " is: " + getDirection() / 2 / PI);
	}

	/**
	 * Executes periodic actions for the SwerveModule.
	 * Calculates the target velocity based on the target state angle and speed.
	 * Sets the turn voltage using PID control based on the current direction and target state angle.
	 * Sets the drive velocity to the calculated target velocity.
	 * Logs the target state, inputs, and target velocity.
	 * Updates the inputs for the SwerveModule.
	 * Processes the inputs and logs them using the Logger.
	 */
	public void periodic() {
		double target_vel = Math.abs(Math.cos((getDirection() - target_state.angle.getRadians())))
				* target_state.speedMetersPerSecond;

		io.setTurnVoltage(pidTurn.calculate(getDirection(), target_state.angle.getRadians()));
		io.setDriveVelocity(target_vel);

		io.logTargetState(inputs, target_state, target_vel);
		io.updateInputs(inputs);
		Logger.processInputs("Drive/Module" + name, inputs);
	}
}
