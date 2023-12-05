package frc.robot.subsystems.drive;

import static frc.robot.constants.Constants.*;
import static java.lang.Math.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import org.littletonrobotics.junction.Logger;

public class SwerveModule {
	public PIDController pidTurn = new PIDController(3, 0, 0);
	public PIDController pidSpeed = new PIDController(3, 0.05, 0);
	public double kV = 3;

	public static final double MAX_ACCEL = 20;
	public TrapezoidProfile speed_profile = new TrapezoidProfile(new Constraints(MAX_ACCEL, 1e5));

	// public static final State zeroState = new State(0, 0);
	// public TrapezoidProfile profile = new TrapezoidProfile(SWERVE_TURN_TRAPEZOID, zeroState, zeroState);
	// public double profile_t0 = Timer.getFPGATimestamp();
	private ModuleIO io;
	private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
	private int index;

	double offset;
	SwerveModuleState target_state = new SwerveModuleState();

	public SwerveModule(ModuleIO io, int index) {
		this.io = io;
		this.index = index;
		pidTurn.enableContinuousInput(-PI, PI);
	}

	public double getDirection() {
		return inputs.turnAbsolutePosition.getRadians();
	}

	public double getVelocity() { // in m/s
		return inputs.driveVelocityRadPerSec * SWERVE_WHEEL_RAD;
	}

	public double getDrivePosition() {
		return inputs.drivePositionRad * SWERVE_WHEEL_RAD;
	}

	public void setState(SwerveModuleState state) {
		state = SwerveModuleState.optimize(state, new Rotation2d(getDirection()));
		target_state = state;
		// double delta = getDirection() - state.angle.getRadians();
		// delta = MathUtil.angleModulus(delta);
		// profile = new TrapezoidProfile(SWERVE_TURN_TRAPEZOID, new State(delta, getAngularVelocity()), zeroState);
		// profile_t0 = Timer.getFPGATimestamp();
	}

	public SwerveModulePosition getPosition() {
		return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getDirection() + PI / 2));
	}

	public SwerveModuleState getTargetState() {
		return target_state;
	}

	public SwerveModuleState getMeasuredState() {
		return new SwerveModuleState(inputs.driveVelocityRadPerSec * SWERVE_WHEEL_RAD, new Rotation2d(getDirection()));
	}

	double drive_v, turn_v;

	public void fixOffset() {
		System.out.println("ERROR Offset for Cancoder: " + this.index + " is: " + getDirection() / 2 / PI);
	}

	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);

		double target_vel = Math.abs(Math.cos((getDirection() - target_state.angle.getRadians())))
				* target_state.speedMetersPerSecond;
		double target_vel_profiled = speed_profile.calculate(
				0.020,
				new State(target_vel, 0),
				new State(getVelocity(), 0)).position;
		Logger.recordOutput("Drive/Module"+Integer.toString(index)+"/target_vel", target_vel);
		Logger.recordOutput("Drive/Module"+Integer.toString(index)+"/target_vel_profiled", target_vel_profiled);
		io.setDriveVoltage(target_vel_profiled * kV +
			pidSpeed.calculate(getMeasuredState().speedMetersPerSecond, target_vel_profiled));
		io.setTurnVoltage(pidTurn.calculate(getDirection(), target_state.angle.getRadians()));
	}
}
