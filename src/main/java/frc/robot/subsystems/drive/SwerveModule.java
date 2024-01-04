package frc.robot.subsystems.drive;

import static frc.robot.constants.Constants.*;
import static java.lang.Math.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.Logger;

public class SwerveModule {
	public PIDController pidTurn = new PIDController(3, 0, 0);
	public PIDController pidSpeed = new PIDController(2, 0, 0);
	public double kV = 3;

	// public static final State zeroState = new State(0, 0);
	// public TrapezoidProfile profile = new TrapezoidProfile(SWERVE_TURN_TRAPEZOID, zeroState, zeroState);
	// public double profile_t0 = Timer.getFPGATimestamp();
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

	public void setState(SwerveModuleState state) {
		state = SwerveModuleState.optimize(state, new Rotation2d(getDirection()));
		target_state = state;
		// double delta = getDirection() - state.angle.getRadians();
		// delta = MathUtil.angleModulus(delta);
		// profile = new TrapezoidProfile(SWERVE_TURN_TRAPEZOID, new State(delta, getAngularVelocity()), zeroState);
		// profile_t0 = Timer.getFPGATimestamp();
	}

	public SwerveModulePosition getPosition() {
		return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getDirection()));
	}

	public SwerveModuleState getTargetState() {
		return target_state;
	}

	public SwerveModuleState getMeasuredState() {
		return new SwerveModuleState(inputs.driveVelocityMetersPerSec, new Rotation2d(getDirection()));
	}

	public void fixOffset() {
		System.out.println("ERROR Offset for Cancoder: " + this.name + " is: " + getDirection() / 2 / PI);
	}

	public void periodic() {
		// double target_vel = Math.abs(Math.cos((getDirection() - target_state.angle.getRadians())))
		//		* target_state.speedMetersPerSecond;
		double target_vel = target_state.speedMetersPerSecond;
		// io.setDriveVoltage(target_vel * kV + pidSpeed.calculate(getMeasuredState().speedMetersPerSecond,
		// target_vel));
		io.setTurnVoltage(pidTurn.calculate(getDirection(), target_state.angle.getRadians()));
		// io.setDriveVelocity(target_vel / SWERVE_WHEEL_RAD / 2.0 / PI / L2_DRIVE_RATIO * 60);
		io.setDriveVelocity(target_vel);
		io.updateInputs(inputs);
		io.logTargetState(inputs, target_state, target_vel);
		Logger.processInputs("Drive/Module" + name, inputs);
	}
}
