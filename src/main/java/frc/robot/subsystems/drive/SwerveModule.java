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
		Logger.recordOutput("Drive/Module"+Integer.toString(index)+"/target_vel", target_vel);
		io.setDriveVoltage(target_vel * kV +
			pidSpeed.calculate(getMeasuredState().speedMetersPerSecond, target_vel));
		io.setTurnVoltage(pidTurn.calculate(getDirection(), target_state.angle.getRadians()));
	}
}
