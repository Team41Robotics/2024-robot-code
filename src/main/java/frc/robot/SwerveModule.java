package frc.robot;

import static frc.robot.constants.Constants.*;
import static java.lang.Math.*;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
	public PIDController pidTurn = new PIDController(9, 0, 0);
	public PIDController pidSpeed = new PIDController(0, 0, 0);

	// public static final State zeroState = new State(0, 0);
	// public TrapezoidProfile profile = new TrapezoidProfile(SWERVE_TURN_TRAPEZOID, zeroState, zeroState);
	// public double profile_t0 = Timer.getFPGATimestamp();

	CANcoder encoder;
	CANSparkMax turn_motor, drive_motor;
	double offset;
	SwerveModuleState target_state = new SwerveModuleState();

	public SwerveModule(int port_cancoder, int port_turn_motor, int port_drive_motor, double offset) {
		encoder = new CANcoder(port_cancoder, "rio");
		turn_motor = new CANSparkMax(port_turn_motor, MotorType.kBrushless);
		drive_motor = new CANSparkMax(port_drive_motor, MotorType.kBrushless);
		this.offset = offset;
		pidTurn.enableContinuousInput(-PI, PI);
	}

	public double getDirection() {
		return (encoder.getAbsolutePosition().getValue() - this.offset) * 2 * PI;
	}

	public double getAngularVelocity() { // in rad/s
		return turn_motor.getEncoder().getVelocity() * 2 * PI / 60 * L2_TURN_RATIO;
	}

	public double getVelocity() { // in m/s
		return drive_motor.getEncoder().getVelocity() * 2 * PI / 60 * L2_DRIVE_RATIO * SWERVE_WHEEL_RAD;
	}

	public void setState(SwerveModuleState state) {
		state = SwerveModuleState.optimize(state, new Rotation2d(getDirection()));
		target_state = state;
		// double delta = getDirection() - state.angle.getRadians();
		// delta = MathUtil.angleModulus(delta);
		// profile = new TrapezoidProfile(SWERVE_TURN_TRAPEZOID, new State(delta, getAngularVelocity()), zeroState);
		// profile_t0 = Timer.getFPGATimestamp();
	}

	double drive_v, turn_v;

	public void fixOffset() {
		System.out.println("Offset for Cancoder: " + this.encoder.getDeviceID() + " is: " + getDirection() / 2 / PI);
	}

	public void periodic() {
		// State turn_ref = profile.calculate(Timer.getFPGATimestamp() - profile_t0);

		// drive_motor.setVoltage(
		// drive_v = DRIVE_KS * signum(target_state.speedMetersPerSecond)
		// + DRIVE_KV * target_state.speedMetersPerSecond
		// + pidSpeed.calculate(getVelocity(), target_state.speedMetersPerSecond));

		// turn_motor.setVoltage(
		// turn_v = TURN_KS * signum(turn_ref.velocity)
		// + TURN_KV * turn_ref.velocity
		// + pidTurn.calculate(getDirection(), turn_ref.position + target_state.angle.getRadians()));

		// jank wayy
		// if (this.encoder.getDeviceID() == 5)

		//	System.out.println(new Rotation2d(this.getDirection()) + " ; " + target_state);
		double MAX_SPEED = 1;
		drive_motor.setVoltage(Math.abs(Math.cos((getDirection() - target_state.angle.getRadians())))
				* target_state.speedMetersPerSecond
				/ MAX_SPEED
				* 9);
		// turn_motor.setVoltage(MathUtil.angleModulus(getDirection() - target_state.angle.getRadians()) * 2);
		turn_motor.setVoltage(pidTurn.calculate(getDirection(), target_state.angle.getRadians()));
		// System.out.println(target_state.speedMetersPerSecond / MAX_SPEED * 9);
		// System.out.println((MathUtil.angleModulus(getDirection() - target_state.angle.getRadians()) * 3));
	}
}
