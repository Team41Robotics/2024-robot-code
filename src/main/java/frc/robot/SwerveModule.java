package frc.robot;

import static frc.robot.constants.Constants.*;
import static java.lang.Math.*;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
	public PIDController pidTurn = new PIDController(0, 0, 0, 0);
	public PIDController pidSpeed = new PIDController(0, 0, 0, 0);

	CANCoder encoder;
	CANSparkMax turn_motor, drive_motor;

	public SwerveModule(int port_cancoder, int port_turn_motor, int port_drive_motor) {
		encoder = new CANCoder(port_cancoder);
		turn_motor = new CANSparkMax(port_turn_motor, MotorType.kBrushless);
		drive_motor = new CANSparkMax(port_drive_motor, MotorType.kBrushless);
	}

	public double getDirection() {
		return encoder.getAbsolutePosition() / 180 * PI;
	}

	public void setState(SwerveModuleState swerveModuleState) {
		SwerveModuleState.optimize(swerveModuleState, new Rotation2d(getDirection()));
		pidTurn.setSetpoint(swerveModuleState.angle.getRadians());
		pidSpeed.setSetpoint(swerveModuleState.speedMetersPerSecond);
	}

	public void periodic() {
		// Run motors with PID
		double turnOutput = pidTurn.calculate(encoder.getAbsolutePosition());
		double speedOutput = pidSpeed.calculate(
				drive_motor.getEncoder().getVelocity() / L2_DRIVE_RATIO * SWERVE_WHEEL_RAD * 2 * PI);
		turn_motor.set(turnOutput);
		drive_motor.set(speedOutput);
	}
}
