package frc.robot.subsystems.drive;

import static frc.robot.constants.Constants.*;
import static java.lang.Math.*;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.SwerveModuleConfiguration;

/**
 * Module IO implementation for SparkMax drive motor controller, SparkMax turn motor controller (NEO
 * or NEO 550), and analog absolute encoder connected to the RIO
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using a CANcoder, copy from "ModuleIOTalonFX")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOSparkMax implements ModuleIO {
	// Gear ratios for SDS MK4i L2, adjust as necessary
	private static final double DRIVE_GEAR_RATIO = 1 / L2_DRIVE_RATIO;
	private static final double TURN_GEAR_RATIO = 1 / L2_TURN_RATIO;

	private final CANSparkMax driveSparkMax;
	private final CANSparkMax turnSparkMax;

	private final SparkPIDController drivePID;
	private final SparkPIDController turnPID;

	private final RelativeEncoder driveEncoder;
	private final RelativeEncoder turnRelativeEncoder;
	private final CANcoder turnAbsoluteEncoder;

	private final boolean isTurnMotorInverted = true;
	private final Rotation2d absoluteEncoderOffset;

	public ModuleIOSparkMax(SwerveModuleConfiguration config) {
		driveSparkMax = new CANSparkMax(config.DRIVE_MOTOR, MotorType.kBrushless);
		turnSparkMax = new CANSparkMax(config.TURN_MOTOR, MotorType.kBrushless);
		turnAbsoluteEncoder = new CANcoder(config.ENCODER);
		absoluteEncoderOffset = config.offset; // MUST BE CALIBRATED
		driveSparkMax.restoreFactoryDefaults();
		turnSparkMax.restoreFactoryDefaults();

		driveSparkMax.setCANTimeout(250);
		turnSparkMax.setCANTimeout(250);

		driveEncoder = driveSparkMax.getEncoder();
		driveEncoder.setVelocityConversionFactor(
				Units.rotationsPerMinuteToRadiansPerSecond(1) / DRIVE_GEAR_RATIO * SWERVE_WHEEL_RAD);
		driveEncoder.setPosition(0.0);
		// System.out.println(driveEncoder.getVelocityConversionFactor());

		turnRelativeEncoder = turnSparkMax.getEncoder();

		turnSparkMax.setInverted(isTurnMotorInverted);
		driveSparkMax.setSmartCurrentLimit(40);
		turnSparkMax.setSmartCurrentLimit(30);
		driveSparkMax.enableVoltageCompensation(12.0);
		turnSparkMax.enableVoltageCompensation(12.0);

		turnRelativeEncoder.setPosition(0.0);
		turnRelativeEncoder.setMeasurementPeriod(10);
		turnRelativeEncoder.setAverageDepth(2);

		driveSparkMax.setCANTimeout(0);
		turnSparkMax.setCANTimeout(0);
		drivePID = driveSparkMax.getPIDController();
		// drivePID.setI(0.001);
		drivePID.setP(DRIVE_KP);
		drivePID.setFF(DRIVE_KF);
		// drivePID.setOutputRange(-1, 1);

		turnPID = turnSparkMax.getPIDController();

		driveSparkMax.burnFlash();
		turnSparkMax.burnFlash();
	}

	@Override
	public void updateInputs(ModuleIOInputs inputs) {
		inputs.drivePositionRad = driveEncoder.getPosition() * 2 * PI / DRIVE_GEAR_RATIO;
		// inputs.driveVelocityRadPerSec =
		// 		Units.rotationsPerMinuteToRadiansPerSecond(driveEncoder.getVelocity()) / DRIVE_GEAR_RATIO;
		inputs.driveVelocityMetersPerSec = driveEncoder.getVelocity();
		inputs.driveAppliedVolts = driveSparkMax.getAppliedOutput() * driveSparkMax.getBusVoltage();
		inputs.driveCurrentAmps = new double[] {driveSparkMax.getOutputCurrent()};

		inputs.turnAbsolutePosition = new Rotation2d(Units.rotationsToRadians(
						turnAbsoluteEncoder.getAbsolutePosition().getValueAsDouble()))
				.minus(absoluteEncoderOffset);
		inputs.turnAbsolutePositionRad = inputs.turnAbsolutePosition.getRadians();
		inputs.turnPosition = Rotation2d.fromRotations(turnRelativeEncoder.getPosition() / TURN_GEAR_RATIO);
		inputs.turnVelocityRadPerSec =
				Units.rotationsPerMinuteToRadiansPerSecond(turnRelativeEncoder.getVelocity()) / TURN_GEAR_RATIO;
		inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
		inputs.turnCurrentAmps = new double[] {turnSparkMax.getOutputCurrent()};
	}

	@Override
	public void setDriveVoltage(double volts) {
		driveSparkMax.setVoltage(volts);
	}

	@Override
	public void setDriveVelocity(double mps) {
		drivePID.setReference(mps, ControlType.kVelocity);
	}

	@Override
	public void setTurnVoltage(double volts) {
		turnSparkMax.setVoltage(volts);
	}

	@Override
	public void setDriveBrakeMode(boolean enable) {
		driveSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
	}

	@Override
	public void setTurnBrakeMode(boolean enable) {
		turnSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
	}

	@Override
	public void logTargetState(ModuleIOInputs inputs, SwerveModuleState state, double compensatedVel) {
		inputs.targetRad = state.angle.getRadians();
		inputs.targetVel = state.speedMetersPerSecond;
		inputs.compensatedTargetVel = compensatedVel;
	}
}
