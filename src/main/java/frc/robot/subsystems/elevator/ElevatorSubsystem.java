package frc.robot.subsystems.elevator;

import static frc.robot.constants.Constants.ELEVATOR_MOTOR_LEFT;
import static frc.robot.constants.Constants.ELEVATOR_MOTOR_RIGHT;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/**
 * The ElevatorSubsystem class represents the elevator subsystem of the robot.
 * It controls the movement and position of the elevator using PID controllers.
 */
public class ElevatorSubsystem extends SubsystemBase {

	private static final int CLIMBER_MIN_HEIGHT = -70;
	private static final int CLIMBER_MAX_HEIGHT = 40;
	public TalonFX climberMotor1 = new TalonFX(ELEVATOR_MOTOR_LEFT);
	public TalonFX climberMotor2 = new TalonFX(ELEVATOR_MOTOR_RIGHT);
	public PIDController climber1PID = new PIDController(1, 0, 0);
	public PIDController climber2PID = new PIDController(1, 0, 0);

	public ElevatorSubsystem() {
		climberMotor1.setNeutralMode(NeutralModeValue.Brake);
		climberMotor2.setNeutralMode(NeutralModeValue.Brake);
		climberMotor1.setPosition(0);
		climberMotor2.setPosition(0);
		climberMotor2.setInverted(false);
	}

	/**
	 * Resets the encoders and PID setpoints of the elevator subsystem to zero.
	 *
	 * @return The command to reset the encoders and setpoints.
	 */
	public Command zeroEncoders() {
		return this.runOnce(() -> {
			climberMotor1.setPosition(0);
			climber1PID.setSetpoint(0);
			climberMotor2.setPosition(0);
			climber2PID.setSetpoint(0);
		});
	}

	/**
	 * This method is called periodically to update the elevator subsystem.
	 * It records various outputs using the Logger class and calculates the voltage
	 * to be applied to the elevator motors using PID controllers.
	 */
	@Override
	public void periodic() {
		Logger.recordOutput("Elevator/Left/PID/Setpoint", climber1PID.getSetpoint());
		Logger.recordOutput("Elevator/Right/PID/Setpoint", climber2PID.getSetpoint());

		Logger.recordOutput("Elevator/Left/PID/err", climber1PID.getPositionError());
		Logger.recordOutput("Elevator/Right/PID/err", climber2PID.getPositionError());

		Logger.recordOutput("Elevator/Left/Pos", climberMotor1.getPosition().getValueAsDouble());
		Logger.recordOutput("Elevator/Right/Pos", climberMotor2.getPosition().getValueAsDouble());

		Logger.recordOutput(
				"Elevator/Left/curr", climberMotor1.getTorqueCurrent().getValueAsDouble());
		Logger.recordOutput(
				"Elevator/Right/curr", climberMotor2.getTorqueCurrent().getValueAsDouble());

		climberMotor1.setVoltage(
				climber1PID.calculate(climberMotor1.getPosition().getValueAsDouble()));
		climberMotor2.setVoltage(
				climber2PID.calculate(climberMotor2.getPosition().getValueAsDouble()));
	}

	/**
	 * Sets the speed of the left climber motor.
	 *
	 * @param speed the speed to set for the left climber motor
	 */
	public void setLeft(double speed) {
		climberMotor1.set(speed);
	}

	/**
	 * Sets the speed of the right climber motor.
	 *
	 * @param speed the speed to set the right climber motor to
	 */
	public void setRight(double speed) {
		climberMotor2.set(speed);
	}

	/**
	 * Sets the position of the right climber to the specified value.
	 * The position is clamped between the maximum and minimum heights of the climber.
	 *
	 * @param pos The desired position of the right climber.
	 */
	public void setRightPos(double pos) {
		climber2PID.setSetpoint(MathUtil.clamp(pos, -CLIMBER_MAX_HEIGHT, -CLIMBER_MIN_HEIGHT));
	}

	/**
	 * Sets the position of the left climber to the specified value.
	 * The position is clamped between the minimum and maximum height of the climber.
	 *
	 * @param pos the desired position of the left climber
	 */
	public void setLeftPos(double pos) {
		climber1PID.setSetpoint(MathUtil.clamp(pos, CLIMBER_MIN_HEIGHT, CLIMBER_MAX_HEIGHT));
	}

	/**
	 * Returns the target position of the left climber in the elevator subsystem.
	 *
	 * @return The target position of the left climber.
	 */
	public double getLeftPos() {
		return climber1PID.getSetpoint();
	}

	/**
	 * Returns the target position of the right climber in the elevator subsystem.
	 *
	 * @return The target position of the right climber.
	 */
	public double getRightPos() {
		return climber2PID.getSetpoint();
	}
}
