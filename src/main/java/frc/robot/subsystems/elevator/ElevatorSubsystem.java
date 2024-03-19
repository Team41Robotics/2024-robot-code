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

public class ElevatorSubsystem extends SubsystemBase {

	private static final int CLIMBER_MIN_HEIGHT = -50;
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

	public Command zeroEncoders() {
		return this.runOnce(() -> {
			climberMotor1.setPosition(0);
			climber1PID.setSetpoint(0);
			climberMotor2.setPosition(0);
			climber2PID.setSetpoint(0);
		});
	}

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

	public void setLeft(double speed) {
		climberMotor1.set(speed);
	}

	public void setRightPos(double pos) {
		climber2PID.setSetpoint(MathUtil.clamp(pos, -CLIMBER_MAX_HEIGHT, -CLIMBER_MIN_HEIGHT));
	}

	public void setLeftPos(double pos) {
		climber1PID.setSetpoint(MathUtil.clamp(pos, CLIMBER_MIN_HEIGHT, CLIMBER_MAX_HEIGHT));
	}

	public double getLeftPos() {
		return climber1PID.getSetpoint();
	}

	public double getRightPos() {
		return climber2PID.getSetpoint();
	}

	public void setRight(double speed) {
		climberMotor2.set(speed);
	}
}
