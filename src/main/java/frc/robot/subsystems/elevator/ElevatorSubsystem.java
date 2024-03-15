package frc.robot.subsystems.elevator;

import static frc.robot.constants.Constants.ELEVATOR_MOTOR_LEFT;
import static frc.robot.constants.Constants.ELEVATOR_MOTOR_RIGHT;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends SubsystemBase {

	private static final int CLIMBER_MIN_HEIGHT = 0;
	private static final int CLIMBER_MAX_HEIGHT = 70;
	public TalonFX climberMotor1 = new TalonFX(ELEVATOR_MOTOR_LEFT);
	public TalonFX climberMotor2 = new TalonFX(ELEVATOR_MOTOR_RIGHT);
	public PIDController climber1PID = new PIDController(1, 0, 0);
	public PIDController climber2PID = new PIDController(1, 0, 0);

	public ElevatorSubsystem() {
		climberMotor1.setNeutralMode(NeutralModeValue.Brake);
		climberMotor2.setNeutralMode(NeutralModeValue.Brake);
		climberMotor1.setPosition(0);
		climberMotor2.setPosition(0);
	}

	@Override
	public void periodic() {
		Logger.recordOutput("Elevator/Left/Pos", climber1PID.getSetpoint());
		Logger.recordOutput("Elevator/Right/Pos", climber2PID.getSetpoint());

		climberMotor1.setVoltage(
				climber1PID.calculate(climberMotor1.getPosition().getValueAsDouble()));
		climberMotor2.setVoltage(
				climber2PID.calculate(climberMotor2.getPosition().getValueAsDouble()));
	}

	public void setLeft(double speed) {
		climberMotor1.set(speed);
	}

	public void setRightPos(double pos) {
		climber2PID.setSetpoint(MathUtil.clamp(pos, CLIMBER_MIN_HEIGHT, CLIMBER_MAX_HEIGHT));
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
