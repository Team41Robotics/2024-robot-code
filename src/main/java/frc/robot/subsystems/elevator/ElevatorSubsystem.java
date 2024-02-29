package frc.robot.subsystems.elevator;

import static frc.robot.constants.Constants.ELEVATOR_MOTOR_LEFT;
import static frc.robot.constants.Constants.ELEVATOR_MOTOR_RIGHT;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

	public TalonFX climberMotor1 = new TalonFX(ELEVATOR_MOTOR_RIGHT);
	public TalonFX climberMotor2 = new TalonFX(ELEVATOR_MOTOR_LEFT);
	public PIDController climber1PID = new PIDController(0, 0, 0);
	public PIDController climber2PID = new PIDController(0, 0, 0);

	public ElevatorSubsystem() {
		climberMotor1.setNeutralMode(NeutralModeValue.Brake);
		climberMotor2.setNeutralMode(NeutralModeValue.Brake);
	}

	public void setLeft(double speed) {
		climberMotor1.set(speed);
	}

	public void setRight(double speed) {
		climberMotor2.set(speed);
	}
}
