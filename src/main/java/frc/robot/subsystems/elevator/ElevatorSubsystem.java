package frc.robot.subsystems.elevator;

import static frc.robot.constants.Constants.ELEVATOR_MOTOR_LEFT;
import static frc.robot.constants.Constants.ELEVATOR_MOTOR_RIGHT;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

	public TalonFX climberMotor1 = new TalonFX(ELEVATOR_MOTOR_RIGHT);
	public TalonFX climberMotor2 = new TalonFX(ELEVATOR_MOTOR_LEFT);
	public PIDController climber1PID = new PIDController(0, 0, 0);
	public PIDController climber2PID = new PIDController(0, 0, 0);

	public ElevatorSubsystem() {}

	public void setMotor(double speed) {
		climberMotor1.set(climber1PID.calculate(speed));
		climberMotor2.set(climber2PID.calculate(speed));
	}
}
