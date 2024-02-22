package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {

	DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(new DigitalOutput(1));

	CANSparkMax pivotMotor = new CANSparkMax(2, MotorType.kBrushless);
	CANSparkMax turnMotor = new CANSparkMax(14, MotorType.kBrushless);

	public PIDController pivotPID = new PIDController(0, 0, 0);
	public PIDController turnPID = new PIDController(0, 0, 0);

	private Optional<Rotation2d> target_angle = Optional.empty();

	private ArrayList<Double> intakeCurrent = new ArrayList<Double>();
	private double meanCurrentVoltage = 0;

	// jank
	public double calculateMeanCurrentVoltage() {
		double sum = 0;
		for (int i = 0; i < intakeCurrent.size(); i++) {
			sum += intakeCurrent.get(i);
		}
		return sum / intakeCurrent.size();
	}

	public IntakeSubsystem() {
		pivotMotor.setIdleMode(IdleMode.kBrake);
		turnMotor.setIdleMode(IdleMode.kCoast);
		pivotEncoder.setPositionOffset(pivotEncoder.getAbsolutePosition());
	}

	public void runIntakeMotor() {
		turnMotor.set(.5);
	}

	public void stopIntakeMotor() {
		turnMotor.set(0);
	}

	public boolean angleAtSetpoint() {
		return pivotPID.atSetpoint();
	}

	public Rotation2d getAngle() {
		double angle = pivotEncoder.getAbsolutePosition();
		if (angle < 0.3) angle += 1;
		angle -= pivotEncoder.getPositionOffset();
		return Rotation2d.fromRotations(angle);
	}

	public void setAngle(Rotation2d target) {
		this.target_angle = Optional.of(target);
	}

	public void runPivot() {
		if (this.target_angle.isEmpty()) {
			return;
		}
		if ((this.target_angle.get().getDegrees() - this.getAngle().getDegrees()) > 1) {
			pivotMotor.set(pivotPID.calculate(
					this.target_angle.get().getDegrees(), this.getAngle().getDegrees()));
		}

		double currentOutputVoltage = pivotMotor.getAppliedOutput();
		intakeCurrent.add(currentOutputVoltage);
		this.meanCurrentVoltage = calculateMeanCurrentVoltage();
		Logger.recordOutput("Intake/Current", currentOutputVoltage);
	}
	// Angles in degrees right now, may change in the future
	public void periodic() {}
}
