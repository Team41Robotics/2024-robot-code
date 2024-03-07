package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {
	DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(1);

	CANSparkMax pivotMotor = new CANSparkMax(14, MotorType.kBrushless);
	CANSparkMax turnMotor = new CANSparkMax(2, MotorType.kBrushless);
	public double kg = 0; // 0.25;
	public ProfiledPIDController pivotPID =
			new ProfiledPIDController(9, 0, 0.3, new TrapezoidProfile.Constraints(9, 11));
	public PIDController turnPID = new PIDController(0, 0, 0);

	private DigitalInput limitSwitch = new DigitalInput(2);
	Optional<Rotation2d> target_angle = Optional.empty();

	public IntakeSubsystem() {
		pivotMotor.setIdleMode(IdleMode.kCoast);
		turnMotor.setIdleMode(IdleMode.kCoast);
		pivotPID.setTolerance(10, 1);
		pivotEncoder.setPositionOffset(pivotEncoder.getAbsolutePosition());
	}

	public void runIntakeMotor(double speed) {
		turnMotor.set(speed);
	}

	public void stopIntakeMotor() {
		turnMotor.set(0);
	}

	public boolean angleAtSetpoint() {
		if (this.target_angle.isEmpty()) return false;
		if (Math.abs(pivotPID.getGoal().position - pivotPID.getSetpoint().position) > 1) return false;
		return Math.abs(this.target_angle.get().getDegrees() - getAngle().getDegrees()) < 10;
	}

	public Rotation2d getAngle() {
		double angle = pivotEncoder.getAbsolutePosition();
		if (angle > 0.6) angle -= 1;
		// angle -= pivotEncoder.getPositionOffset();
		return Rotation2d.fromRotations(angle).minus(Rotation2d.fromDegrees(70));
		// return Rotation2d.fromRotations(pivotEncoder.getAbsolutePosition()).minus(Rotation2d.fromDegrees(70));
		// negative is up
	}

	public boolean intakeSwitch() {
		return limitSwitch.get();
	}

	public void setAngle(Rotation2d target) {
		this.target_angle = Optional.of(target);
		// pivotPID.reset();
		pivotPID.reset(getAngle().getRadians());
		pivotPID.setGoal(target.getRadians());
	}

	public void runPivot() {
		if (this.target_angle.isEmpty()) return;
		double out = pivotPID.calculate(getAngle().getRadians());
		Logger.recordOutput("Pivot/PID/output", out);
		Logger.recordOutput(
				"Pivot/rawEror",
				Math.abs(this.target_angle.get().getDegrees() - getAngle().getDegrees()));
		Logger.recordOutput("Pivot/PID/err", pivotPID.getPositionError());
		Logger.recordOutput("Pivot/AtGoal", angleAtSetpoint());
		double adjusted_out = MathUtil.clamp(out - getAngle().getSin() * this.kg, -4, 4);
		Logger.recordOutput("Pivot/Adjusted Out", adjusted_out);
		Logger.recordOutput("Intake/LimitSwitch", limitSwitch.get());
		pivotMotor.setVoltage(adjusted_out); // MathUtil.clamp(out, -3, 3));
	}

	public void periodic() {

		// System.out.println(getAngle().getDegrees() + "\t" + out);
		runPivot();
		Logger.recordOutput("Pivot/RawAngle", pivotEncoder.getAbsolutePosition());
		Logger.recordOutput("Pivot/Angle", getAngle().getDegrees());
		Logger.recordOutput("Pivot/kg", kg);
	}

	public Command runIntake(double speed) {
		return new StartEndCommand(() -> this.runIntakeMotor(speed), this::stopIntakeMotor);
	}
}
