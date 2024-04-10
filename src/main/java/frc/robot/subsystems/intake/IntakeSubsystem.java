package frc.robot.subsystems.intake;

import static frc.robot.constants.Constants.INTAKE_FEEDER_MOTOR;
import static frc.robot.constants.Constants.INTAKE_PIVOT_MOTOR;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {

	DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(1);

	Translation3d intake_pos = new Translation3d(0, Units.inchesToMeters(12.23), Units.inchesToMeters(6.878));
	CANSparkMax pivotMotor = new CANSparkMax(INTAKE_PIVOT_MOTOR, MotorType.kBrushless);
	CANSparkMax turnMotor = new CANSparkMax(INTAKE_FEEDER_MOTOR, MotorType.kBrushless);
	public double kg = 0; // 0.25;
	public ProfiledPIDController pivotPID =
			new ProfiledPIDController(30, 0, 0.1, new TrapezoidProfile.Constraints(40, 40));
	public PIDController turnPID = new PIDController(0, 0, 0);

	private DigitalInput limitSwitch = new DigitalInput(2);
	Optional<Rotation2d> target_angle = Optional.empty();

	public IntakeSubsystem() {
		turnMotor.restoreFactoryDefaults();
		pivotMotor.setIdleMode(IdleMode.kCoast);
		turnMotor.setIdleMode(IdleMode.kCoast);
		pivotPID.setTolerance(10, 1);
		pivotEncoder.setPositionOffset(pivotEncoder.getAbsolutePosition());
		turnMotor.setSmartCurrentLimit(30);
	}

	public void runIntakeMotor(double speed) {
		turnMotor.set(speed);
	}

	public void stopIntakeMotor() {
		turnMotor.set(0);
	}

	public boolean intakeSwitch() {
		return limitSwitch.get();
	}

	/**
	 * Checks if the intake angle is at the setpoint.
	 *
	 * @return true if the intake angle is at the setpoint, false otherwise.
	 */
	public boolean angleAtSetpoint() {
		if (this.target_angle.isEmpty()) return false;
		if (Math.abs(pivotPID.getGoal().position - pivotPID.getSetpoint().position) > 1) return false;
		return Math.abs(this.target_angle.get().getDegrees() - getAngle().getDegrees()) < 10;
	}
	/**
	 * Returns the current angle of the intake, linearized and zeroed
	 * <p>
	 * 0 Degrees is straight up, -90 is directly inwards, and +90 is directly forwards
	 *
	 * @return angle of the intake
	 */
	public Rotation2d getAngle() {
		double angle = pivotEncoder.getAbsolutePosition();
		if (angle > 0.6) angle -= 1;
		return Rotation2d.fromRotations(angle).minus(Rotation2d.fromDegrees(75));
	}

	/**
	 * Sets the angle of the intake subsystem to the specified target angle, and resets the I gain to prevent weird things from happening
	 *
	 * @param target the target angle to set the intake subsystem to
	 */
	public void setAngle(Rotation2d target) {
		this.target_angle = Optional.of(target);
		pivotPID.reset(getAngle().getRadians());
		pivotPID.setGoal(target.getRadians());
	}

	/**
	 * Runs the pivot mechanism of the intake subsystem.
	 * If the target angle is empty, the method returns without performing any action.
	 * Calculates the output using the pivotPID controller based on the current angle.
	 * Adjusts the output by subtracting the sine of the angle multiplied by the kg constant,
	 * and clamps the adjusted output between -4 and 4.
	 * Sets the voltage of the pivot motor to the adjusted output.
	 * Records various outputs using the Logger class for debugging purposes.
	 */
	public void runPivot() {
		if (this.target_angle.isEmpty()) return;
		double out = pivotPID.calculate(getAngle().getRadians());

		double adjusted_out = MathUtil.clamp(out - getAngle().getSin() * this.kg, -4, 4);
		pivotMotor.setVoltage(adjusted_out); // MathUtil.clamp(out, -3, 3));

		Logger.recordOutput("Pivot/PID/output", out);
		Logger.recordOutput(
				"Pivot/rawEror",
				Math.abs(this.target_angle.get().getDegrees() - getAngle().getDegrees()));
		Logger.recordOutput("Pivot/PID/err", pivotPID.getPositionError());
		Logger.recordOutput("Pivot/AtGoal", angleAtSetpoint());
		Logger.recordOutput("Pivot/PID/goal", pivotPID.getGoal().position);
		Logger.recordOutput("Pivot/PID/setpoint", pivotPID.getSetpoint().position);
		Logger.recordOutput("Pivot/Adjusted Out", adjusted_out);
		Logger.recordOutput("Intake/LimitSwitch", limitSwitch.get());
	}

	/**
	 * Executes periodic tasks for the intake subsystem.
	 * This method is called repeatedly in a loop to update the state of the intake subsystem.
	 */
	public void periodic() {

		runPivot();

		Logger.recordOutput(
				"3DPoses/intake",
				new Pose3d(intake_pos, new Rotation3d(getAngle().getRadians(), 0, 0)));
		Logger.recordOutput("Pivot/RawAngle", pivotEncoder.getAbsolutePosition());
		Logger.recordOutput("Pivot/Angle", getAngle().getDegrees());
		Logger.recordOutput("Pivot/kg", kg);
		Logger.recordOutput("Intake/Curr", turnMotor.getOutputCurrent());
		Logger.recordOutput("Intake/Vel", turnMotor.getEncoder().getVelocity());
	}

	/**
	 * Returns a new startEnd command that runs the intake at a specified speed and stops it when the command ends
	 *
	 * @param speed the desired speed to run the intake at, -1 to 1
	 *
	 * @return the runIntake command
	 */
	public Command runIntake(double speed) {
		return new StartEndCommand(() -> this.runIntakeMotor(speed), this::stopIntakeMotor);
	}

	public Command toDegree(double degrees) {
		return this.runOnce(() -> this.target_angle = Optional.of(Rotation2d.fromDegrees(degrees)));
	}

	/**
	 * Returns a Command object that performs automatic intake.
	 * The intake will pivot all the way down to 120 degree, and then run the intake motor until the limit switch is pressed.
	 *
	 * @return the Command object for automatic intake
	 */
	public Command automaticIntake() {
		return toDegree(120)
				.andThen(new WaitUntilCommand(() -> getAngle().getDegrees() > 0))
				.andThen(runIntake(0.75).until(() -> !intakeSwitch()));
	}
}
