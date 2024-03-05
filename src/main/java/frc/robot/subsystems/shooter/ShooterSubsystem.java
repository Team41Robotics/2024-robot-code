package frc.robot.subsystems.shooter;

import static frc.robot.RobotContainer.drive;
import static frc.robot.constants.Constants.*;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.Util;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {
	public final CANSparkMax angleMotor = new CANSparkMax(SHOOTER_PIVOT_MOTOR1, MotorType.kBrushless);
	public final CANSparkMax angleMotor2 = new CANSparkMax(SHOOTER_PIVOT_MOTOR2, MotorType.kBrushless);

	public final CANSparkFlex sm_top = new CANSparkFlex(SHOOTER_MOTOR_TOP, MotorType.kBrushless);
	public final CANSparkFlex sm_bot = new CANSparkFlex(SHOOTER_MOTOR_BOT, MotorType.kBrushless);
	private double sign = 1;
	private final RelativeEncoder en_top = sm_top.getEncoder();
	private final RelativeEncoder en_bot = sm_bot.getEncoder();
	LinearFilter top_curr = LinearFilter.movingAverage(5);
	LinearFilter bot_curr = LinearFilter.movingAverage(5);

	private final BangBangController bang_top = new BangBangController(200);
	private final BangBangController bang_bot = new BangBangController(200);

	public final CANSparkMax feeder = new CANSparkMax(FEEDER_MOTOR, MotorType.kBrushless);
	private static double BEAM_BREAK_THRESHOLD = 0.2;
	DutyCycleEncoder angleEncoder = new DutyCycleEncoder(SHOOTER_ENCODER);
	AnalogInput ringSensorAnalogInput = new AnalogInput(RING_SENSOR);
	Trigger ringSensor = new Trigger(() -> (ringSensorAnalogInput.getVoltage() <= BEAM_BREAK_THRESHOLD));
	public PIDController angle_pid = new PIDController(6e-3, 0, 0);

	private Optional<Rotation2d> target_angle = Optional.empty();

	public double top_speed = 0;
	public double bot_speed = 0;

	public ShooterSubsystem() {

		angleMotor2.follow(angleMotor, true);
		angleMotor.setIdleMode(IdleMode.kBrake);
		angleMotor2.setIdleMode(IdleMode.kBrake);
		sm_top.setIdleMode(IdleMode.kCoast);
		sm_bot.setIdleMode(IdleMode.kCoast);
		sm_top.setInverted(true);
		angle_pid.setTolerance(5);
		angleEncoder.setPositionOffset(SHOOTER_ENCODER_OFFSET);

		bang_bot.setSetpoint(0);
		bang_top.setSetpoint(0);
	}

	/**
	 * returns the angle of the shooter.
	 * @returns angle: from 90 straight forward to 0 straight up
	 */
	public Rotation2d getAngle() {
		double angle = angleEncoder.getAbsolutePosition();
		if (angle < 0.3) angle += 1;
		angle -= angleEncoder.getPositionOffset();
		return Rotation2d.fromRotations(angle);
	}

	public void init() {}

	private void runPivot() {
		if (target_angle.isEmpty()) return;

		double angle = getAngle().getDegrees();
		double target_angle = this.target_angle.get().getDegrees();

		double output = angle_pid.calculate(angle, target_angle);
		Logger.recordOutput("Shooter/PID/heartbeat", Timer.getFPGATimestamp());
		Logger.recordOutput("Shooter/PID/Angle/Output", output);
		Logger.recordOutput("Shooter/PID/Angle/Current", angle);
		Logger.recordOutput("Shooter/PID/Angle/Target", target_angle);

		Logger.recordOutput("Shooter/PID/Angle/Error", angle_pid.getPositionError());

		angleMotor.set(MathUtil.clamp(output, -0.1, 0.1));
	}

	/**
	 * returns true if the angle PID is within the tolerance range for position
	 * @return whether angle error is within acceptable bounds
	 */
	public boolean angleAtSetpoint() {
		return angle_pid.atSetpoint();
	}

	private void runShooters() {
		Logger.recordOutput("bangSetpoint", bang_bot.getSetpoint() * sign);
		Logger.recordOutput("bangOutput", 10 * sign * bang_top.calculate(sign * en_top.getVelocity()));
		sm_top.setVoltage(10 * sign * bang_top.calculate(sign * en_top.getVelocity()));
		sm_bot.setVoltage(10 * sign * bang_bot.calculate(sign * en_bot.getVelocity()));
	}

	@Override
	public void periodic() {
		runShooters();
		runPivot();

		Logger.recordOutput("Shooter/IsReady", isReady());
		Logger.recordOutput("Shooter/Angle", angleEncoder.getAbsolutePosition());
		Logger.recordOutput("Shooter/CorrectedAngle", getAngle());

		Logger.recordOutput("Shooter/Motors/TopMotor/Speed", sm_top.getEncoder().getVelocity());
		Logger.recordOutput("Shooter/Motors/BotMotor/Speed", sm_bot.getEncoder().getVelocity());

		Logger.recordOutput("Shooter/Motors/TopMotor/Current", top_curr.calculate(sm_top.getOutputCurrent()));
		Logger.recordOutput("Shooter/Motors/BotMotor/Current", bot_curr.calculate(sm_bot.getOutputCurrent()));

		Logger.recordOutput("Shooter/Pivot1Position", angleMotor.getEncoder().getPosition());
		Logger.recordOutput("Shooter/Pivot2Position", angleMotor2.getEncoder().getPosition());
		Logger.recordOutput("Shooter/BeamBreakOutput", ringSensorAnalogInput.getVoltage());
		Logger.recordOutput("Shooter/Ring Present", ringLoaded());
		Logger.recordOutput("Shooter/AtSetpoint", angleAtSetpoint());
	}

	public double calculateAngle() {
		double targetY = TARGET_Y;
		double targetX = Util.isRed() ? TARGET_X_RED : TARGET_X_BLUE;
		double dx = drive.getPose().getX() - targetX;
		double dy = drive.getPose().getY() - targetY;
		double distance = Math.hypot(dx, dy);

		double y = TARGET_HEIGHT - SHOOTER_HEIGHT;
		double flight_time = distance / NOTE_VELOCITY;
		y += 9.8 / 2 * flight_time * flight_time;
		Logger.recordOutput("Angle", Units.radiansToDegrees(Math.atan(y / distance)));
		Logger.recordOutput("Distance", distance);

		return 90 - Units.radiansToDegrees(Math.atan(y / distance));
	}

	public void zeroAbsoluteEncoder() {
		System.out.println("curr pos: " + angleEncoder.getAbsolutePosition() + " curr offset: "
				+ angleEncoder.getPositionOffset() + " pos no offset: "
				+ (angleEncoder.getAbsolutePosition() - angleEncoder.getPositionOffset()));
		angleEncoder.setPositionOffset(angleEncoder.getAbsolutePosition());
	}

	public void setAngle(Rotation2d angle) { // someone is kinda stpuid
		// TODO
		if (angle.getDegrees() < 0 || angle.getDegrees() > 90) {
			this.target_angle = Optional.of(Rotation2d.fromDegrees(15));
			System.out.println("ERROR: Out of bounds angle");
			return;
		}
		this.target_angle = Optional.of(angle);
	}

	public void runFeederMotor(double percent) {
		feeder.set(percent);
	}

	public boolean ringLoaded() {
		return ringSensor.getAsBoolean();
	}

	public void runMotors(double speed) {
		sign = Math.signum(speed);
		bang_bot.setSetpoint(Math.abs(speed) * 6000);
		bang_top.setSetpoint(Math.abs(speed) * 0.85 * 6000);
	}

	public boolean isReady() {
		return Math.abs(bang_bot.getSetpoint() - en_bot.getVelocity()) < 500;
	}

	// Commands
	public Command runFeeder() {
		return this.startEnd(() -> runFeederMotor(0.3), () -> runFeederMotor(0)).until(ringSensor);
	}

	public Command autoShoot() {
		return shootSingle(0.7)
				.deadlineWith(new RunCommand(() -> setAngle(Rotation2d.fromDegrees(calculateAngle()))))
				.andThen(new InstantCommand(() -> runMotors(0.4)));
	}

	public Command shootSingle(double speed) {
		return new StartEndCommand(() -> runMotors(speed), () -> runMotors(0)).until(ringSensor.negate());
	}

	public Command toAngleCommand(Supplier<Rotation2d> speed) {
		return new InstantCommand(() -> setAngle(speed.get()));
	}

	public Command toAngleCommand(Rotation2d speed) {
		return this.toAngleCommand(() -> speed);
	}

	public Command toAngleDegreeCommand(double speed) {
		return this.toAngleCommand(() -> Rotation2d.fromDegrees(speed));
	}

	public Command muzzleLoad() {
		return new SequentialCommandGroup(
				new InstantCommand(() -> runMotors(-0.2)),
				new WaitUntilCommand(ringSensor),
				new InstantCommand(() -> runFeederMotor(-0.2)),
				new WaitUntilCommand(ringSensor.negate()),
				this.runFeeder(),
				new InstantCommand(() -> runMotors(0)));
	}
}
