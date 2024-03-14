package frc.robot.subsystems.shooter;

import static frc.robot.RobotContainer.drive;
import static frc.robot.RobotContainer.ds;
import static frc.robot.constants.Constants.FEEDER_MOTOR;
import static frc.robot.constants.Constants.MIDDLE_BEAM_BREAK_PORT;
import static frc.robot.constants.Constants.NOTE_VELOCITY;
import static frc.robot.constants.Constants.RING_SENSOR;
import static frc.robot.constants.Constants.SHOOTER_ENCODER;
import static frc.robot.constants.Constants.SHOOTER_ENCODER_OFFSET;
import static frc.robot.constants.Constants.SHOOTER_HEIGHT;
import static frc.robot.constants.Constants.SHOOTER_MOTOR_BOT;
import static frc.robot.constants.Constants.SHOOTER_MOTOR_TOP;
import static frc.robot.constants.Constants.SHOOTER_PIVOT_MOTOR1;
import static frc.robot.constants.Constants.SHOOTER_PIVOT_MOTOR2;
import static frc.robot.constants.Constants.TARGET_HEIGHT;
import static frc.robot.constants.Constants.TARGET_X_BLUE;
import static frc.robot.constants.Constants.TARGET_X_RED;
import static frc.robot.constants.Constants.TARGET_Y;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
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
import frc.robot.constants.Constants;
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

	// private final BangBangController bang_top = new BangBangController(200);
	// private final BangBangController bang_bot = new BangBangController(200);
	PIDController pid_top = new PIDController(6 / 6000., 0 / 12000., 0);
	PIDController pid_bot = new PIDController(6 / 6000., 0 / 12000., 0);

	private final DigitalInput middleBeamBreak = new DigitalInput(MIDDLE_BEAM_BREAK_PORT);
	public final CANSparkMax feeder = new CANSparkMax(FEEDER_MOTOR, MotorType.kBrushless);
	private static double BEAM_BREAK_THRESHOLD = 0.2;
	DutyCycleEncoder angleEncoder = new DutyCycleEncoder(SHOOTER_ENCODER);
	AnalogInput ringSensorAnalogInput = new AnalogInput(RING_SENSOR);
	Trigger ringSensor = new Trigger(() -> (ringSensorAnalogInput.getVoltage() <= BEAM_BREAK_THRESHOLD));
	public PIDController angle_pid = new PIDController(1e-2, 0, 0);

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
		target_angle = Optional.of(Rotation2d.fromDegrees(15));
		pid_bot.setSetpoint(0);
		pid_top.setSetpoint(0);
		pid_bot.setIntegratorRange(-3, 3);
		pid_top.setIntegratorRange(-3, 3);
	}

	/**
	 * returns the angle of the shooter.
	 * @returns angle: from 90 straight forward to 0 straight up
	 */
	public Rotation2d getAngle() {
		double angle = angleEncoder.getAbsolutePosition();
		// if (angle < 0.3) angle += 1;
		angle -= angleEncoder.getPositionOffset();
		return Rotation2d.fromRotations(-angle);
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

		angleMotor.set(MathUtil.clamp(output, -0.25, 0.25));
	}

	/**
	 * returns true if the angle PID is within the tolerance range for position
	 * @return whether angle error is within acceptable bounds
	 */
	public boolean angleAtSetpoint() {
		return angle_pid.atSetpoint();
	}

	private void runShooters() {
		Logger.recordOutput("Shooter/Setpoint", pid_top.getSetpoint());
		Logger.recordOutput("Shooter/Output", pid_top.calculate(en_top.getVelocity()));
		if (pid_bot.getSetpoint() == 0) {
			sm_bot.setVoltage(0);
			sm_top.setVoltage(0);
			return;
		}
		sm_top.setVoltage(pid_top.calculate(en_top.getVelocity()) + 12 * pid_top.getSetpoint() / 6000);
		sm_bot.setVoltage(pid_bot.calculate(en_bot.getVelocity()) + 12 * pid_bot.getSetpoint() / 6000);
	}

	@Override
	public void periodic() {
		runShooters();
		runPivot();
		boolean toggle = ds.button(2).getAsBoolean();
		// angleMotor.setIdleMode(toggle ? IdleMode.kBrake : IdleMode.kCoast);
		// angleMotor2.setIdleMode(toggle ? IdleMode.kBrake : IdleMode.kCoast);

		Logger.recordOutput("Shooter/IsReady", isReady());
		Logger.recordOutput("Shooter/Angle", angleEncoder.getAbsolutePosition());
		Logger.recordOutput("Shooter/CorrectedAngle", getAngle().getRotations());

		Logger.recordOutput("Shooter/Motors/TopMotor/Speed", sm_top.getEncoder().getVelocity());
		Logger.recordOutput("Shooter/Motors/BotMotor/Speed", sm_bot.getEncoder().getVelocity());

		Logger.recordOutput("Shooter/Motors/TopMotor/Current", top_curr.calculate(sm_top.getOutputCurrent()));
		Logger.recordOutput("Shooter/Motors/BotMotor/Current", bot_curr.calculate(sm_bot.getOutputCurrent()));

		Logger.recordOutput("Shooter/Pivot1Position", angleMotor.getEncoder().getPosition());
		Logger.recordOutput("Shooter/Pivot2Position", angleMotor2.getEncoder().getPosition());
		Logger.recordOutput("Shooter/BeamBreakOutput", ringSensorAnalogInput.getVoltage());
		Logger.recordOutput("Shooter/Ring Present", ringLoaded());
		Logger.recordOutput("Shooter/AtSetpoint", angleAtSetpoint());

		Logger.recordOutput("Shooter/MiddleBeamBreak", middleBeamBreak.get());
	}

	public double calculateAngle() {
		double targetY = TARGET_Y;
		double targetX = Util.isRed() ? TARGET_X_RED : TARGET_X_BLUE;
		double dx = drive.getPose().getX() - targetX;
		double dy = drive.getPose().getY() - targetY;
		double distance = Math.hypot(dx, dy);

		double y = TARGET_HEIGHT - SHOOTER_HEIGHT;
		double flight_time = distance
				/ (NOTE_VELOCITY)
				* MathUtil.clamp(sm_bot.getEncoder().getVelocity() / pid_bot.getSetpoint(), 0.25, 1);
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
		pid_bot.setSetpoint(speed * 6000);
		pid_top.setSetpoint(speed * 0.85 * 6000);
	}

	public boolean isReady() {
		return Math.abs(pid_bot.getSetpoint() - en_bot.getVelocity()) < 250
				&& Math.abs(angle_pid.getPositionError()) < 2;
	}

	// Commands
	public Command runFeeder() {
		return this.run(() -> runFeederMotor(!middleBeamBreak.get() ? 0.3 : 0.7))
				.until(ringSensor)
				.finallyDo(() -> runFeederMotor(0));
		// return this.startEnd(() -> runFeederMotor(0.3), () -> runFeederMotor(0)).until(ringSensor;
	}

	public Command autoShoot() {
		return shootSingle(Constants.SHOOTER_SPEAKER_SPEED)
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
