package frc.robot.subsystems.shooter;

import static frc.robot.RobotContainer.drive;
import static frc.robot.constants.Constants.FEEDER_MOTOR;
import static frc.robot.constants.Constants.FieldConstants.*;
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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import frc.robot.util.Util;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * The ShooterSubsystem class represents the shooter subsystem of the robot.
 * It controls the angle and speed of the shooter motors, as well as the feeder motor.
 * The subsystem also provides methods for calculating the shooting angle and checking if a ring is loaded.
 */
public class ShooterSubsystem extends SubsystemBase {
	public final CANSparkMax angleMotor = new CANSparkMax(SHOOTER_PIVOT_MOTOR1, MotorType.kBrushless);
	public final CANSparkMax angleMotor2 = new CANSparkMax(SHOOTER_PIVOT_MOTOR2, MotorType.kBrushless);

	public final CANSparkFlex sm_top = new CANSparkFlex(SHOOTER_MOTOR_TOP, MotorType.kBrushless);
	public final CANSparkFlex sm_bot = new CANSparkFlex(SHOOTER_MOTOR_BOT, MotorType.kBrushless);
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
	public PIDController angle_pid = new PIDController(5e-2, 0, 0.0);

	private Optional<Rotation2d> target_angle = Optional.empty();

	public double top_speed = 0;
	public double bot_speed = 0;

	public ShooterSubsystem() {

		sm_bot.restoreFactoryDefaults();
		sm_top.restoreFactoryDefaults();
		angleMotor2.follow(angleMotor, true);
		angleMotor.setIdleMode(IdleMode.kBrake);
		angleMotor2.setIdleMode(IdleMode.kBrake);

		// sm_top.setSmartCurrentLimit(60);
		// sm_bot.setSmartCurrentLimit(60);

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
	 * @return angle: from 90 straight forward to 0 straight up
	 */
	public Rotation2d getAngle() {
		double angle = angleEncoder.getAbsolutePosition();
		// if (angle < 0.3) angle += 1;
		angle -= angleEncoder.getPositionOffset();
		return Rotation2d.fromRotations(-angle);
	}

	/**
	 * Runs the pivot mechanism of the shooter subsystem.
	 * If the target angle is not set, the method returns without performing any action.
	 * Calculates the output using the angle PID controller and records relevant outputs to the logger.
	 * Sets the angle motor output based on the calculated output, clamped between -0.25 and 0.25 volts.
	 */
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

	/**
	 * Runs the shooters and records their output.
	 * If the setpoint of the bottom shooter is 0, the voltage of both shooters is set to 0.
	 * Otherwise, the voltage of each shooter is calculated based on PID + FeedForward.
	 */
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
		logTelemetry();
	}

	/**
	 * Logs telemetry data for the shooter subsystem.
	 * This method records various sensor readings and motor outputs to the logger.
	 */
	private void logTelemetry() {
		Logger.recordOutput("Shooter/IsReady", isReady());
		Logger.recordOutput("Shooter/Angle", angleEncoder.getAbsolutePosition());
		Logger.recordOutput("Shooter/CorrectedAngle", getAngle().getRotations());

		Logger.recordOutput("Shooter/Motors/TopMotor/Speed", sm_top.getEncoder().getVelocity());
		Logger.recordOutput("Shooter/Motors/BotMotor/Speed", sm_bot.getEncoder().getVelocity());

		Logger.recordOutput("Shooter/Motors/TopMotor/setPoint", pid_top.getSetpoint());
		Logger.recordOutput("Shooter/Motors/BotMotor/setPoint", pid_bot.getSetpoint());

		Logger.recordOutput("Shooter/Motors/TopMotor/Current", top_curr.calculate(sm_top.getOutputCurrent()));
		Logger.recordOutput("Shooter/Motors/BotMotor/Current", bot_curr.calculate(sm_bot.getOutputCurrent()));

		Logger.recordOutput("Shooter/Pivot1Position", angleMotor.getEncoder().getPosition());
		Logger.recordOutput("Shooter/Pivot2Position", angleMotor2.getEncoder().getPosition());
		Logger.recordOutput("Shooter/BeamBreakOutput", ringSensorAnalogInput.getVoltage());
		Logger.recordOutput("Shooter/Ring Present", ringLoaded());
		Logger.recordOutput("Shooter/AtSetpoint", angleAtSetpoint());

		Logger.recordOutput("Shooter/MiddleBeamBreak", middleBeamBreak.get());
	}

	/**
	 * Calculates the angle needed to shoot the target based on the robot's position and target coordinates.
	 * <p>
	 * Uses estimated drop due to gravity along with inverse trig to
	 *
	 * @return The angle in degrees.
	 */
	public double calculateAngle() {
		double targetY = TARGET_Y;
		double targetX = Util.isRed() ? TARGET_X_RED : TARGET_X_BLUE;
		double dx = drive.getPose().getX() - targetX;
		double dy = drive.getPose().getY() - targetY;
		double distance = Math.hypot(dx, dy);

		double y = TARGET_HEIGHT - SHOOTER_HEIGHT;
		double flight_time = distance
				/ (NOTE_VELOCITY + drive.getXVel())
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

	/**
	 * Sets the angle of the shooter, and clamps it between 0 and 90 degrees.
	 *
	 * @param angle the desired angle of the shooter
	 */
	public void setAngle(Rotation2d angle) {
		if (angle.getDegrees() < 0 || angle.getDegrees() > 90) {
			this.target_angle = Optional.of(Rotation2d.fromDegrees(15));
			System.out.println("ERROR: Out of bounds angle");
			return;
		}
		this.target_angle = Optional.of(angle);
	}

	/**
	 * Sets the feeder motor to run at the specified percentage.
	 * 
	 * @param percent the percentage at which to run the feeder motor, ranging from -1.0 to 1.0
	 */
	public void runFeederMotor(double percent) {
		feeder.set(percent);
	}

	/**
	 * Checks if a ring is loaded in the shooter subsystem.
	 *
	 * @return true if a ring is loaded, false otherwise
	 */
	public boolean ringLoaded() {
		return ringSensor.getAsBoolean();
	}

	/**
	 * Runs the shooter motors at the specified speed, with the top motor not being changed until after 0.6 seconds
	 * the top motor gets run at 0.85 times the speed of the bottom motor
	 *
	 * @param speed the speed at which to run the motors from 0-1
	 */
	public void runMotors(double speed) {
		pid_bot.setSetpoint(speed * 6000);
		new WaitCommand(0.6)
				.andThen(new InstantCommand(() -> pid_top.setSetpoint(speed * 0.85 * 6000)))
				.schedule();
	}

	/**
	 * Checks if the shooter is ready to shoot.
	 * The shooter is considered ready if the top and bottom PID setpoints are within 250 rpm of their respective velocities,
	 * and the angle PID position error is less than 2 degrees.
	 *
	 * @return true if the shooter is ready, false otherwise
	 */
	public boolean isReady() {
		return (pid_top.getSetpoint() - en_top.getVelocity() < 250 && Math.abs(angle_pid.getPositionError()) < 2)
				&& (pid_bot.getSetpoint() - en_bot.getVelocity() < 250 && Math.abs(angle_pid.getPositionError()) < 2);
	}

	/**
	 * Runs the feeder motor until the ring sensor is triggered and then stops the motor.
	 *
	 * @return The command to run the feeder.
	 */
	public Command loadNote() {
		return this.run(() -> runFeederMotor(0.2)).until(ringSensor).finallyDo(() -> runFeederMotor(0));
	}

	/**
	 * Executes the autoShoot command. Spins the motors up to shooting speed and sets the target angle to optimal angle
	 *
	 * @return The command to start autoshoot
	 */
	public Command autoShoot() {
		return shootSingle(Constants.SHOOTER_SPEAKER_SPEED)
				.deadlineWith(new RunCommand(() -> setAngle(Rotation2d.fromDegrees(calculateAngle()))))
				.andThen(new InstantCommand(() -> runMotors(0.4)));
	}

	/**
	 * Creates a command to shoot a note ball at a specified speed.
	 *
	 * @param speed the speed at which to shoot the note
	 * @return the command to shoot the note
	 */
	public Command shootSingle(double speed) {
		return new StartEndCommand(() -> runMotors(speed), () -> runMotors(0)).until(ringSensor.negate());
	}

	/**
	 * Creates a Command object that sets the angle of the pivot based on the provided angle supplier.
	 *
	 * @param angle the supplier that provides the desired angle for the pivot
	 * @return the command
	 */
	public Command toAngleCommand(Supplier<Rotation2d> angle) {
		return new InstantCommand(() -> setAngle(angle.get()));
	}

	/**
	 * Creates a Command object that sets the pivot to a specific angle.
	 *
	 * @param angle the desired angle for the pivot subsystem
	 * @return the command
	 */
	public Command toAngleCommand(Rotation2d angle) {
		return this.toAngleCommand(() -> angle);
	}

	/**
	 * Returns a Command object that sets the pivot to a specific angle in degrees.
	 *
	 * @param angle the desired angle in degrees
	 * @return the command
	 */
	public Command toAngleDegreeCommand(double angle) {
		return this.toAngleCommand(() -> Rotation2d.fromDegrees(angle));
	}

	/**
	 * Preset speeds for a shot in the AMP
	 * TODO: Make Work
	 * @return The shooting command.
	 */
	public Command ampShoot() {
		return this.runOnce(() -> {
			pid_bot.setSetpoint(0.18);
			pid_top.setSetpoint(-0.1);
		});
	}

	/**
	 * Returns a Command object that represents the process of muzzle loading.
	 * The muzzle loading process includes running the motors at a negative speed,
	 * waiting until the ring sensor detects a ring, running the feeder motor at a negative speed,
	 * waiting until the ring sensor no longer detects a ring, running the feeder,
	 * and finally stopping the motors.
	 * <p>
	 * yes, this is cursed, but it works quite well
	 * The note will get sucked in, go past the sensor, and then moved forward a tiny bit to be ready to fire
	 * @return a Command object representing the muzzle loading process
	 */
	public Command muzzleLoad() {
		return new SequentialCommandGroup(
				new InstantCommand(() -> runMotors(-0.2)),
				new WaitUntilCommand(ringSensor),
				new InstantCommand(() -> runFeederMotor(-0.2)),
				new WaitUntilCommand(ringSensor.negate()),
				this.loadNote(),
				stopMotors());
	}

	/**
	 * Returns a Command object that represents a subwoofer shot.
	 * This command first moves the shooter to a 35-degree angle,
	 * and then shoots a single projectile with a power of 0.7.
	 *
	 * @return The Command object representing the subwoofer shot.
	 */
	public Command subwooferShot() {
		return toAngleDegreeCommand(35).andThen(shootSingle(0.7));
	}

	/**
	 * Stops the motors of the shooter subsystem.
	 * 
	 * @return The command to stop the motors.
	 */
	public Command stopMotors() {
		return this.runOnce(() -> runMotors(0));
	}

	/**
	 * Returns a Command object that represents a feeder shot.
	 * This command first sets the shooter angle to 80 degrees using the toAngleDegreeCommand method,
	 * and then shoots a single ball with a power of 0.7 using the shootSingle method.
	 *
	 * @return a Command object representing a feeder shot
	 */
	public Command feederShot() {
		return toAngleDegreeCommand(80).andThen(shootSingle(0.7));
	}
}
