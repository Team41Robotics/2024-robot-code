package frc.robot.subsystems.shooter;

import static frc.robot.RobotContainer.drive;
import static frc.robot.constants.Constants.*;
import static java.lang.Math.PI;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Util;

import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {
	public final CANSparkMax angleMotor = new CANSparkMax(SHOOTER_PIVOT_MOTOR1, MotorType.kBrushless);
	public final CANSparkMax angleMotor2 = new CANSparkMax(SHOOTER_PIVOT_MOTOR2, MotorType.kBrushless);

	private final CANSparkFlex sm_top = new CANSparkFlex(SHOOTER_MOTOR_TOP, MotorType.kBrushless);
	private final CANSparkFlex sm_bot = new CANSparkFlex(SHOOTER_MOTOR_BOT, MotorType.kBrushless);

	public final CANSparkMax feeder = new CANSparkMax(FEEDER_MOTOR, MotorType.kBrushless);

	DutyCycleEncoder angleEncoder = new DutyCycleEncoder(SHOOTER_ENCODER);

	public PIDController angle_pid = new PIDController(4e-4, 3e-4, 0);

	public double top_speed = 0;
	public double bot_speed = 0;

	public ShooterSubsystem() {
		angleMotor2.follow(angleMotor);
		angleMotor.setIdleMode(IdleMode.kBrake);
		sm_top.setIdleMode(IdleMode.kCoast);
		sm_bot.setIdleMode(IdleMode.kCoast);

		angleEncoder.setPositionOffset(SHOOTER_ENCODER_OFFSET);
	}

	public double getAngle() {
		double angle = angleEncoder.getAbsolutePosition();
		if (angle < 0.3) angle += 1;
		angle -= angleEncoder.getPositionOffset();
		return angle * 2 * PI;
	}

	public void init() {}

	@Override
	public void periodic() {
		Logger.recordOutput("Shooter/Angle", getAngle());
	}

	public double calculateAngle() {
		double targetY = TARGET_Y;
		double targetX = Util.isRed() ? TARGET_X_RED : TARGET_X_BLUE;
		double dx = drive.getPose().getX() - targetX;
		double dy = drive.getPose().getY() - targetY;
		double distance = Math.hypot(dx, dy);
		System.out.println("Distance: " + distance);

		double y = TARGET_HEIGHT - SHOOTER_HEIGHT;
		double flight_time = distance / NOTE_VELOCITY;
		y += 9.8 / 2 * flight_time * flight_time;
		return Units.radiansToDegrees(Math.atan(y / distance));
	}

	public void zeroAbsoluteEncoder() {
		System.out.println("curr pos: " + angleEncoder.getAbsolutePosition() + " curr offset: "
				+ angleEncoder.getPositionOffset() + " pos no offset: "
				+ (angleEncoder.getAbsolutePosition() - angleEncoder.getPositionOffset()));
		angleEncoder.setPositionOffset(angleEncoder.getAbsolutePosition());
	}

	public void setAngle(double angle) { // someone is kinda stpuid
		// TODO
	}

	public void runIntakeMotor(double volts) {
		feeder.setVoltage(volts);
	}
}
