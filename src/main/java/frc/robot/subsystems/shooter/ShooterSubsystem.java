package frc.robot.subsystems.shooter;

import static frc.robot.RobotContainer.drive;
import static frc.robot.constants.Constants.SHOOTER_HEIGHT;
import static frc.robot.constants.Constants.TARGET_HEIGHT;

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
import frc.robot.constants.Constants;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {
	public CANSparkMax angleMotor = new CANSparkMax(Constants.SHOOTER_PIVOT_MOTOR1, MotorType.kBrushless);
	public CANSparkMax angleMotor2 = new CANSparkMax(Constants.SHOOTER_PIVOT_MOTOR2, MotorType.kBrushless);

	private final CANSparkFlex sm1 = new CANSparkFlex(Constants.SHOOTER_MOTOR_1, MotorType.kBrushless);
	private final CANSparkFlex sm2 = new CANSparkFlex(Constants.SHOOTER_MOTOR_2, MotorType.kBrushless);

	public final CANSparkMax feeder = new CANSparkMax(3, MotorType.kBrushless);
	DutyCycleEncoder angleEncoder = new DutyCycleEncoder(0);
	public double targetX = Units.inchesToMeters(
			(DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red)) ? -1.5 : 652.3);
	public double targetY = Units.inchesToMeters(218.42);
	ShooterSparkFlexIO io;

	public double ims;
	public boolean enabled;

	public PIDController lef_pid = new PIDController(4e-4, 3e-4, 0);
	public PIDController rgt_pid = new PIDController(4e-4, 3e-4, 0);
	// public PIDController angle_pid = new PIDController(4e-4, 3e-4, 0);

	public ShooterSubsystem(ShooterSparkFlexIO nIO) {
		this.io = nIO;
		angleMotor.setIdleMode(IdleMode.kBrake);
		angleMotor2.setIdleMode(IdleMode.kBrake);
		sm1.setIdleMode(IdleMode.kCoast);
		sm2.setIdleMode(IdleMode.kCoast);
		angleEncoder.setPositionOffset(0.89);
	}

	public void init() {
		ims = 0;
		enabled = false;
	}

	@Override
	public void periodic() {
		// io.updateInputs();
		double angle = angleEncoder.getAbsolutePosition();
		if (angle < 0.3) {
			angle += 1;
		}
		Logger.recordOutput("Shooter", angle);
		Logger.recordOutput("Angle_adjusted", angle - angleEncoder.getPositionOffset());
	}

	public double shooterAngle() {
		double angle = angleEncoder.getAbsolutePosition();
		if (angle < 0.3) {
			angle += 1;
		}

		return angle - angleEncoder.getPositionOffset();
	}

	public double calculateAngle() {
		double dx = drive.getPose().getX() - targetX;
		double dy = drive.getPose().getY() - targetY;
		double distance = Math.hypot(dx, dy);
		System.out.println("Distance: " + distance);

		double y = TARGET_HEIGHT - SHOOTER_HEIGHT;
		double flight_time = distance / drive.note_vel;
		y += 9.8 / 2 * flight_time * flight_time;
		return Units.radiansToDegrees(Math.atan((y / distance)));
	}

	public void runMotors() {
		sm1.set(.5);
		sm2.set(0.4);
		// io.setVelocity(ims);
		enabled = true;
	}

	public void zeroAbsoluteEncoder() {
		System.out.println("curr pos: " + angleEncoder.getAbsolutePosition() + " curr offset: "
				+ angleEncoder.getPositionOffset() + " pos no offset: "
				+ (angleEncoder.getAbsolutePosition() - angleEncoder.getPositionOffset()));
		angleEncoder.setPositionOffset(angleEncoder.getAbsolutePosition());
	}

	public void setAngle(double angle) {
		double current_angle = angleEncoder.getAbsolutePosition();
		while (current_angle - angle > 0.5) {
			angleMotor.set(lef_pid.calculate(current_angle, angle));
			angleMotor2.set(rgt_pid.calculate(current_angle, angle));
		}
	}

	public void runIntakeMotor(double speed) {
		feeder.set(speed);
	}

	public void setSpeed(double speed) {
		sm1.set(speed);
		sm2.set(-speed * 0.85);
		ims = speed;
	}
}
