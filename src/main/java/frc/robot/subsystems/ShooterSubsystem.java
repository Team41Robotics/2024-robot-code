package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class ShooterSubsystem extends SubsystemBase {
		/* 
		CANSparkFlex im1 = new CANSparkFlex(Constants.SHOOTER_MOTOR_1, MotorType.kBrushless);
		CANSparkFlex im2 = new CANSparkFlex(Constants.SHOOTER_MOTOR_2, MotorType.kBrushless);
		CANSparkMax angleMotor = new CANSparkMax(Constants.SHOOTER_ANGLE_MOTOR, MotorType.kBrushless);

		public double im1S, im2S;
		public boolean enabled;


		public PIDController lef_pid = new PIDController(4e-4, 3e-4, 0);
		public PIDController rgt_pid = new PIDController(4e-4, 3e-4, 0);
		public PIDController angle_pid = new PIDController(4e-4, 3e-4, 0);

		public void init(){
				im1S = 0;
				im2S = 0;
				enabled = false;
		}

		@Override
		public void periodic() {


		}

		public void runMotors(){
				im1.set(im1S);
				im2.set(im2S);
		}


		public void stopMotors(){
				im1.set(0);
				im2.set(0);
		}

		public void setAngle(double angle){
			angle_pid.setSetpoint(angle);
		}
*/
}

