package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase{
	
			CANSparkMax pivotMotor = new CANSparkMax(30, MotorType.kBrushless);
			CANSparkMax turnMotor = new CANSparkMax(31, MotorType.kBrushless);
			PIDController pivotPID = new PIDController(0, 0, 0);

			DutyCycleEncoder iEncoder = new DutyCycleEncoder(1);

			public void init(){
				pivotMotor.setIdleMode(IdleMode.kBrake);
				turnMotor.setIdleMode(IdleMode.kBrake);
			}

			public void runIntake(double speed){
					turnMotor.set(speed);
			}

			public void setPivot(double angle){
				double current_angle = iEncoder.getAbsolutePosition();
				while(Math.abs(current_angle-angle) >= 0.5){
					pivotMotor.set(pivotPID.calculate(current_angle, angle));
				}
			}	

			public void zeroAbsoluteEncoder(){
				iEncoder.setPositionOffset(iEncoder.getAbsolutePosition());
			}


}
