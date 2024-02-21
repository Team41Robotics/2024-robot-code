package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.Optional;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase{
        
        DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(new DigitalOutput(1));

        CANSparkMax pivotMotor = new CANSparkMax(2, MotorType.kBrushless);
        CANSparkMax turnMotor = new CANSparkMax(14, MotorType.kBrushless);

        public PIDController pivotPID = new PIDController(0, 0, 0);
        public PIDController turnPID = new PIDController(0, 0, 0);

        private Optional<Rotation2d> target_angle = Optional.empty();

        public IntakeSubsystem(){
                pivotMotor.setIdleMode(IdleMode.kBrake);
                turnMotor.setIdleMode(IdleMode.kCoast);
                pivotEncoder.setPositionOffset(pivotEncoder.getAbsolutePosition());
        }

        public void runIntakeMotor(){
                turnMotor.set(.5);
        }

        public void stopIntakeMotor(){
                turnMotor.set(0);
        }

        public boolean angleAtSetpoint(){
                return pivotPID.atSetpoint();
        }

        public Rotation2d getAngle(){
                double angle = pivotEncoder.getAbsolutePosition();
		if (angle < 0.3) angle += 1;
		angle -= pivotEncoder.getPositionOffset();
		return Rotation2d.fromRotations(angle);
        }

        public void setAngle(Rotation2d target){
                this.target_angle = Optional.of(target);
        }
         // Angles in degrees right now, may change in the future
        public void periodic(){
                if((this.target_angle.get().getDegrees() - this.getAngle().getDegrees()) > 1){
                        pivotMotor.set(pivotPID.calculate(this.target_angle.get().getDegrees(), this.getAngle().getDegrees()));
                }
        }

}
