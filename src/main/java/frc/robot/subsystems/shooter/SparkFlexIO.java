package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalOutput;
import frc.robot.constants.Constants;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;

public class SparkFlexIO implements ShooterIO{
        
        private final CANSparkFlex sm1 = null;//new CANSparkFlex(Constants.SHOOTER_MOTOR_1, MotorType.kBrushless);
	private final CANSparkFlex sm2 = null;//new CANSparkFlex(Constants.SHOOTER_MOTOR_2, MotorType.kBrushless);
        //private DigitalOutput bBIO = new DigitalOutput(0);

        private PIDController sm1PID;
        private PIDController sm2PID;

        private double sm1s;
        private double sm2s;

        public SparkFlexIO(){
                sm1PID = new PIDController(0, 0, 0);
                sm2PID = new PIDController(0, 0, 0);
                //sm1.setIdleMode(IdleMode.kBrake);
                //sm2.setIdleMode(IdleMode.kBrake);
        }

        @Override
        public void setVelocity(double speed){
                sm1.set(sm1PID.calculate(speed));
                sm2.set(sm2PID.calculate(speed*1.05));
        }

        @Override
        public void stopMotors(){
                sm1.set(0);
                sm2.set(0);
        }

        public void updateInputs(ShooterIOInputs inputs){
                inputs.currentVelocityMotor1 = sm1s;
                inputs.currentVelocityMotor2 = sm2s;
                //inputs.currentBeamBrakeState = bBIO.get();
                inputs.currentVoltageMotor1 = sm1.getAppliedOutput() * sm1.getBusVoltage();
                inputs.currentVoltageMotor2 = sm2.getAppliedOutput() * sm2.getBusVoltage();

                inputs.currentAppliedAmpsMotor1 = new double[] {sm1.getOutputCurrent()};
                inputs.currentAppliedAmpsMotor2 = new double[] {sm2.getOutputCurrent()};

        }      

}
