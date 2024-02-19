package frc.robot.commands.Shooter;

import static frc.robot.RobotContainer.shooter;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

public class toAngle extends Command{
        DoubleSupplier target_angle;
        PIDController t1Controller = new PIDController(2, 0, 0);
        //angle is from horizontal
        public toAngle(DoubleSupplier angle){

                addRequirements(shooter);
                target_angle = () -> Units.degreesToRotations(90 - angle.getAsDouble());
                System.out.println("Converted: " + Units.degreesToRotations(90 - angle.getAsDouble()));
                t1Controller.setSetpoint(target_angle.getAsDouble());
                t1Controller.setTolerance(0.01);
        }
        public toAngle(double angle){
                this(() -> angle);
        }
        
        @Override
        public void execute() {
                double angle = shooter.shooterAngle();
                System.out.println(shooter.calculateAngle());
                double output = t1Controller.calculate(angle, target_angle.getAsDouble());
                Logger.recordOutput("PID_output", output);
                Logger.recordOutput("PID_error",t1Controller.getPositionError());
                shooter.angleMotor.set(output);
                shooter.angleMotor2.set(output);
        }
        @Override
        public boolean isFinished() {
                return Math.abs(t1Controller.getPositionError()) >= 0.5;
                //t1Controller.atSetpoint();
        }
}