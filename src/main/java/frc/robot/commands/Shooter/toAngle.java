package frc.robot.commands.Shooter;

import static frc.robot.RobotContainer.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

public class toAngle extends Command{
        double target_angle;
        PIDController t1Controller = new PIDController(1, 0, 0);
        PIDController t2Controller = new PIDController(1, 0, 0);

        public toAngle(double angle){
                addRequirements(shooter);
                target_angle = angle;
                t1Controller.setSetpoint(target_angle);
                t2Controller.setSetpoint(target_angle);
                t1Controller.setTolerance(0.01);
        }
        
        @Override
        public void execute() {
                double angle = shooter.shooterAngle();
                
                Logger.recordOutput("PID_output", t1Controller.calculate(angle));
                Logger.recordOutput("PID_error",t1Controller.getPositionError());
                shooter.angleMotor.set(t1Controller.calculate(angle));
                shooter.angleMotor2.set(t2Controller.calculate(angle));
        }
        @Override
        public boolean isFinished() {
                return false;//t1Controller.atSetpoint();
        }
}
