package frc.robot.commands.intake;

import static frc.robot.RobotContainer.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class SetPivot extends Command{
        
        public SetPivot(DoubleSupplier target){
                addRequirements(intake);
                intake.pivotPID.setSetpoint(target.getAsDouble());
        }

        public SetPivot(double target){
                this (() -> target);
        }

        @Override
        public void execute(){
                intake.setAngle(new Rotation2d());
        }

        @Override
        public boolean isFinished(){
                return intake.angleAtSetpoint();
        }



}
