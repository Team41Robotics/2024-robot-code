package frc.robot.commands;

import static frc.robot.RobotContainer.drive;
import static frc.robot.RobotContainer.photon;
import static frc.robot.RobotContainer.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class LaunchRing extends Command{
        /* 
        public LaunchRing(){
                addRequirements(shooter);
                addRequirements(drive);
        }

        @Override 
        public void execute(){
                shooter.setAngle(calculateAngle());
                shooter.runMotors();
        }

        public double calculateAngle(){

                double robotX = drive.getPose().getX();
                double distance = Math.sqrt(Math.pow((180 - robotX),2) + Math.pow(180 - drive.getPose().getY(),2));

                /*double y = 180-Units.inchesToMeters(15);

                return Math.atan((y/distance));

        }
*/
}