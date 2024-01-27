package frc.robot.commands;

import static frc.robot.RobotContainer.drive;
import static frc.robot.RobotContainer.photon;
import frc.robot.constants.Constants;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;


public class AlignToSpeaker extends Command {

        // TODO: Tune these
        public PIDController yPID = new PIDController(10.0, 1 ,0.60);
        public PIDController omegaPID = new PIDController(10.0, 0, 0.60);

        public double targetY = 5.5;
        public double targetRotation = Units.degreesToRadians((DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Blue)) ? 0 : 180);

	public AlignToSpeaker() {
		addRequirements(drive);
	}

	@Override
	public void execute() {
                
		photon.switchMode(0);
		Pose2d current_pose = drive.getPose();
                
                double currentRotation = current_pose.getRotation().getRadians();
                double currentY = current_pose.getY();

                //System.out.println(currentRotation);
                //System.out.println(targetRotation);
                //System.out.println(Math.abs(targetRotation - currentRotation));

                System.out.println(targetY - currentY);
                        double vY = yPID.calculate(currentY, targetY);
                        if(vY > 1){
                                vY = 1;
                        }else if(vY < -1){
                                vY = -1;
                        }

                        

                        drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
                                        0,
                                        vY,
                                        omegaPID.calculate(currentRotation, targetRotation),
                                        drive.getPose().getRotation()));         
	}

	@Override
	public boolean isFinished() {
		Pose2d current_pose = drive.getPose();
                double currentRotation = current_pose.getRotation().getRadians();
                if(Math.abs(current_pose.getY() - targetY) < 0.05 /*&& Math.abs(targetRotation - currentRotation) < 0.02*/){
                        return true;
                }
                return false;
	}
}
