package frc.robot.commands;

import static frc.robot.RobotContainer.drive;
import static frc.robot.RobotContainer.photon;
import frc.robot.constants.Constants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;


public class AlignToSpeaker extends Command {

        public PIDController yPID = new PIDController(0.5, 0 ,0.2);
        public PIDController omegaPID = new PIDController(0.5, 0, 0);

        public double targetY = Constants.APRILTAG_4_Y;
        public double targetRotation = Units.degreesToRadians(0);

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

                System.out.println(Math.abs(targetRotation - currentRotation));

                if(Math.abs(targetY - currentY) > 0.1){
                        drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
                                        0,
                                        yPID.calculate(currentY, targetY),
                                        omegaPID.calculate(currentRotation, targetRotation),
                                        drive.getPose().getRotation()));
                }         
	}

	@Override
	public boolean isFinished() {
		Pose2d current_pose = drive.getPose();
                double currentRotation = current_pose.getRotation().getRadians();
                if(Math.abs(current_pose.getY() - targetY) < 0.15 && Math.abs(targetRotation - currentRotation) < 0.1){
                        return true;
                }
                return false;
	}
}
