package frc.robot.commands;

import static frc.robot.RobotContainer.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.Util;

public class AlignToSpeaker extends Command {

	// TODO: Tune these
	public PIDController yPID = new PIDController(10.0, 1, 0.50);
	public PIDController omegaPID = new PIDController(2, 0, 0);

	public double targetY = 5.5;
	public double targetRotation = Units.degreesToRadians( Util.isRed() ? 180 : 0);

	public AlignToSpeaker() {
		addRequirements(drive);
		omegaPID.enableContinuousInput(0, Math.PI * 2);
	}

	@Override
	public void execute() {

		Pose2d current_pose = drive.getPose();

		double currentRotation = current_pose.getRotation().getRadians();
		double currentY = current_pose.getY();

		double vY = MathUtil.clamp(yPID.calculate(currentY, targetY), -1, 1);
		if (Units.radiansToDegrees(currentRotation) < 0) currentRotation = 2 * Math.PI + currentRotation;
		double vW = MathUtil.clamp(omegaPID.calculate(currentRotation, targetRotation), -1, 1);

		System.out.println("Cr: " + currentRotation + "Tr: " + targetRotation);
		drive.drive(
				ChassisSpeeds.fromFieldRelativeSpeeds(0, vY, vW, drive.getPose().getRotation()));
	}

	@Override
	public boolean isFinished() {
		Pose2d current_pose = drive.getPose();
		double currentRotation = current_pose.getRotation().getRadians();
		if (Math.abs(current_pose.getY() - targetY) < 0.05 && Math.abs(targetRotation - currentRotation) < 0.05) {
			return true;
		}
		return false;
	}
}
