package frc.robot.commands.drive;

import static frc.robot.RobotContainer.drive;
import static frc.robot.RobotContainer.imu;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class ResetOdom extends Command {

	public ResetOdom() {
		addRequirements(drive);
	}
	// TODO whoever wrote this: you realize that pose_est.
	/*
	public void resetPosition(
		Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d poseMeters) {
		*/
	public void execute() {

		Pose2d pose = new Pose2d();

		drive.pose_est = new SwerveDrivePoseEstimator(
				drive.kinematics,
				new Rotation2d(imu.yaw()),
				drive.getPositions(),
				pose, // I have no idea what the pose for the podium is lmao
				VecBuilder.fill(0.1, 0.1, 0.1),
				VecBuilder.fill(2, 2, 2));
	}

	public boolean isFinished() {
		return true;
	}
}
