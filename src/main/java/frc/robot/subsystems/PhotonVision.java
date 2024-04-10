package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.Constants;
import java.io.IOException;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonVision {
	private PhotonCamera note_cam;
	private PhotonCamera april_cam;
	AprilTagFieldLayout fieldLayout;

	Transform3d robotToCam = new Transform3d(
			new Translation3d(Units.inchesToMeters(2 * 14.5), Units.inchesToMeters(0), 0.1),
			new Rotation3d(0, Units.degreesToRadians(30), 0));
	PhotonPoseEstimator photonPoseEstimator;

	private Pose2d camRobot = new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(9.5), new Rotation2d());

	public PhotonVision() {
		try {
			fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
		} catch (IOException e) {
			System.out.println("Couldn't Find April Tag Layout File");
			e.printStackTrace();
		}
		note_cam = null; // ew PhotonCamera("HD_USB_Camera");

		april_cam = new PhotonCamera("Global_Shutter_Camera");
		photonPoseEstimator =
				new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, april_cam, robotToCam);
		photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
	}

	/**
	 * Returns the nearest note's pose relative to the robot's pose.
	 *
	 * @return An Optional containing the nearest note's pose if it exists, or an empty Optional otherwise.
	 */
	public Optional<Pose2d> getNearestNote() {
		if (note_cam == null) return Optional.empty();
		if (!note_cam.getLatestResult().hasTargets()) return Optional.empty();
		PhotonTrackedTarget target = note_cam.getLatestResult().getBestTarget();
		if (target == null) return Optional.empty();
		double pitch = Units.degreesToRadians(target.getPitch());
		double yaw = Units.degreesToRadians(target.getYaw());
		double dx = Constants.CAMERA_HEIGHT / Math.tan(pitch);
		double dy = dx * Math.tan(yaw);
		Transform2d noteCam = new Transform2d(dx, dy, new Rotation2d());
		if (pitch > 0) return Optional.empty();
		else return Optional.of(camRobot.plus(noteCam));
	}

	/**
	 * Returns an optional EstimatedRobotPose object representing the estimated global pose of the robot.
	 * If the April camera is not available, not connected, or does not detect at least 2 targets, an empty optional is returned.
	 *
	 * @return an optional EstimatedRobotPose object representing the estimated global pose of the robot, or an empty optional if the pose cannot be estimated
	 */
	public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
		if (april_cam == null) return Optional.empty();
		if (!april_cam.isConnected()) return Optional.empty();
		if (april_cam.getLatestResult().getTargets().size() < 2) return Optional.empty();

		return photonPoseEstimator.update();
	}
}
