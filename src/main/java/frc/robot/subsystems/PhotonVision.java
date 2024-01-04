package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.io.IOException;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class PhotonVision {
	private PhotonCamera cam;
	AprilTagFieldLayout fieldLayout;
	Transform3d robotToCam = new Transform3d(new Translation3d(0, 0.2, 0.1), new Rotation3d(0, 0, 0));
	PhotonPoseEstimator photonPoseEstimator;

	public PhotonVision() {
		try {
			fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
		} catch (IOException e) {
			System.out.println("Couldn't Find April Tag Layout File");
			e.printStackTrace();
		}
		cam = new PhotonCamera("Front_Camera");
		photonPoseEstimator =
				new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cam, robotToCam);
		photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
	}

	public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
		photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);

		if (cam.getLatestResult().getTargets().size() >= 2) return photonPoseEstimator.update();
		else return Optional.empty();
	}
}
