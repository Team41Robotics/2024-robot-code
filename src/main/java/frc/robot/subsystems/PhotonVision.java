package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
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
	private PhotonCamera cam;
	AprilTagFieldLayout fieldLayout;
	Transform3d robotToCam = new Transform3d(new Translation3d(0, 0.2, 0.1), new Rotation3d(0, 0, 0));
	PhotonPoseEstimator photonPoseEstimator;

	enum Cam_Mode {
		APRILTAG,
		NOTESLAM
	}

	private Cam_Mode currentMode;
	private Pose2d camRobot = new Pose2d(Units.inchesToMeters(9.5), Units.inchesToMeters(7), new Rotation2d());

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

	public void switchMode(int pipelineIndex) {
		cam.setPipelineIndex(pipelineIndex);
		currentMode = Cam_Mode.values()[pipelineIndex];
	}

	public Optional<Pose2d> getNearestNote() {
		if (currentMode == Cam_Mode.APRILTAG) return Optional.empty();
		PhotonTrackedTarget target = cam.getLatestResult().getBestTarget();
		if(target == null) return Optional.empty();
		double pitch = target.getPitch();
		double yaw = target.getYaw();
		double dx = Constants.cam_height / Math.tan(Units.degreesToRadians(pitch));
		// System.out.println("Dx: " + dx);
		double dy = dx * Math.tan(Units.degreesToRadians(yaw)) - Units.inchesToMeters(7);
		// System.out.println("Dy: " + dy);
		Pose2d noteCam = new Pose2d(dx, dy, new Rotation2d());
		return Optional.of(noteCam.relativeTo(camRobot));
	}

	public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
		if (currentMode == Cam_Mode.NOTESLAM) return Optional.empty();
		photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);

		if (cam.getLatestResult().getTargets().size() >= 2) return photonPoseEstimator.update();
		return Optional.empty();
	}
}
