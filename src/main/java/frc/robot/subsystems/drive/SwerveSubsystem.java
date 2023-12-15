package frc.robot.subsystems.drive;

import static frc.robot.RobotContainer.*;
import static frc.robot.constants.Constants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.SwerveModuleConfiguration;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

public class SwerveSubsystem extends SubsystemBase {
	public SwerveModule[] modules = new SwerveModule[] {
		new SwerveModule(new ModuleIOSparkMax(SwerveModuleConfiguration.NW), "NW"),
		new SwerveModule(new ModuleIOSparkMax(SwerveModuleConfiguration.NE), "NE"),
		new SwerveModule(new ModuleIOSparkMax(SwerveModuleConfiguration.SW), "SW"),
		new SwerveModule(new ModuleIOSparkMax(SwerveModuleConfiguration.SE), "SE"),
	};
	public SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
			new Translation2d(ROBOT_LENGTH / 2, ROBOT_WIDTH / 2),
			new Translation2d(ROBOT_LENGTH / 2, -ROBOT_WIDTH / 2),
			new Translation2d(-ROBOT_LENGTH / 2, ROBOT_WIDTH / 2),
			new Translation2d(-ROBOT_LENGTH / 2, -ROBOT_WIDTH / 2));
	public SwerveDrivePoseEstimator pose_est;
	

	Field2d field = new Field2d();

	public void initShuffleboard() {
		Shuffleboard.getTab("swerve").add(field);
	}

	public void init(Pose2d init_pose) {
		pose_est = new SwerveDrivePoseEstimator(kinematics, new Rotation2d(imu.yaw()), getPositions(), init_pose);
		AutoBuilder.configureHolonomic(
				this::getPose,
				(pose) -> {
					pose_est.resetPosition(new Rotation2d(imu.yaw()), getPositions(), pose);
				},
				this::getVelocity,
				this::drive,
				new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your
						// Constants class
						new PIDConstants(1.0, 0.0, 0.0), // Translation PID constants
						new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
						.5, // Max module speed, in m/s
						0.4488, // Drive base radius in meters. Distance from robot center to furthest module.
						new ReplanningConfig() // Default path replanning config. See the API for the options here
						),
				this);

		PathPlannerLogging.setLogActivePathCallback((activePath) -> {
			Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
		});
		PathPlannerLogging.setLogTargetPoseCallback((targetPose) -> {
			Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
		});
	}

	public SwerveModulePosition[] getPositions() {
		SwerveModulePosition[] pos = new SwerveModulePosition[4];
		for (int i = 0; i < 4; i++) pos[i] = modules[i].getPosition();
		return pos;
	}

	public ChassisSpeeds getVelocity() {
		SwerveModuleState[] pos = new SwerveModuleState[4];
		for (int i = 0; i < 4; i++) pos[i] = modules[i].getMeasuredState();
		return kinematics.toChassisSpeeds(pos);
	}

	public ChassisSpeeds desired_speeds = new ChassisSpeeds();

	public void drive(ChassisSpeeds speed) {
		speed = ChassisSpeeds.discretize(speed, 0.02);
		desired_speeds = speed;
		SwerveModuleState[] states = kinematics.toSwerveModuleStates(speed);
		SwerveDriveKinematics.desaturateWheelSpeeds(states, Math.max(SWERVE_MAXSPEED, 5));
		for (int i = 0; i < 4; i++) {
			modules[i].setState(states[i]);
		}
	}

	public void getOffsets() {
		for (SwerveModule module : modules) module.fixOffset();
	}

	public void zero() {
		for (int i = 0; i < 4; i++) {
			modules[i].setState(new SwerveModuleState());
		}
	}

	public void cross() {
		for (int i = 0; i < 4; i++) {
			modules[i].setState(new SwerveModuleState(0, new Rotation2d(45)));
		}
	}

	public void periodic() {
		Logger.recordOutput("/Swerve/desired_speeds", new double[] {
			desired_speeds.vxMetersPerSecond, desired_speeds.vyMetersPerSecond, desired_speeds.omegaRadiansPerSecond
		});
		ChassisSpeeds speeds = getVelocity();
		Logger.recordOutput(
				"/Swerve/actual_speeds",
				new double[] {speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond});
		for (SwerveModule module : modules) module.periodic();
		pose_est.update(new Rotation2d(imu.yaw()), getPositions());
		Optional<EstimatedRobotPose> vis_pos = photon.getEstimatedGlobalPose(pose_est.getEstimatedPosition());
		if (vis_pos.isPresent()) {
			EstimatedRobotPose new_pose = vis_pos.get();
			pose_est.addVisionMeasurement(new_pose.estimatedPose.toPose2d(), new_pose.timestampSeconds);
		}

		field.setRobotPose(pose_est.getEstimatedPosition());
		Logger.recordOutput("odom", pose_est.getEstimatedPosition());
		Logger.recordOutput("odom_rot", pose_est.getEstimatedPosition().getRotation());
		// SwerveModuleState[] states = new SwerveModuleState[4];
		// for (int i = 0; i < 4; i++) states[i] = modules[i].getTargetState();
		double[] states = new double[8];
		for (int i = 0; i < 4; i++) states[i * 2 + 1] = modules[i].getTargetState().speedMetersPerSecond;
		for (int i = 0; i < 4; i++)
			states[i * 2] = modules[i].getTargetState().angle.getRadians();
		Logger.recordOutput("target States", states);
		for (int i = 0; i < 4; i++) states[i * 2 + 1] = modules[i].getMeasuredState().speedMetersPerSecond;
		for (int i = 0; i < 4; i++)
			states[i * 2] = modules[i].getMeasuredState().angle.getRadians();
		Logger.recordOutput("measured States", states);
		double[] curr_pos = {
			pose_est.getEstimatedPosition().getX(),
			pose_est.getEstimatedPosition().getY(),
			pose_est.getEstimatedPosition().getRotation().getRadians()
		};
		Logger.recordOutput("Current Pos", curr_pos);
		Logger.recordOutput("curr_x", pose_est.getEstimatedPosition().getX());
		Logger.recordOutput("curr_y", pose_est.getEstimatedPosition().getY());
	}

	public Pose2d getPose() {
		return pose_est.getEstimatedPosition();
	}

	public Command followPath(String fileString) {
		PathPlannerPath path = PathPlannerPath.fromPathFile(fileString);
		return new FollowPathHolonomic(
				path,
				this::getPose,
				this::getVelocity,
				this::drive,
				new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your
						// Constants class
						new PIDConstants(1.0, 0.0, 0.0), // Translation PID constants
						new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
						.5, // Max module speed, in m/s
						0.4488, // Drive base radius in meters. Distance from robot center to furthest module.
						new ReplanningConfig() // Default path replanning config. See the API for the options here
						),
				this);
	}
}
