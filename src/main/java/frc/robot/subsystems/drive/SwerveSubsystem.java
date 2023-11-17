package frc.robot.subsystems.drive;

import static frc.robot.RobotContainer.*;
import static frc.robot.constants.Constants.*;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.Ports;

public class SwerveSubsystem extends SubsystemBase {
	public SwerveModule[] modules = new SwerveModule[] {
		new SwerveModule(new ModuleIOSparkMax(0),0),
		new SwerveModule(new ModuleIOSparkMax(1),1),
		new SwerveModule(new ModuleIOSparkMax(2),2),
		new SwerveModule(new ModuleIOSparkMax(3),3),

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

	public void initOdom(Pose2d init_pose) {
		pose_est = new SwerveDrivePoseEstimator(kinematics, new Rotation2d(imu.yaw()), getPositions(), init_pose);
	}

	public SwerveModulePosition[] getPositions() {
		SwerveModulePosition[] pos = new SwerveModulePosition[4];
		for (int i = 0; i < 4; i++) pos[i] = modules[i].getPosition();
		return pos;
	}

	public void drive(ChassisSpeeds speed) {
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
		for (SwerveModule module : modules) module.periodic();
		pose_est.update(new Rotation2d(imu.yaw()), getPositions());
		field.setRobotPose(pose_est.getEstimatedPosition());
	}
}
