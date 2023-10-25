package frc.robot.subsystems;

import static frc.robot.constants.Constants.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.SwerveModule;
import frc.robot.constants.Ports;

public class SwerveSubsystem {
	SwerveModule[] modules = new SwerveModule[] {
		new SwerveModule(Ports.NW_ENCODER, Ports.NW_TURN_MOTOR, Ports.NW_DRIVE_MOTOR),
		new SwerveModule(Ports.NE_ENCODER, Ports.NE_TURN_MOTOR, Ports.NE_DRIVE_MOTOR),
		new SwerveModule(Ports.SW_ENCODER, Ports.SW_TURN_MOTOR, Ports.SW_DRIVE_MOTOR),
		new SwerveModule(Ports.SE_ENCODER, Ports.SE_TURN_MOTOR, Ports.SE_DRIVE_MOTOR)
	};
	SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
			new Translation2d(ROBOT_LENGTH / 2, ROBOT_WIDTH / 2),
			new Translation2d(ROBOT_LENGTH / 2, -ROBOT_WIDTH / 2),
			new Translation2d(-ROBOT_LENGTH / 2, ROBOT_WIDTH / 2),
			new Translation2d(-ROBOT_LENGTH / 2, -ROBOT_WIDTH / 2));

	public void drive(ChassisSpeeds speed) {
		SwerveModuleState[] states = kinematics.toSwerveModuleStates(speed);
		SwerveDriveKinematics.desaturateWheelSpeeds(states, SWERVE_MAXSPEED);
		for (int i = 0; i < 4; i++) {
			modules[i].setState(states[i]);
		}
	}

	public void periodic() {
		for (SwerveModule module : modules) module.periodic();
	}
}
