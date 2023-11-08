package frc.robot.subsystems;

import static frc.robot.constants.Constants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveModule;
import frc.robot.constants.Constants;
import frc.robot.constants.Ports;

public class SwerveSubsystem extends SubsystemBase {
	public SwerveModule[] modules = new SwerveModule[] {
		new SwerveModule(Ports.NW_ENCODER, Ports.NW_TURN_MOTOR, Ports.NW_DRIVE_MOTOR, Constants.NW_ENCODER_OFFSET),
		new SwerveModule(Ports.NE_ENCODER, Ports.NE_TURN_MOTOR, Ports.NE_DRIVE_MOTOR, Constants.NE_ENCODER_OFFSET),
		new SwerveModule(Ports.SW_ENCODER, Ports.SW_TURN_MOTOR, Ports.SW_DRIVE_MOTOR, Constants.SW_ENCODER_OFFSET),
		new SwerveModule(Ports.SE_ENCODER, Ports.SE_TURN_MOTOR, Ports.SE_DRIVE_MOTOR, Constants.SE_ENCODER_OFFSET)
	};
	public SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
			new Translation2d(ROBOT_LENGTH / 2, ROBOT_WIDTH / 2),
			new Translation2d(ROBOT_LENGTH / 2, -ROBOT_WIDTH / 2),
			new Translation2d(-ROBOT_LENGTH / 2, ROBOT_WIDTH / 2),
			new Translation2d(-ROBOT_LENGTH / 2, -ROBOT_WIDTH / 2));

	public void drive(ChassisSpeeds speed) {
		SwerveModuleState[] states = kinematics.toSwerveModuleStates(speed);
		SwerveDriveKinematics.desaturateWheelSpeeds(states, Math.max(SWERVE_MAXSPEED, 5));
		for (int i = 0; i < 4; i++) {
			modules[i].setState(states[i]);
		}
	}
	public void getOffsets(){
		for (SwerveModule module : modules) module.fixOffset();
	}
	public void zero() {
		for (int i = 0; i < 4; i++) {
			modules[i].setState(new SwerveModuleState());
		}
	}
	public void cross(){
		for (int i = 0; i < 4; i++) {
			modules[i].setState(new SwerveModuleState(0,new Rotation2d(45)));
		}
	}

	public void periodic() {
		for (SwerveModule module : modules) module.periodic();
	}
}
