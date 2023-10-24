package frc.robot.subsystems;

import static frc.robot.constants.Constants.*;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.SwerveModule;

public class SwerveSubsystem {
	SwerveModule[] modules = new SwerveModule[4]; // TODO
	SwerveDriveKinematics kinematics = new SwerveDriveKinematics(/* ... */ );

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
