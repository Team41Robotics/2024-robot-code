package frc.robot.commands;

import static frc.robot.RobotContainer.*;
import static frc.robot.constants.Constants.*;
import static java.lang.Math.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DefaultDrive extends CommandBase {

	public DefaultDrive() {
		addRequirements(drive);
	}

        @Override
	public void execute() {
                double mag = left_js.getMagnitude();
                double ma2 = MathUtil.applyDeadband(mag, 0.1);
		drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
				// MathUtil.applyDeadband(controller.getLeftX(), 0.1) * SWERVE_MAXSPEED,
				// MathUtil.applyDeadband(-controller.getLeftY(), 0.1) * SWERVE_MAXSPEED,
				// MathUtil.applyDeadband(controller.getRightX(), 0.1),
				// MathUtil.applyDeadband(left_js.getRawAxis(0), 0.1) * SWERVE_MAXSPEED,
				// MathUtil.applyDeadband(-left_js.getRawAxis(1), 0.1) * SWERVE_MAXSPEED,
                                cos(left_js.getDirectionRadians()) * ma2*SWERVE_MAXSPEED,
                                sin(left_js.getDirectionRadians()) * ma2*SWERVE_MAXSPEED,
				MathUtil.applyDeadband(right_js.getRawAxis(0), 0.1),
				new Rotation2d(-imu.yaw())));
	}
}
