package frc.robot.commands;

import static frc.robot.RobotContainer.*;
import static frc.robot.constants.Constants.*;
import static java.lang.Math.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

public class DefaultDrive extends Command {
	DoubleSupplier vx_sup, vy_sup, w_sup;

	public DefaultDrive(DoubleSupplier vx_sup, DoubleSupplier vy_sup, DoubleSupplier w_sup) {
		addRequirements(drive);
		this.vx_sup = vx_sup;
		this.vy_sup = vy_sup;
		this.w_sup = w_sup;
	}

	public void run(double vx, double vy, double w) {
		double mag = Math.hypot(vx, vy);
		double ma2 = MathUtil.applyDeadband(mag, 0.1);
		double theta = Math.atan2(vy, vx);
		double sign = (DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Blue) ? 1.0 : -1.0);
		// drive.drive(new ChassisSpeeds(
		// 		cos(theta) * ma2 * SWERVE_MAXSPEED * SPEED_MULT,
		// 		sin(theta) * ma2 * SWERVE_MAXSPEED * SPEED_MULT,
		// 		MathUtil.applyDeadband(w, 0.1) * 2.5));
		drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
				cos(theta) * ma2 * SWERVE_MAXSPEED * SPEED_MULT * sign,
				sin(theta) * ma2 * SWERVE_MAXSPEED * SPEED_MULT * sign,
				MathUtil.applyDeadband(w, 0.1) * 2.5,
				drive.getPose().getRotation()));
	}

	@Override
	public void execute() {
		run(vx_sup.getAsDouble(), vy_sup.getAsDouble(), w_sup.getAsDouble());
	}
}
