package frc.robot.commands.drive;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.Util;
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
		ChassisSpeeds speeds = Util.joystickToSpeeds(
				vx, vy, w, right_js.button(1).getAsBoolean(), drive.getPose().getRotation());
		drive.drive(speeds);
	}

	@Override
	public void execute() {
		if (ds.button(2).getAsBoolean()) {
			run(0, 0, 0);
		} else {
			run(vx_sup.getAsDouble(), vy_sup.getAsDouble(), w_sup.getAsDouble());
		}
	}
}
