package frc.robot.commands.drive;

import static frc.robot.RobotContainer.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

public class FaceSpeaker extends Command {
	private PIDController wPID = new PIDController(5, 0, 0);
	public double targetX = Units.inchesToMeters(
			(DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red)) ? -1.5 : 652.3);
	public double targetY = Units.inchesToMeters(218.42);

	public FaceSpeaker() {
		addRequirements(drive);
		wPID.setTolerance(Units.degreesToRadians(1));
		wPID.enableContinuousInput(0, Math.PI * 2);
	}

	@Override
	public void execute() {

		Pose2d currentPose = drive.getPose();

		double cX = currentPose.getX();
		double cY = currentPose.getY();

		double dx = targetX - cX;
		double dy = targetY - cY;

		double targetRotation = Math.atan(dy / dx);
		double currentRotation = currentPose.getRotation().getRadians();
		wPID.setSetpoint(targetRotation);
		// System.out.println("dx: " + dx + " dy: " + dy + "theta: " + targetRotation + " Current: " + currentRotation);
		drive.drive(new ChassisSpeeds(0, 0, wPID.calculate(currentRotation)));
	}

	@Override
	public boolean isFinished() {
		return wPID.atSetpoint();
	}
}
