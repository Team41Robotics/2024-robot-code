package frc.robot.commands.Shooter;

import static frc.robot.RobotContainer.shooter;
import static frc.robot.RobotContainer.drive;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.Constants;
import edu.wpi.first.wpilibj2.command.Command;
public class LaunchRing extends Command {
	
			public LaunchRing(){
					addRequirements(shooter);
					addRequirements(drive);
			}

			@Override
			public void execute(){
					//shooter.setAngle(calculateAngle());
					shooter.setSpeed(0.6);
					shooter.runMotors();
			}

			public double calculateAngle(){

					double robotX = drive.getPose().getX();
					double distance = Math.sqrt(Math.pow((180 - robotX),2) + Math.pow(180 - drive.getPose().getY(),2));

					double y = 180-Units.inchesToMeters(Constants.SHOOTER_HEIGHT);

					return Math.atan((y/distance));

			}
	
}
