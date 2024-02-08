package frc.robot.commands;

import static frc.robot.RobotContainer.*;
import static frc.robot.constants.Constants.*;
import static java.lang.Math.*;

import edu.wpi.first.wpilibj2.command.Command;

import static frc.robot.RobotContainer.drive;
import static frc.robot.RobotContainer.photon;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

public class FaceSpeaker extends Command{

        PIDController wPID = new PIDController(E, ROBOT_LENGTH, E);

        public FaceSpeaker(){
                addRequirements(drive);
                wPID.enableContinuousInput(0, Math.PI * 2);
        }

        @Override
        public void execute(){
                photon.switchMode(1);
                Pose2d currentPose = drive.getPose();

                double currentRotation = currentPose.getRotation().getRadians();
                Transform2d targetTransform = new Transform2d(null, null);


        }
}