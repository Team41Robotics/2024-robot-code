package frc.robot.subsystems;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase{}
        /* 
        CANSparkFlex im1 = new CANSparkFlex(20, MotorType.kBrushless);
        CANSparkFlex im2 = new CANSparkFlex(40, MotorType.kBrushless);

        public double im1S, im2S;
        public boolean enabled;

        DoubleTopic lef, rgt, lef_curr, rgt_curr, batt;
        DoublePublisher lef_pub, rgt_pub, lef_curr_pub, rgt_curr_pub, batt_pub;

        public PIDController lef_pid = new PIDController(4e-4, 3e-4, 0);
        public PIDController rgt_pid = new PIDController(4e-4, 3e-4, 0);
        
        public void init(){
                im1.setIdleMode(IdleMode.kCoast);
                im2.setIdleMode(IdleMode.kCoast);
                
                lef = new DoubleTopic(NetworkTableInstance.getDefault().getTable("Flywheel").getTopic("LeftSpeed"));
                rgt = new DoubleTopic(NetworkTableInstance.getDefault().getTable("Flywheel").getTopic("RightSpeed"));
                lef_curr = new DoubleTopic(NetworkTableInstance.getDefault().getTable("Flywheel").getTopic("LeftCurrent"));
                rgt_curr = new DoubleTopic(NetworkTableInstance.getDefault().getTable("Flywheel").getTopic("RightCurrent"));

                batt = new DoubleTopic(NetworkTableInstance.getDefault().getTable("Flywheel").getTopic("Voltage"));
                lef_curr_pub = lef_curr.publish();
                rgt_curr_pub = rgt_curr.publish();
                lef_pub = lef.publish();
                rgt_pub = rgt.publish();
                batt_pub = batt.publish();
        }

        @Override
        public void periodic() { 
                if(lef_pub != null) lef_pub.set(im1.getEncoder().getVelocity());
                if(rgt_pub != null) rgt_pub.set(-im2.getEncoder().getVelocity());

                if(lef_curr_pub != null) lef_curr_pub.set(im1.getOutputCurrent());
                if(rgt_curr_pub != null) rgt_curr_pub.set(im2.getOutputCurrent());

                if(batt_pub != null) batt_pub.set(RobotController.getBatteryVoltage());

        }

        public void runMotors(){
                im1.set(im1S);
                im2.set(im2S);
        }


        public void stopMotors(){
                im1.set(0);
                im2.set(0);
        }

}*/
