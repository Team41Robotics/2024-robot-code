package frc.robot.commands.elevator;

import static frc.robot.RobotContainer.elevator;

import edu.wpi.first.wpilibj2.command.Command;

public class RunElevator extends Command{
        
        double speed = 0;
        double maxVel1 = 0;
        double maxVel2 = 0;

        public RunElevator(double s){
                addRequirements(elevator);
                this.speed = s;
        }

        @Override
        public void initialize(){
                maxVel1 = 0;
                maxVel2 = 0;
                elevator.setMotor(speed);
        }

        @Override 
        public void execute(){
                //maxVel1 = max(maxVel1, (elevator.climberMotor1.getVelocity()));
                // I have no idea what im doing with this lmao

        }

}
