package frc.robot.CommandBases;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.Constants.FeederConstants;
import frc.robot.generated.Constants.IntakeConstants;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.AMP.AMPPivotSubsystem;
import frc.robot.subsystems.AMP.ElevatorSubsystem;


public class ElevatorWithSpeed extends Command{

    private final ElevatorSubsystem elevator;
    private double speed;


    public ElevatorWithSpeed(ElevatorSubsystem elevator,double speed) {
        this.elevator = elevator;
        addRequirements(elevator);
      }

      @Override
  public void initialize() {
    elevator.setVelocity(speed);
    
  }

      @Override
      public boolean isFinished() {
       return elevator.LimitChecks();
      }

     @Override
     public void end(boolean interrupted) {
      elevator.setVelocity(0);
      elevator.setPosition(elevator.getEncoder());
     }
}