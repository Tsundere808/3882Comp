package frc.robot.CommandBases;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.Constants.FeederConstants;
import frc.robot.generated.Constants.IntakeConstants;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Shooter.FeederSubsystem;
import frc.robot.subsystems.Shooter.IntakeSubsystem;
import frc.robot.subsystems.Shooter.PivotSubsystem;


public class PivotwithSpeed extends Command{

    private final PivotSubsystem pivot;
    private double speed;


    public PivotwithSpeed(PivotSubsystem pivot,double speed) {
        this.pivot = pivot;
        this.speed = speed;
        addRequirements(pivot);
      }

      @Override
  public void initialize() {
    pivot.setVelocity(speed);
    
  }

      @Override
      public boolean isFinished() {
       return pivot.LimitChecks();
      }

     @Override
     public void end(boolean interrupted) {
      pivot.setVelocity(0);
     }
}