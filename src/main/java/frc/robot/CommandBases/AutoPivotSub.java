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


public class AutoPivotSub extends Command{

    private final PivotSubsystem pivot;

    public AutoPivotSub(PivotSubsystem pivot) {
        this.pivot = pivot;
        addRequirements(pivot);
      }

      @Override
  public void initialize() {
      pivot.setPosition(25);
    
  }

      @Override
      public boolean isFinished() {
        return true;
      }

     @Override
     public void end(boolean interrupted) {
      pivot.setPosition(25);
    }
}