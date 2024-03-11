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


public class FeederShot extends Command{

    private final FeederSubsystem feeder;

    public FeederShot(FeederSubsystem feeder) {
        this.feeder = feeder;
        addRequirements(feeder);
      }

      @Override
  public void initialize() {
    feeder.setVelocity(.32);
    
  }

      @Override
      public boolean isFinished() {
       return true;
      }

     @Override
     public void end(boolean interrupted) {

     }
}