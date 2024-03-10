package frc.robot.CommandBases;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.Constants.FeederConstants;
import frc.robot.generated.Constants.IntakeConstants;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.AMP.AMPPivotSubsystem;


public class AMPPivotwithSpeed extends Command{

    private final AMPPivotSubsystem pivot;
    private double speed;


    public AMPPivotwithSpeed(AMPPivotSubsystem pivot,double speed) {
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