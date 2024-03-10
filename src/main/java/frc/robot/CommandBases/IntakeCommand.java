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


public class IntakeCommand extends Command{

    private final IntakeSubsystem intake;
    private final FeederSubsystem feeder;
    private final PivotSubsystem pivot;

    private final LEDSubsystem led;

    public IntakeCommand(IntakeSubsystem intake,FeederSubsystem feeder, LEDSubsystem led, PivotSubsystem pivot) {
        this.intake = intake;
        this.feeder = feeder;
        this.pivot = pivot;
        this.led = led;
        addRequirements(intake,feeder,led);
      }

      @Override
  public void initialize() {
    feeder.setVelocity(.3);
    intake.setVelocity(-15);
    pivot.intakePosition();
    
  }

  @Override
  public void execute()
  {
    pivot.intakePosition();
  }

      @Override
      public boolean isFinished() {
       return feeder.noteCheck();
      }

     @Override
     public void end(boolean interrupted) {
      feeder.setVelocity(0);
      intake.setVelocity(0);
     }
}