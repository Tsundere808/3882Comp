package frc.robot.CommandBases;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
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
        addRequirements(pivot,intake,feeder,led);
      }

      @Override
  public void initialize() {
    led.setRED();
    feeder.setVelocity(.32);
    intake.setVelocity(-50);
    pivot.intakePosition();
    led.setRED();
    
  }

  @Override
  public void execute()
  {
    feeder.setVelocity(.32);
    intake.setVelocity(-50);
  }

      @Override
      public boolean isFinished() {
       return feeder.noteCheck();
      }

     @Override
     public void end(boolean interrupted) {
      feeder.setVelocity(0);
      intake.setVelocity(0);
      pivot.setVelocity(0);
      if(!interrupted)
      {
        led.setGREEN();
        LimelightHelpers.setLEDMode_ForceBlink("limelight-lunas");
      }
        LimelightHelpers.setLEDMode_ForceOff("limelight-lunas");

    }
}