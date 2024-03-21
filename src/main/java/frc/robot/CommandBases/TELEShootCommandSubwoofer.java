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
import frc.robot.subsystems.Shooter.ShooterSubsystem;


public class TELEShootCommandSubwoofer extends Command{

    private final FeederSubsystem feeder;
    private final PivotSubsystem pivot;
    private final ShooterSubsystem shooter;

    private final LEDSubsystem led;

    public TELEShootCommandSubwoofer(ShooterSubsystem shooter,FeederSubsystem feeder, LEDSubsystem led, PivotSubsystem pivot) {
        this.shooter = shooter;
        this.feeder = feeder;
        this.pivot = pivot;
        this.led = led;
        addRequirements(pivot,shooter,feeder,led);
      }

      @Override
  public void initialize() {
    led.setYELLOW();
    shooter.setVelocity(100);
    pivot.subwooferPosition();    
  }

  @Override
  public void execute()
  {
    shooter.setVelocity(100);
    if(shooter.getSpeed()  < -70 && pivot.getPosition() > 24){
      led.setBLUE();
    }
  }

      @Override
      public boolean isFinished() {
       return !feeder.noteCheck();
      }

     @Override
     public void end(boolean interrupted) {
      feeder.setVelocity(0);
      shooter.setVelocity(0);
      led.setRED();
     }
}