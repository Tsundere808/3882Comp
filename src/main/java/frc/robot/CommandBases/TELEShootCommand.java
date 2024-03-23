package frc.robot.CommandBases;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.Constants.FeederConstants;
import frc.robot.generated.Constants.IntakeConstants;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Shooter.FeederSubsystem;
import frc.robot.subsystems.Shooter.IntakeSubsystem;
import frc.robot.subsystems.Shooter.PivotSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;


public class TELEShootCommand extends Command{

    private final FeederSubsystem feeder;
    private final ShooterSubsystem shooter;

    private final Timer timer = new Timer();

    private final LEDSubsystem led;

    public TELEShootCommand(ShooterSubsystem shooter,FeederSubsystem feeder, LEDSubsystem led) {
        this.shooter = shooter;
        this.feeder = feeder;
        this.led = led;
        addRequirements(shooter,feeder,led);
      }

      @Override
  public void initialize() {
    led.setBLUE();
    shooter.setVelocity(100);
    timer.reset();
  }

  @Override
  public void execute()
  {
    shooter.setVelocity(100);
    if(shooter.getSpeed()  < -70){
      feeder.setVelocity(.32);
      timer.start();

    }
  }

      @Override
      public boolean isFinished() {
        if(timer.get() > 1.4){
        return true;
        }
        return false;
      }

     @Override
     public void end(boolean interrupted) {
      feeder.setVelocity(0);
      shooter.setVelocity(0);
      led.setRED();
     }
}