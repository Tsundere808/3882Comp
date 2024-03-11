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


public class AutoShoot extends Command{

    private final FeederSubsystem feeder;
    private final ShooterSubsystem shooter;


    public AutoShoot(FeederSubsystem feeder,ShooterSubsystem shooter) {
        this.feeder = feeder;
        this.shooter =  shooter;
        addRequirements(shooter,feeder);
      }

      @Override
  public void initialize() {
    shooter.setVelocity(100);
  }

      @Override
      public boolean isFinished() {
        return shooter.getSpeed() < -70;
      }

     @Override
     public void end(boolean interrupted) {
    }
}