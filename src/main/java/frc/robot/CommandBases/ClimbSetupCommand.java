package frc.robot.CommandBases;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.Constants.FeederConstants;
import frc.robot.generated.Constants.IntakeConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Shooter.FeederSubsystem;
import frc.robot.subsystems.Shooter.IntakeSubsystem;
import frc.robot.subsystems.Shooter.PivotSubsystem;


public class ClimbSetupCommand extends Command{

    private final ClimberSubsystem climber;
    private final LEDSubsystem led;

    public ClimbSetupCommand(ClimberSubsystem climber,LEDSubsystem led) {
        this.climber = climber;
        this.led = led;
        addRequirements(climber,led);
      }

      @Override
  public void initialize() {
    climber.setPosition(-160);
  }

      @Override
      public boolean isFinished() {
        return climber.setUPCHECK();
      }

     @Override
     public void end(boolean interrupted) {
      led.setRGB(170, 51, 106);
     }
}