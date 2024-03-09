package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix.motorcontrol.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.Constants.FeederConstants;

public class FeederSubsystem  extends SubsystemBase{
 
private final WPI_TalonSRX  m_feeder;

private final VelocityVoltage m_voltageVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);

private final NeutralOut m_brake = new NeutralOut();

 Encoder encoder;
    
public FeederSubsystem()
{

       m_feeder = new WPI_TalonSRX(FeederConstants.feeder);

       encoder = new Encoder(FeederConstants.feederEncoderA, FeederConstants.feederEncoderB);


     
}

public void disable()
{
    m_feeder.set(0);
}

public double getFeederRate()
{
  return MathUtil.applyDeadband(encoder.getRate(), 1);
}

public void setVelocity(double desiredRotationsPerSecond)
{
    //m_feeder.setControl(m_voltageVelocity.withVelocity(desiredRotationsPerSecond));
    m_feeder.set(desiredRotationsPerSecond);
  }

public Command highspeed()
{
  return run(() -> this.setVelocity(1));
}

public Command midspeed()
{
  return run(() -> this.setVelocity(.5));
}

public Command slowspeed()
{
  return run(() -> this.setVelocity(.1));
}

public Command withVelocity(double desiredRotationsPerSecond)
{
  return run(() -> this.setVelocity(desiredRotationsPerSecond));
}

public Command withDisable()
{
    return run(() -> this.disable());
}

}
