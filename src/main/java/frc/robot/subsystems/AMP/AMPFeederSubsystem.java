package frc.robot.subsystems.AMP;

import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix.motorcontrol.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.Constants.AMPFeederConstants;

public class AMPFeederSubsystem  extends SubsystemBase{
 
private final VictorSPX m_feeder;

private final VelocityVoltage m_voltageVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);

private final NeutralOut m_brake = new NeutralOut();

 Encoder encoder;
    
public AMPFeederSubsystem()
{
       m_feeder = new VictorSPX(AMPFeederConstants.feeder);
      // encoder = new Encoder(AMPFeederConstants.feederEncoderA, AMPFeederConstants.feederEncoderB);     
}

public void disable()
{
    m_feeder.set(ControlMode.PercentOutput,0);
}

public double getFeederRate()
{
  return MathUtil.applyDeadband(encoder.getRate(), 1);
}

public void setVelocity(double desiredRotationsPerSecond)
{
    //m_feeder.setControl(m_voltageVelocity.withVelocity(desiredRotationsPerSecond));
    m_feeder.set(ControlMode.PercentOutput,desiredRotationsPerSecond);
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
