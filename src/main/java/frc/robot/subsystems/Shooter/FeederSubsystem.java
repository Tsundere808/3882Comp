package frc.robot.subsystems.Shooter;

import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix.motorcontrol.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.Constants.FeederConstants;

public class FeederSubsystem  extends SubsystemBase{
 
private final WPI_TalonSRX  m_feeder;

private final VelocityVoltage m_voltageVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);

private final DigitalInput intakeLine;

private   Debouncer m_debouncer = new Debouncer(0.06, Debouncer.DebounceType.kBoth);

 Encoder encoder;

 private ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
private GenericEntry shooterspeed =
      tab.add("Feeder Speed", 0)
         .getEntry();
private GenericEntry shooterVoltage =
      tab.add("Feeder Voltage", 0)
         .getEntry();

private GenericEntry feedercheck =
      tab.add("Feeder Note Check", false)
         .getEntry();
    
public FeederSubsystem()
{

       m_feeder = new WPI_TalonSRX(FeederConstants.feeder);
       intakeLine = new DigitalInput(1);

     
}

public boolean noteCheck()
{
  return m_debouncer.calculate(intakeLine.get());
}

public void disable()
{
    m_feeder.set(0);
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

@Override
public void periodic() {
  // This method will be called once per scheduler run
SmartDashboard.putBoolean("FEEDER NOTE CHECK", intakeLine.get());


shooterVoltage.setDouble(m_feeder.getMotorOutputVoltage());
shooterspeed.setDouble(m_feeder.getMotorOutputPercent());
feedercheck.setBoolean(intakeLine.get());
}


}
