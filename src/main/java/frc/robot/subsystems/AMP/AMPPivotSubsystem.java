package frc.robot.subsystems.AMP;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.Constants.AMPPivotConstants;

public class AMPPivotSubsystem extends SubsystemBase{

private CANSparkMax m_pviot;
private SparkPIDController m_pidController;
private RelativeEncoder m_encoder;
public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;


private ShuffleboardTab tab = Shuffleboard.getTab("AMP");
private GenericEntry aPivotEncoder =
      tab.add("aPivot Encoder", 0)
         .getEntry();

private GenericEntry aPivotVoltage =
      tab.add("aPivot aPivotVoltage", 0)
         .getEntry();

public AMPPivotSubsystem()
{
  m_pviot = new CANSparkMax(AMPPivotConstants.pivot, MotorType.kBrushless);
  m_pviot.restoreFactoryDefaults();
  /**
     * In order to use PID functionality for a controller, a SparkPIDController object
     * is constructed by calling the getPIDController() method on an existing
     * CANSparkMax object
     */
  m_pidController = m_pviot.getPIDController();

   // Encoder object created to display position values
   m_encoder = m_pviot.getEncoder();

   // PID coefficients
   kP = 0.40; 
   kI = 0.02;
   kD = 1; 
   kIz = 0; 
   kFF = 0; 
   kMaxOutput = 1; 
   kMinOutput = -1;

   // set PID coefficients
   m_pidController.setP(kP);
   m_pidController.setI(kI);
   m_pidController.setD(kD);
   m_pidController.setIZone(kIz);
   m_pidController.setFF(kFF);
   m_pidController.setOutputRange(kMinOutput, kMaxOutput);

   m_encoder.setPosition(0);
}

public void setVelocity(double setPoint)
{
         SmartDashboard.putNumber("Amp Pivot Encoder", m_encoder.getPosition());

  //m_pidController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
m_pviot.set(setPoint);
}

private void setPosition(double setPoint)
{
  m_pidController.setReference(setPoint, CANSparkMax.ControlType.kPosition);
}

public double getEncoderPosition()
{
  return m_encoder.getPosition();
}

public Command withCalculatedPosition(double distance)
{
  //height / distance
  //feet
  double c_setPoint = Math.atan( 10 / distance);
  return runOnce(() -> this.setPosition(c_setPoint));
}

public Command withPosition(double setPoint)
{
  return run(() -> this.setPosition(setPoint));
}

public Command holdPosition()
{
  return run(() -> this.setPosition(this.getEncoderPosition()));
}

public Command high()
{
  return runOnce(() -> this.setPosition(80));
}

public Command mid()
{
  return runOnce(() -> this.setPosition(50));
}

public Command low()
{
  return runOnce(() -> this.setPosition(20));
}

public Command slowUp()
{
  return run(() -> this.setVelocity(.1));
}

public Command slowDown()
{
  return run(() -> this.setVelocity(-.1));
}

public Command stop()
{
  return run(() -> this.setVelocity(0));
}

public boolean LimitChecks()
{
return ((m_encoder.getPosition() < 1 && m_pviot.getAppliedOutput() < 0) || (m_encoder.getPosition() > 43 && m_pviot.getAppliedOutput() > 0));
}

 @Override
public void periodic() {
  // This method will be called once per scheduler run


aPivotEncoder.setDouble( m_encoder.getPosition());
aPivotVoltage.setDouble(m_pviot.getAppliedOutput());

SmartDashboard.putNumber("AMP Pivot Encoder", m_encoder.getPosition());
SmartDashboard.putNumber("AMP Pivot Appied Voltage", m_pviot.getAppliedOutput() );

}

}
