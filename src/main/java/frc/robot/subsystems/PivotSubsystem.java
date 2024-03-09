package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.Constants.PivotConstants;
import frc.robot.generated.Constants.ShooterConstants;;

public class PivotSubsystem extends SubsystemBase{

private CANSparkMax m_pviot;
private SparkPIDController m_pidController;
private RelativeEncoder m_encoder;
public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;


public PivotSubsystem()
{
  m_pviot = new CANSparkMax(PivotConstants.pivot, MotorType.kBrushless);
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
   kP = 0.1; 
   kI = 1e-4;
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
}

private void setVelocity(double setPoint)
{
  //m_pidController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
m_pviot.set(setPoint);
}

private void setPosition(double setPoint)
{
  m_pidController.setReference(setPoint, CANSparkMax.ControlType.kPosition);
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
  return runOnce(() -> this.setPosition(setPoint));
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

@Override
public void periodic() {
  // This method will be called once per scheduler run
SmartDashboard.putNumber("Shooter Pivot Encoder", m_encoder.getPosition());

/* 
if(m_encoder.getPosition() < 0 && m_pviot.getAppliedOutput() < 0)//Pivot Limit Home
{
 this.stop();
}
*/

}


}
