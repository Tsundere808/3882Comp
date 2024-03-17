package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase{

private CANSparkFlex m_climber;
private SparkPIDController c_pidController;
private RelativeEncoder c_encoder;
public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;


public ClimberSubsystem()
{
  m_climber = new CANSparkFlex(ClimberConstants.climber, MotorType.kBrushless);
  m_climber.restoreFactoryDefaults();
  m_climber.setIdleMode(IdleMode.kBrake);
  /**
     * In order to use PID functionality for a controller, a SparkPIDController object
     * is constructed by calling the getPIDController() method on an existing
     * CANSparkMax object
     */
  c_pidController = m_climber.getPIDController();

   // Encoder object created to display position values
   c_encoder = m_climber.getEncoder();

   // PID coefficients
   kP = 0.03; 
   kI = 0;
   kD = 0; 
   kIz = 0; 
   kFF = 0.000015; 
   kMaxOutput = 1; 
   kMinOutput = -1;
   maxRPM = 5700;

   // set PID coefficients
   c_pidController.setP(kP);
   c_pidController.setI(kI);
   c_pidController.setD(kD);
   c_pidController.setIZone(kIz);
   c_pidController.setFF(kFF);
   c_pidController.setOutputRange(kMinOutput, kMaxOutput);

}

private void resetEnc()
{
  c_encoder.setPosition(0);
}

private void setVelocity(double setPoint)
{
  m_climber.set(-setPoint);
       SmartDashboard.putNumber("Right Drive Encoder", c_encoder.getPosition());
  //l_pidController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
}

private void setPosition(double setPoint)
{
  c_pidController.setReference(setPoint, CANSparkMax.ControlType.kPosition);
}

public Command withVelocity(double setPoint)
{
  return runOnce(() -> this.setVelocity(setPoint));
}

public Command slowUp()
{
  return run(() -> this.setVelocity(.8));
}


public Command slowDown()
{
  return run(() -> this.setVelocity(-.1));
}


public Command stop()
{
  return run(() -> this.setVelocity(0));
}


public Command withPosition(double setPoint)
{
  return run(() -> this.setPosition(setPoint));
}


public Command setHomePosition()
{
  return run(() -> this.setPosition(0)); //need to find
}


public Command setUpPosition()
{
  return run(() -> this.setPosition(-61)); // need to find
}

public Command ClimbedPosition()
{
  return run(() -> this.setPosition(-100)); // need to find
}

@Override
public void periodic() {

// This method will be called once per scheduler run
SmartDashboard.putNumber("Cliber Pivot Encoder", c_encoder.getPosition());

}

}
