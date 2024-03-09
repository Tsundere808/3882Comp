// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {

 //intake Subsystem
  public final IntakeSubsystem intake = new IntakeSubsystem();

  //shooter Subsystem
  public final ShooterSubsystem shooter = new ShooterSubsystem();

  //Feeder Subsystem
  public final FeederSubsystem feeder = new FeederSubsystem();

  //pivot Subsystem
  public final PivotSubsystem pivot = new PivotSubsystem();



  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

   
    drivetrain.registerTelemetry(logger::telemeterize);


//intake
      ////////////////////////
      intake.setDefaultCommand(intake.withDisable());
      joystick.x().onTrue(intake.slowspeed());
      joystick.y().onTrue(intake.withVelocity(20));
      joystick.a().onTrue(intake.withDisable());

      
 

      //Shooter
      shooter.setDefaultCommand(shooter.withDisable());
      joystick.rightBumper().onTrue(shooter.slowspeed());
      //xbox.rightTrigger().onTrue(shooter.highspeed());
      joystick.rightTrigger().onTrue(shooter.highspeed());
      joystick.leftTrigger().onTrue(shooter.withDisable());

      
       
      //feeder
      feeder.setDefaultCommand(feeder.withDisable());
      joystick.pov(270).onTrue(feeder.midspeed());
      //xbox.pov(270).onTrue(feeder.withVelocity(maxSpeed.getDouble(0)));
      joystick.leftBumper().onTrue(feeder.withDisable());
      joystick.b().onTrue(feeder.withVelocity(-.1));

   

      //pivot
      pivot.setDefaultCommand(pivot.stop());
      joystick.pov(0).whileTrue(pivot.slowUp());
      joystick.pov(180).whileTrue(pivot.slowDown());
      joystick.pov(90).onTrue(pivot.stop());
      

  }

  public RobotContainer() {
    configureBindings();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
