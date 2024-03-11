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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.CommandBases.AMPPivotwithSpeed;
import frc.robot.CommandBases.ElevatorWithSpeed;
import frc.robot.CommandBases.IntakeCommand;
import frc.robot.CommandBases.PivotwithSpeed;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.AMP.AMPFeederSubsystem;
import frc.robot.subsystems.AMP.AMPPivotSubsystem;
import frc.robot.subsystems.AMP.ElevatorSubsystem;
import frc.robot.subsystems.Shooter.FeederSubsystem;
import frc.robot.subsystems.Shooter.IntakeSubsystem;
import frc.robot.subsystems.Shooter.PivotSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class RobotContainer {

 //intake Subsystem
  public final IntakeSubsystem intake = new IntakeSubsystem();

  //shooter Subsystem
  public final ShooterSubsystem shooter = new ShooterSubsystem();

  //Feeder Subsystem
  public final FeederSubsystem feeder = new FeederSubsystem();

  //pivot Subsystem
  public final PivotSubsystem pivot = new PivotSubsystem();
  public final PivotwithSpeed pivotup = new PivotwithSpeed(pivot,.1);
  public final PivotwithSpeed pivotdown = new PivotwithSpeed(pivot,-.1);

  //AMPPivot
  public final AMPPivotSubsystem amppivot = new AMPPivotSubsystem();

  public final AMPPivotwithSpeed amppivotup = new AMPPivotwithSpeed(amppivot,.1);
  public final AMPPivotwithSpeed amppivotdown = new AMPPivotwithSpeed(amppivot,-.1);


  //ELEVATOR
  public final ElevatorSubsystem elevator = new ElevatorSubsystem();

  public final ElevatorWithSpeed elevatorup = new ElevatorWithSpeed(elevator, .1);
  public final ElevatorWithSpeed elevatordown = new ElevatorWithSpeed(elevator, -.1);

  //AMPFEEDER
  public final AMPFeederSubsystem ampfeeder = new AMPFeederSubsystem();
  //Commands
  private final LEDSubsystem led = new LEDSubsystem(0);


  IntakeCommand intakecommand = new IntakeCommand(intake, feeder, led,pivot);

  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController xbox = new CommandXboxController(0); // My joystick
  private final CommandJoystick joystick = new CommandJoystick(1); // My joystick
  
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.RobotCentric robotdrive = new SwerveRequest.RobotCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(xbox.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(xbox.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-xbox.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

      xbox.x().onTrue( drivetrain.applyRequest(() -> robotdrive.withVelocityX(xbox.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(xbox.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-xbox.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));
   
    //drivetrain.registerTelemetry(logger::telemeterize);


//intake
      ////////////////////////
      intake.setDefaultCommand(intake.withDisable());
      //joystick.x().onTrue(intake.slowspeed());

//feeder
      feeder.setDefaultCommand(feeder.withDisable());
      xbox.rightTrigger().whileTrue(feeder.midspeed());


//Intake Commands
      xbox.rightBumper().onTrue(intakecommand);
      xbox.leftBumper().whileTrue(new ParallelCommandGroup(intake.withVelocity(20),feeder.withVelocity(-20),shooter.withDisable()));
      
//Shooter
      shooter.setDefaultCommand(shooter.withDisable());
      xbox.leftTrigger().onTrue(shooter.highspeed());

      //pivot
      pivot.setDefaultCommand(pivot.stop());
     // joystick.pov(0).whileTrue(pivot.slowUp());
     // joystick.pov(180).whileTrue(pivot.slowDown());
       xbox.pov(0).whileTrue(pivotup);
       xbox.pov(180).whileTrue(pivotdown);
       //joystick.pov(90).onTrue(pivot.stop());
      xbox.y().onTrue(pivot.withPosition(24.96));

      SmartDashboard.putData("Autonomous Command", drivetrain.runOnce(() ->  drivetrain.seedFieldRelative()));





//OPERATOR CONTROLS

//AmpFeeder
ampfeeder.setDefaultCommand(ampfeeder.withDisable());
joystick.button(1).whileTrue(ampfeeder.midspeed());
joystick.button(2).whileTrue(ampfeeder.withVelocity(-0.8));
  
//AMPpivot
amppivot.setDefaultCommand(amppivot.holdPosition());
joystick.pov(180).whileTrue(amppivotup);
joystick.pov(0).whileTrue(amppivotdown);

joystick.button(7).onTrue(amppivot.withPosition(44));//intake position
joystick.button(8).onTrue(amppivot.withPosition(3)); //home 

//Elevator
////////////////////////
elevator.setDefaultCommand(elevator.holdPosition());
joystick.button(10).whileTrue(elevatorup);
joystick.button(9).whileTrue(elevatordown);
//joystick.button(12).onTrue(elevator.setUpPosition());
joystick.button(12).onTrue(new ParallelCommandGroup(elevator.setUpPosition(),amppivot.withPosition(27.3)));

joystick.button(11).onTrue(elevator.setHomePosition());


}

  public RobotContainer() {
    configureBindings();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
