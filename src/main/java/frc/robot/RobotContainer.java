// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.CommandBases.AMPPivotwithSpeed;
import frc.robot.CommandBases.AUTOShootCommandIntakePos;
import frc.robot.CommandBases.AutoPivotSub;
import frc.robot.CommandBases.AutoShoot;
import frc.robot.CommandBases.ElevatorWithSpeed;
import frc.robot.CommandBases.FeederShot;
import frc.robot.CommandBases.IntakeCommand;
import frc.robot.CommandBases.PivotwithSpeed;
import frc.robot.CommandBases.AUTOShootCommandSubwoofer;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.AMP.AMPFeederSubsystem;
import frc.robot.subsystems.AMP.AMPPivotSubsystem;
import frc.robot.subsystems.AMP.ElevatorSubsystem;
import frc.robot.subsystems.Shooter.FeederSubsystem;
import frc.robot.subsystems.Shooter.IntakeSubsystem;
import frc.robot.subsystems.Shooter.PivotSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class RobotContainer {


  private final SendableChooser<Command> autoChooser;

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


  //Climber
  public final ClimberSubsystem climber = new ClimberSubsystem();

  //Commands
  private final LEDSubsystem led = new LEDSubsystem(0);


  IntakeCommand intakecommand = new IntakeCommand(intake, feeder, led,pivot);
  //AutoPivotSub autoPivotSub = new AutoPivotSub(pivot);
  AutoShoot autoshoot = new AutoShoot(feeder, shooter);
  AUTOShootCommandIntakePos autoshootintakepos = new AUTOShootCommandIntakePos(shooter,feeder,led);

  FeederShot feederShot = new FeederShot(feeder);

  AUTOShootCommandSubwoofer shootCommand = new AUTOShootCommandSubwoofer(shooter,feeder,led,pivot);
  
  SequentialCommandGroup SHOOTAUTO = new SequentialCommandGroup(autoshoot,feederShot,new WaitCommand(.5));

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

      xbox.rightTrigger().onTrue(intakecommand);
      //xbox.b().onTrue(shooter.slowspeed());

      
      //pivot
      pivot.setDefaultCommand(pivot.holdPosition());
     // joystick.pov(0).whileTrue(pivot.slowUp());
     // joystick.pov(180).whileTrue(pivot.slowDown());
       xbox.pov(0).whileTrue(pivotup);
       xbox.pov(180).whileTrue(pivotdown);
       //joystick.pov(90).onTrue(pivot.stop());
       xbox.y().onTrue(pivot.withPosition(24.96));
       xbox.b().onTrue(pivot.lightlightAutoAim(LimelightHelpers.getTY("limelight-lunas")));
       xbox.b().onTrue(led.setPink(LimelightHelpers.getTV("limelight-lunas")));
      SmartDashboard.putData("Autonomous Command", drivetrain.runOnce(() ->  drivetrain.seedFieldRelative()));






//OPERATOR CONTROLS

//Shoot Command
//joystick.button(11).onTrue(new SequentialCommandGroup(pivot.withPosition(30.0),shooter.midspeed()));

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
joystick.button(12).onTrue(elevator.setUpPosition());
  //new ParallelCommandGroup(elevator.setUpPosition(),amppivot.withPosition(27.3)));

joystick.button(11).onTrue(elevator.setHomePosition());

//Climber
climber.setDefaultCommand(climber.stop());
joystick.button(3).whileTrue(climber.slowUp());

//joystick.axisGreaterThan(3,0.9).onTrue(climber.setUpPosition());
//joystick.axisLessThan(3,-0.9).onTrue(climber.ClimbedPosition());

//xbox.leftBumper().onTrue(climber.stop());

}

  public RobotContainer() {


  NamedCommands.registerCommand("setFieldRelative",drivetrain.runOnce(() ->  drivetrain.seedFieldRelative()));
  NamedCommands.registerCommand("startIntake", intakecommand);
  //NamedCommands.registerCommand("SubwooferPivot",autoPivotSub);
  NamedCommands.registerCommand("FeederShoot", feederShot);
  NamedCommands.registerCommand("AutoShoot",autoshoot );
  NamedCommands.registerCommand("PivotShot", autoshootintakepos);
  NamedCommands.registerCommand("pivothold", new InstantCommand(() -> pivot.setPosition(pivot.getPosition())));

  //12 feet - 37.84
  //Middle note - 34.25
  //121inch stage - 35.1
  //69inch - 28.88 
  NamedCommands.registerCommand("ShootSubwoofer", shootCommand);
//SHOOTAUTO
  //shootCommand
//new WaitCommand(1),new ParallelCommandGroup(intake.withVelocity(0),feeder.withVelocity(0),shooter.withDisable())
  autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
  SmartDashboard.putData("Auto Mode", autoChooser);
    configureBindings();
    SmartDashboard.putData(CommandScheduler.getInstance());

  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
