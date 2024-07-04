// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.imgcodecs.Imgcodecs;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ConstantSpeedDrive;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import java.util.function.BooleanSupplier;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import com.revrobotics.*;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  final double AutoShooterPower = 0.9;
  
  Robot robot;
  // Initialize your camera
    public void initializeCamera() {
      UsbCamera camera = CameraServer.startAutomaticCapture();
      camera.setPixelFormat(PixelFormat.kMJPEG);
      camera.setFPS(30);
      camera.setResolution(160, 120);
    }

  private DigitalInput Beam;
  private final SlewRateLimiter limitY = new SlewRateLimiter(4.0);
  private final SlewRateLimiter limitX = new SlewRateLimiter(4.0);
  private final SlewRateLimiter limitRot = new SlewRateLimiter(6.0);
  
      // Load a Choreo trajectory as a PathPlannerPath


  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase;
  public final SendableChooser<Command> AUTO_CHOOSER;
  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driverXbox = new CommandXboxController(2);

  

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer(Robot robot)
  {
    /*if(Robot.Beam.get() == false)
    {
      Robot.controller_driver.setRumble(RumbleType.kBothRumble, 3);
    }*/
   
    drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/neo"));

    this.robot = robot;
    NamedCommands.registerCommand("ShootOnly", ShootOnly());
    NamedCommands.registerCommand("TestAuto", TestAuto());
    NamedCommands.registerCommand("ShootAndBackOut", ShootAndBackOut());
    NamedCommands.registerCommand("Shoot", Shoot());
    NamedCommands.registerCommand("ZeroGyro", zeroGyro());
    NamedCommands.registerCommand("Intake", intake());
    NamedCommands.registerCommand("IndexerUp", IndexerUp());
    NamedCommands.registerCommand("IndexerDown", IndexerDown());
    NamedCommands.registerCommand("ShooterStop", ShooterStop());
    NamedCommands.registerCommand("IntakeStop", intakeStop());
    NamedCommands.registerCommand("IndexerStop", IndexerStop());
    NamedCommands.registerCommand("Rumble", rumbleCommand());
    NamedCommands.registerCommand("RumbleStop", rumbleCommand());


    AUTO_CHOOSER = AutoBuilder.buildAutoChooser();
    // Configure the trigger bindings
    configureBindings();
   
    SmartDashboard.putData("My Auto's", AUTO_CHOOSER);
    
    

    

    AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                                                                   () -> -MathUtil.applyDeadband(driverXbox.getLeftY(),
                                                                                                OperatorConstants.LEFT_Y_DEADBAND),
                                                                   () -> -MathUtil.applyDeadband(driverXbox.getLeftX(),
                                                                                                OperatorConstants.LEFT_X_DEADBAND),
                                                                   () -> -MathUtil.applyDeadband(driverXbox.getRightX(),
                                                                                                OperatorConstants.RIGHT_X_DEADBAND),
                                                                   driverXbox.getHID()::getYButtonPressed,
                                                                   driverXbox.getHID()::getAButtonPressed,
                                                                   driverXbox.getHID()::getXButtonPressed,
                                                                   driverXbox.getHID()::getBButtonPressed);

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
        () -> -MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> -MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -driverXbox.getRightX(),
        () -> -driverXbox.getRightY());

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> -MathUtil.applyDeadband(limitY.calculate(driverXbox.getLeftY()), OperatorConstants.LEFT_Y_DEADBAND),
        () -> -MathUtil.applyDeadband(limitX.calculate(driverXbox.getLeftX()), OperatorConstants.LEFT_X_DEADBAND),
        () -> -limitRot.calculate(driverXbox.getRightX()));

    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> -MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> -MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -driverXbox.getRawAxis(2));

    drivebase.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocity : driveFieldOrientedDirectAngleSim);

        
        

    
  }
  
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`


    driverXbox.leftBumper().whileTrue(new ConstantSpeedDrive(drivebase));
    driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
    driverXbox.b().whileTrue(
        Commands.deferredProxy(() -> drivebase.driveToPose(
                                   new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                              ));
  
    // driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
      

  public Command getAutonomousCommand()
  {
    return AUTO_CHOOSER.getSelected();
  }
  

  
  public Command Shoot() 
  {
    return new InstantCommand(() -> {
      robot.TopShooter.set(-AutoShooterPower);
      robot.BottomShooter.set(-AutoShooterPower);
    });
  }
  
  public Command zeroGyro() 
  {
    return new InstantCommand(() -> {
      Commands.runOnce(drivebase::zeroGyro);
    });
  }

   public Command intake() 
  {
    return new InstantCommand(() -> {
      robot.Intake.set(-Robot.intake_power);
    });
  }

  public Command IndexerUp() 
  {
    return new InstantCommand(() -> {
      robot.Indexer.set(Robot.Indexer_power);
    });
  }
  public Command IndexerDown() 
  {
    return new InstantCommand(() -> {
      robot.Indexer.set(Robot.Indexer_reverse);
    });
  }

  public Command IndexerStop() 
  {
    return new InstantCommand(() -> {
      robot.Indexer.set(Robot.generic_stop);
    });
  }

  public Command intakeStop() 
  {
    return new InstantCommand(() -> {
      robot.Intake.set(Robot.generic_stop);
    });
  }

  public Command ShooterStop() 
  {
    return new InstantCommand(() -> {
      robot.TopShooter.set(Robot.generic_stop);
      robot.BottomShooter.set(Robot.generic_stop);
    });
  }

  public Command rumbleCommand()
  {
    return new InstantCommand(() -> {
      Robot.controller_driver.setRumble(RumbleType.kBothRumble, 2);
    });
  }

  public Command rumbleStopCommand()
  {
    return new InstantCommand(() -> {
      Robot.controller_driver.setRumble(RumbleType.kBothRumble, 0);
    });
  }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
  public Command TestAuto()
  {
    
    PathPlannerPath Forward = PathPlannerPath.fromChoreoTrajectory("Forward");
    Command forwardCommand = AutoBuilder.followPath(Forward);
  
    return new SequentialCommandGroup(
      zeroGyro().withTimeout(2),
      new WaitCommand(2),
      forwardCommand
      
    );
  }

  public Command ShootAndBackOut()
  {
    PathPlannerPath ForwardShoot = PathPlannerPath.fromChoreoTrajectory("Testing");
    Command forwardShootCommand = AutoBuilder.followPath(ForwardShoot);
  
    return new SequentialCommandGroup(
      zeroGyro().withTimeout(1),
      new WaitCommand(1),
      Shoot().withTimeout(1),
      new WaitCommand(3),
      IndexerUp().withTimeout(1),
      new WaitCommand(2),
      forwardShootCommand,
      new WaitCommand(2),
      ShooterStop().withTimeout(1),
      IndexerStop().withTimeout(1),
      intakeStop().withTimeout(1)
      
    );
  }
  public Command ShootOnly()
  {
  
  
    return new SequentialCommandGroup(
      zeroGyro().withTimeout(1),
      new WaitCommand(1),
      Shoot().withTimeout(1),
      new WaitCommand(3),
      IndexerUp().withTimeout(1),
      new WaitCommand(2),
      ShooterStop().withTimeout(1),
      IndexerStop().withTimeout(1),
      intakeStop().withTimeout(1)
      
    );
  }
  
  
  /*public void solid(Color color) {
    if (color != null) {
        for (int i = 0; i < robot.StripLength; i++) {
            robot.ledBuffer.setLED(i, color);
        }
       // robot.leds.setData(robot.ledBuffer);
    }

}
public void solid(double percent, Color color) {
  for (int i = 0; i < MathUtil.clamp(robot.StripLength * percent, 0, robot.StripLength); i++) {
      robot.ledBuffer.setLED(i, color);
  }

}



public Command solidCommand(Color color){
  return new RunCommand(() -> solid(color)).alongWith(new PrintCommand("test")).ignoringDisable(true);
}

public Command solidCommand(double percent, Color color) {
  //System.out.println("Color received: " + color); // Print the color value
  return new RunCommand(() -> solid(percent, color)).alongWith(new PrintCommand("test"));
}*/

/*public Command Rumble()
{
  return new RunCommand(() -> Robot.controller_driver.setRumble(RumbleType.kBothRumble, 3));
}*/

 

    
    
}

