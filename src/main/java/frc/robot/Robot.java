// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.XboxController;
import java.io.File;
import java.io.IOException;
import java.util.Optional;
import java.util.OptionalInt;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.units.Time;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import javax.sound.sampled.Port;

import swervelib.parser.SwerveParser;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import com.fasterxml.jackson.annotation.JacksonInject.Value;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot
{
  
  
  final AllianceStationID TS = DriverStation.getRawAllianceStation();
  public static XboxController controller_driver = new XboxController(3);
  
  
  
  CANSparkBase Intake = new CANSparkMax(16, MotorType.kBrushed);
  CANSparkBase LeftArm = new CANSparkMax(19, MotorType.kBrushed);
  CANSparkBase RightArm = new CANSparkMax(20, MotorType.kBrushed);
  CANSparkBase TopShooter = new CANSparkMax(17, MotorType.kBrushless);
  CANSparkBase BottomShooter = new CANSparkMax(18, MotorType.kBrushless);
  CANSparkBase Indexer = new CANSparkMax(21, MotorType.kBrushless);
  static DigitalInput Beam;
  public AddressableLED leds = new AddressableLED(9);
  static final double intake_power = 1;
  static final double intake_stop = 0;
  static final double shooter_power = 0.75;
  static final double shooter_halfpower = 0.375;
  static final double shooter_stop = 0;
  static final double arm_power = 0.5;
  static final double arm_reverse = -0.5;
  static final double arm_stop = 0;
  static final double shooter_reverse = -0.9;
  static final double generic_stop = 0;
  static final double Indexer_power = 1;
  static final double Indexer_reverse = -1;
  private static Robot instance;
  private Command m_autonomousCommand;
  private Command m_BlueDSLCommand;
  private Command m_BlueDSFCommand;
  private Command m_BlueDSRCommand;
  private Command m_RedDSLCommand;
  private Command m_RedDSFCommand;
  private Command m_RedDSRCommand;
  private RobotContainer m_robotContainer;
  private Timer disabledTimer;
  private Timer rumbleTimer;
  private boolean isRumbling;
  final int StripLength = 16;
  
  public AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(StripLength);
  

  public Robot()
  {
     
    instance = this;
  }

  public static Robot getInstance()
  {
    return instance;
  }

  /**
   * This function is run when the robot is first started up and should be used for any initialization code.
   */
  @Override
  public void robotInit()
  {
    
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    Beam = new DigitalInput(0);
    m_robotContainer = new RobotContainer(getInstance());
    m_robotContainer.initializeCamera();
    // Create a timer to disable motor brake a few seconds after disable.  This will let the robot stop
    // immediately when disabled, but then also let it be pushed more 
    disabledTimer = new Timer();
    rumbleTimer = new Timer();
    isRumbling = false;
   
    /*leds.setLength(StripLength);
    leds.start();
    ledBuffer.setLED(1, Color.kWhite);
    leds.setData(ledBuffer);
    // m_robotContainer.solid(100, Color.kBlue);
    
    System.out.println("LED Start is Called");
    */
    
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics that you want ran
   * during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic()
  {
    
    
    
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit()
  {
    m_robotContainer.setMotorBrake(true);
    disabledTimer.reset();
    disabledTimer.start();
  }

  @Override
  public void disabledPeriodic()
  {
    if (disabledTimer.hasElapsed(Constants.DrivebaseConstants.WHEEL_LOCK_TIME))
    {
      m_robotContainer.setMotorBrake(false);
      disabledTimer.stop();
    }
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit()
  {
    
    m_robotContainer.setMotorBrake(true);
    m_robotContainer.getAutonomousCommand().schedule();
    
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic()
  {
   
  }
  

  @Override
  public void teleopInit()
  {
   
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    
    /* 
    for (int i = 0; i < StripLength; i++) {
      ledBuffer.setLED(i, Color.kRed);
  }
  ledBuffer.setLED(2,Color.kBlue);
  leds.setData(ledBuffer);*/

   

    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.setDriveMode();
    m_robotContainer.setMotorBrake(true);
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
public void teleopPeriodic() {

/*if(Beam.get() == false)
{

controller_driver.setRumble(RumbleType.kBothRumble, 2);
}

else
{
controller_driver.setRumble(RumbleType.kBothRumble, 0);
}*/
 
//while(Beam.get() == false){
//m_robotContainer.solidCommand(1, Color.kRed).schedule();
 //for (int i = 0; i < StripLength; i++) {
  //ledBuffer.setLED(i, Color.kRed);
  
//}

  


/*for (int i = 0; i < StripLength; i++) {
  ledBuffer.setLED(i, Color.kBlue);
}
 leds.setData(ledBuffer);
 System.out.println("have mercy");*/
 
 
  

// Other teleop periodic code...
/*-------------------------------------------------------- */ /* Intake */
// intake controls for left bumper and right bumper
if(controller_driver.getLeftTriggerAxis() > 0.7) 
  {
    Indexer.set(Indexer_reverse);
    Intake.set(intake_power);
  }
  else if(controller_driver.getLeftBumper() == true && Beam.get() == true)
  {
    Indexer.set(Indexer_power);
    Intake.set(-intake_power);
  }
  else
  {
    if(controller_driver.getYButton() == true)
  {
    Indexer.set(Indexer_power);
      
      
  }
  else if(controller_driver.getAButton() == true)
  {
      Indexer.set(Indexer_reverse);
    }
    else
    {
       Indexer.set(generic_stop);
    }
    Intake.set(intake_stop);
   
  }
  /*------------------------------------------------------*/ /* Arm's */
  // arm controls for the left and right stick
  if(controller_driver.getLeftY() > 0.4)
  {
    LeftArm.set(arm_reverse);
  }
  else if(controller_driver.getLeftY() < -0.4)
  {
    
    LeftArm.set(arm_power);
  }
  else {LeftArm.set(generic_stop);}

  if(controller_driver.getRightY() > 0.4)
  {
    RightArm.set(arm_power);
  }
  else if(controller_driver.getRightY() < -0.4)
  {
    RightArm.set(arm_reverse);
  }
  else{RightArm.set(generic_stop);}
/*-------------------------------------------------------- */ /* Shooter */
/*setting up shooter controls... */
  if(controller_driver.getRightTriggerAxis() > 0.7)
  {
    TopShooter.set(shooter_power);
    BottomShooter.set(shooter_power);
  }
  else if(controller_driver.getRightBumper() == true)
  {
    TopShooter.set(shooter_reverse);
    BottomShooter.set(shooter_reverse);
  }
  else
  {
    TopShooter.set(generic_stop);
    BottomShooter.set(generic_stop);
  }
  /*-------------------------------------------------------- */ /* Indexer */
  /*setting up indexer controls */
  
/*---------------------------------------------------------- */ /* end of operator controls code, you can find controls for the drive base in RobotContainer.java */
  }
  
  
 



  @Override
  public void testInit()
  {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    try
    {
      new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve"));
    } catch (IOException e)
    {
      throw new RuntimeException(e);
    }
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic()
  {
  }

  /**
   * This function is called once when the robot is first started up.
   */
  @Override
  public void simulationInit()
  {
      // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    
   
    
  }

  /**
   * This function is called periodically whilst in simulation.
   */
  @Override
  public void simulationPeriodic()
  {
    
  }
}
