// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.cameraserver.CameraServer;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;


public class Robot extends TimedRobot {
  private DifferentialDrive m_myRobot;
  private XboxController m_Stick;
  private CANSparkMax m_leftMotor1;
  private CANSparkMax m_leftMotor2;
  private CANSparkMax m_rightMotor1;
  private CANSparkMax m_rightMotor2;

  private VictorSPX intake;
  private TalonSRX horizontal_feed;
  private VictorSPX turret;
  private TalonSRX vertical_feed;
  private TalonSRX hood;

  private CANSparkMax m_shooter;

  private final Timer m_timer = new Timer();

  private DoubleSolenoid leftIntakePiston;
  private Compressor pcmCompressor;

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  

  @Override
  public void robotInit() {
  /**
   * SPARK MAX controllers are intialized over CAN by constructing a CANSparkMax object
   * 
   * The CAN ID, which can be configured using the SPARK MAX Client, is passed as the
   * first parameter
   * 
   * The motor type is passed as the second parameter. Motor type can either be:
   *  com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless
   *  com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushed
   * 
   * The example below initializes four brushless motors with CAN IDs 1 and 2. Change
   * these parameters to match your setup
   *
   */





    m_leftMotor1= new CANSparkMax(1, MotorType.kBrushless);
    m_rightMotor1 = new CANSparkMax(2, MotorType.kBrushless);
    m_leftMotor2= new CANSparkMax(3, MotorType.kBrushless);
    m_rightMotor2 = new CANSparkMax(4, MotorType.kBrushless);

    m_shooter = new CANSparkMax(10, MotorType.kBrushless);
    

    MotorControllerGroup m_left = new MotorControllerGroup(m_leftMotor1, m_leftMotor2);
    MotorControllerGroup m_right = new MotorControllerGroup(m_rightMotor1, m_rightMotor2 );
    

    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    m_leftMotor1.restoreFactoryDefaults();
    m_rightMotor1.restoreFactoryDefaults();
    m_leftMotor2.restoreFactoryDefaults();
    m_rightMotor2.restoreFactoryDefaults();
    m_shooter.restoreFactoryDefaults();

    m_leftMotor1.setInverted(true);
    m_leftMotor2.setInverted(true);


    m_myRobot = new DifferentialDrive(m_left, m_right);
    m_Stick = new XboxController(3);

    CameraServer.startAutomaticCapture();

    intake = new VictorSPX(5);
    horizontal_feed = new TalonSRX(6);
    turret = new VictorSPX(7);
    hood = new TalonSRX(8);
    vertical_feed = new TalonSRX(9);


    pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
    leftIntakePiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 3, 2);
    leftIntakePiston.set(kForward);

  }

  public void autonomousInit(){
    m_timer.reset();
    m_timer.start();
  }

  public void autonomousPeriodic() {
    // Drive for 6 seconds
    while (m_timer.get() < 15.0) {
      while (m_timer.get() < 7.0){
        //while(m_timer.get() <= 0.9){
          //m_myRobot.arcadeDrive(-0.50, 0);
        //}
        //m_myRobot.stopMotor();
        m_shooter.set(-1);
        while (m_timer.get() > 2.0 && m_timer.get()<6.0){
            vertical_feed.set(ControlMode.PercentOutput, 0.5);
        }
      }
      m_shooter.set(0);
      vertical_feed.set(ControlMode.PercentOutput, 0);
      while (m_timer.get() > 8.0 && m_timer.get() < 11.0){
        m_myRobot.arcadeDrive(-0.50, 0.0);
      };
      SmartDashboard.putNumber("Time", m_timer.get());
      m_myRobot.stopMotor();
    }  
  }

  @Override
  public void teleopPeriodic() {

    m_myRobot.arcadeDrive(-m_Stick.getLeftY(), -m_Stick.getLeftX());

    hood.set(ControlMode.PercentOutput, 0.6*-m_Stick.getRightY());

    turret.set(ControlMode.PercentOutput, -0.5*m_Stick.getRightX());

    m_shooter.set(-m_Stick.getRightTriggerAxis());

    if(m_Stick.getRightTriggerAxis() > 0){
      m_myRobot.arcadeDrive(0, 0);
    }

    if (m_Stick.getYButtonPressed()) {
      leftIntakePiston.toggle();
   }

   if (m_Stick.getRightBumperPressed()){
     pcmCompressor.enableDigital();
   }

   if(m_Stick.getLeftBumperPressed()){
     pcmCompressor.disable();
   }

   if(pcmCompressor.getPressureSwitchValue()){
     pcmCompressor.enableDigital();
   }



    horizontal_feed.set(ControlMode.PercentOutput, -m_Stick.getLeftTriggerAxis());
    intake.set(ControlMode.PercentOutput, -m_Stick.getLeftTriggerAxis());


  while(m_Stick.getXButton())
    vertical_feed.set(ControlMode.PercentOutput,0.45);

  while(m_Stick.getBButton()){
    vertical_feed.set(ControlMode.PercentOutput, -0.3);
    horizontal_feed.set(ControlMode.PercentOutput,0.3);
    intake.set(ControlMode.PercentOutput, 0.3);
   }
  
  while(m_Stick.getAButton()){
    vertical_feed.set(ControlMode.PercentOutput, 0);
  }

  if(leftIntakePiston.get().equals(kForward)){
    intake.set(ControlMode.PercentOutput, 0);
  }

  if(m_Stick.getStartButtonPressed()){
    m_timer.reset();
    m_timer.start();
    while (m_timer.get() <= 0.9){
      m_myRobot.arcadeDrive(-0.5, 0);
    }
    m_myRobot.arcadeDrive(-m_Stick.getLeftY(), -m_Stick.getLeftX());

  }

  }
}