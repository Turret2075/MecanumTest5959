// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;


import com.studica.frc.AHRS;
/*====================================================
    Red CAN (ID . Dispositivo)
    ==========================
    0 . roboRIO
    
    2 . BackRight
    3 . BackLeft
    4 . FrontRight
    5 . FrontLeft
    8 . Shooter/Intake

======================================================
    Red ENCODERS (PuertoA, PuertoB . Dispositivo)
    0, 1 . FrontLeft
    8, 9 . BackLeft
    4, 5 . FrontRight
    6, 7 . BackRight



 */


public class Robot extends TimedRobot {
  //Variables
  int BackRightID = 2;
  int BackLeftID = 3;
  int FrontRightID = 4;
  int FrontLeftID = 5;

  SparkMax BackRight = new SparkMax(BackRightID, MotorType.kBrushed);
  SparkMax BackLeft = new SparkMax(BackLeftID, MotorType.kBrushed);
  SparkMax FrontRight = new SparkMax(FrontRightID, MotorType.kBrushed);
  SparkMax FrontLeft = new SparkMax(FrontLeftID, MotorType.kBrushed);

  SparkMaxConfig BackRightConfig = new SparkMaxConfig();
  SparkMaxConfig BackLeftConfig = new SparkMaxConfig();
  SparkMaxConfig FrontRightConfig = new SparkMaxConfig();
  SparkMaxConfig FrontLeftConfig = new SparkMaxConfig();

  XboxController ControlCero = new XboxController(0);
  MecanumDrive ChasisMecanum;
  MecanumDriveKinematics xRC_Kinematics;
  MecanumDriveOdometry xRC_Odometry;

  AHRS navx = new AHRS(AHRS.NavXComType.kMXP_SPI);

  Timer kronos = new Timer(); //KORG referencia!

  double RotAuto;
  double chasisVelo;
  double MecanumMove;
  double MecanumStrafe;
  double MecanumRotacion;

  SparkMax Shooter = new SparkMax(9, MotorType.kBrushless);
  SparkMaxConfig ShooterConfig = new SparkMaxConfig();

  Encoder FrontLeftEncoder = new Encoder(0,1,false, Encoder.EncodingType.k4X);
  Encoder BackLeftEncoder = new Encoder(8,9,false, Encoder.EncodingType.k4X);
  Encoder FrontRightEncoder = new Encoder(4,5,true, Encoder.EncodingType.k4X);
  Encoder BackRightEncoder = new Encoder(6,7,true, Encoder.EncodingType.k4X); //SIX SEVEN...?


  // PID por rueda
  PIDController pidFL = new PIDController(0.9, 0.0, 0.0001);
  PIDController pidFR = new PIDController(0.9, 0.0, 0.0001);
  PIDController pidRL = new PIDController(0.9, 0.0, 0.0001);
  PIDController pidRR = new PIDController(0.9, 0.0, 0.0001);

  // Feedforward por rueda (kS, kV, kA) — valores de ejemplo: debes tunear
  SimpleMotorFeedforward ffFL = new SimpleMotorFeedforward(0.3, 2.2, 0.25);
  SimpleMotorFeedforward ffFR = new SimpleMotorFeedforward(0.3, 2.2, 0.25);
  SimpleMotorFeedforward ffRL = new SimpleMotorFeedforward(0.3, 2.2, 0.25);
  SimpleMotorFeedforward ffRR = new SimpleMotorFeedforward(0.3, 2.2, 0.25);

  //Default
  private static final String kCenterAuto = "Auto Centro";
  private static final String kTimerAutoDerecha = "Simple Swipe Derecha";
  private static final String kEncodedAutoDerecha = "Flying Swipe Derecha";
  private static final String kTimerAutoIzquierda = "Simple Swipe Izquierda";
  private static final String kEncodedAutoIzquierda = "Flying Swipe Izquierda";
  private static final String kAutoMuyPro = "No hacer NADA";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  @SuppressWarnings("removal")
  public Robot() {
    //Motors
    BackRightConfig.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit(40);
    BackLeftConfig.inverted(true).idleMode(IdleMode.kBrake).smartCurrentLimit(40);
    FrontRightConfig.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit(40);
    FrontLeftConfig.inverted(true).idleMode(IdleMode.kBrake).smartCurrentLimit(40);

    ShooterConfig.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit(40);

    BackRight.configure(BackRightConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
    BackLeft.configure(BackLeftConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
    FrontRight.configure(FrontRightConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
    FrontLeft.configure(FrontLeftConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
    Shooter.configure(ShooterConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);


    FrontLeftEncoder.setSamplesToAverage(10);
    FrontLeftEncoder.setDistancePerPulse(1.0 / 360 * (Math.PI * 6) * 0.0254); //Conversión
    FrontLeftEncoder.setMinRate(10);
    FrontLeftEncoder.reset();

    BackLeftEncoder.setSamplesToAverage(10);
    BackLeftEncoder.setDistancePerPulse(1.0 / 360 * (Math.PI * 6) * 0.0254); //Conversión
    BackLeftEncoder.setMinRate(10);
    BackLeftEncoder.reset();

    FrontRightEncoder.setSamplesToAverage(10);
    FrontRightEncoder.setDistancePerPulse(1.0 / 360 * (Math.PI * 6) * 0.0254); //Conversión
    FrontRightEncoder.setMinRate(10);
    FrontRightEncoder.reset();

    BackRightEncoder.setSamplesToAverage(10);
    BackRightEncoder.setDistancePerPulse(1.0 / 360 * (Math.PI * 6) * 0.0254); //Conversión
    BackRightEncoder.setMinRate(10);
    BackRightEncoder.reset();

    ChasisMecanum = new MecanumDrive(FrontLeft, BackLeft, FrontRight, BackRight);
    Translation2d frontLeftLocation = new Translation2d(-0.29, 0.29); // Posición del motor delantero izquierdo  metros
    Translation2d rearLeftLocation = new Translation2d(-0.29, -0.29); // Posición del motor trasero izquierdo metros
    Translation2d frontRightLocation = new Translation2d(0.29, 0.29); // Posición del motor delantero derecho en metros
    Translation2d rearRightLocation = new Translation2d(0.29, -0.29); // Posición del motor trasero derecho en metros
    Rotation2d AnguloNavX = Rotation2d.fromDegrees(navx.getAngle());
    Pose2d initialPose = new Pose2d(0,0, AnguloNavX);

    MecanumDriveWheelPositions initialWheelPositions = new MecanumDriveWheelPositions(FrontLeftEncoder.getDistance(),FrontRightEncoder.getDistance(),BackLeftEncoder.getDistance(),BackLeftEncoder.getDistance());

    xRC_Kinematics = new MecanumDriveKinematics(frontLeftLocation, frontRightLocation, rearLeftLocation, rearRightLocation);
    xRC_Odometry = new MecanumDriveOdometry(xRC_Kinematics, AnguloNavX, initialWheelPositions, initialPose);
   

    //Default
    m_chooser.setDefaultOption("Centro Use(less)", kCenterAuto);
    m_chooser.addOption("Simple Swipe Full Trench DERECHA", kTimerAutoDerecha);
    m_chooser.addOption("Simple Swipe ENCODED Derecha", kEncodedAutoDerecha);
    m_chooser.addOption("Simple Swipe Full Trench IZQUIERDA", kTimerAutoIzquierda);
    m_chooser.addOption("Simple Swipe Regresa Bump IZQUIERDA", kEncodedAutoIzquierda);
    m_chooser.addOption("Autonomo Pro", kAutoMuyPro);
    SmartDashboard.putData("Auto choices", m_chooser);
    SmartDashboard.putData("NavX", navx);

  }


  @Override
  public void robotPeriodic() {


        // Actualiza odometría con posiciones en metros
    MecanumDriveWheelPositions wheelPositions = new MecanumDriveWheelPositions(
      FrontLeftEncoder.getDistance(),
      FrontRightEncoder.getDistance(),
      BackLeftEncoder.getDistance(),
      BackRightEncoder.getDistance());

    xRC_Odometry.update(Rotation2d.fromDegrees(navx.getAngle()), wheelPositions);



  // Opcional: publicar valores útiles
    SmartDashboard.putNumber("Pose X (m)", xRC_Odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("Pose Y (m)", xRC_Odometry.getPoseMeters().getY());
    SmartDashboard.putNumber("Heading (deg)", (Rotation2d.fromDegrees(navx.getAngle())).getDegrees());
    SmartDashboard.putData("Chasis", ChasisMecanum);
    SmartDashboard.putNumber("Encoder FrontLeft", FrontLeftEncoder.getDistance());
    SmartDashboard.putNumber("Encoder FrontRight", FrontRightEncoder.getDistance());
    SmartDashboard.putNumber("Encoder BackLeft", BackLeftEncoder.getDistance());
    SmartDashboard.putNumber("Encoder BackRight", BackRightEncoder.getDistance());
  }


  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    navx.reset();

    kronos.start();
    kronos.reset();
    
    FrontLeftEncoder.reset();
    FrontRightEncoder.reset();
    BackLeftEncoder.reset();
    BackRightEncoder.reset();

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    Rotation2d GyroAuto = Rotation2d.fromDegrees(navx.getAngle());
    switch (m_autoSelected) {
      case kTimerAutoDerecha:
        if (kronos.get()<=1.8) {
          ChasisMecanum.driveCartesian(0.4, 0, 0, GyroAuto); //Avanza Trennch
        }
        else if (kronos.get()<=3.8){
          ChasisMecanum.driveCartesian(0, 0, 0.25, GyroAuto); //Gira
          //Desplegar Cajón e Intake
        }
        else if (kronos.get()<=6.5){
          ChasisMecanum.driveCartesian(0, -0.4, 0, GyroAuto); //Recoje FUEL
          //Prende Intake
        }
        else if (kronos.get()<=9.2){
          //Apaga intake en modo FRENO
          ChasisMecanum.driveCartesian(0, 0.4, 0, GyroAuto); //Regresa
        }
        else if (kronos.get()<=11.8){
          ChasisMecanum.driveCartesian(0, 0, -0.2, GyroAuto); //Gira
        }
        else if (kronos.get()<=13.8) {
          ChasisMecanum.driveCartesian(-0.6, 0, 0, GyroAuto); //Regreza a Alliance Zone
        }
        else if (kronos.get()<=14.5){
          ChasisMecanum.driveCartesian(0, 0, -0.3, GyroAuto); //Rota hacia el HUB
        }
        //Disparar ~14-20 fuel
        else{
          Shooter.set(1);
        }
        break;

      case kEncodedAutoDerecha:
        if ((FrontLeftEncoder.getDistance()<=2) || (FrontRightEncoder.getDistance()<=2)){
          ChasisMecanum.driveCartesian(0.6,0,0,GyroAuto);
        }

          if (navx.getAngle()<90){
          ChasisMecanum.driveCartesian(0, 0, 0.3, GyroAuto);
        }
      

          if ((FrontLeftEncoder.getDistance()>0)||(FrontRightEncoder.getDistance()>-2))
        {ChasisMecanum.driveCartesian(-0.5, 0, 0);}
      
  if ((FrontLeftEncoder.getDistance()<2)||(FrontRightEncoder.getDistance()<0))
        {ChasisMecanum.driveCartesian(-0.5, 0, 0);}
      
      else{
          ChasisMecanum.driveCartesian(0, 0, 0,GyroAuto);
        }
      
        break;

      case kTimerAutoIzquierda:
        if (kronos.get()<=1.8) {
          ChasisMecanum.driveCartesian(0.4, 0, 0, GyroAuto); //Avanza Trennch
        }
        else if (kronos.get()<=3.8){
          ChasisMecanum.driveCartesian(0, 0, -0.25, GyroAuto); //Gira
          //Desplegar Cajón e Intake
        }
        else if (kronos.get()<=6.5){
          ChasisMecanum.driveCartesian(0, 0.4, 0, GyroAuto); //Recoje FUEL
          //Prende Intake
        }
        else if (kronos.get()<=9.2){
          //Apaga intake en modo FRENO
          ChasisMecanum.driveCartesian(0, -0.4, 0, GyroAuto); //Regresa
        }
        else if (kronos.get()<=11.8){
          ChasisMecanum.driveCartesian(0, 0, 0.2, GyroAuto); //Gira
        }
        else if (kronos.get()<=13.8) {
          ChasisMecanum.driveCartesian(-0.6, 0, 0, GyroAuto); //Regreza a Alliance Zone
        }
        else if (kronos.get()<=14.5){
          ChasisMecanum.driveCartesian(0, 0, 0.3, GyroAuto); //Rota hacia el HUB
        }
        //Disparar ~14-20 fuel
        else{
          Shooter.set(1);
        }
        break;

      case kEncodedAutoIzquierda:
        break;

      case kCenterAuto: //Autónomo Retroceder y Disparar ~6-8 Fuel
        if (kronos.get()<=2.2) {
          ChasisMecanum.driveCartesian(-0.2, 0, 0);
        }
        //Disparar Fuel
        else if (kronos.get()<=8){
          Shooter.set(1);
        }
        break;

      case kAutoMuyPro:
      default:
      System.out.println("Dejaselo a los pros");
      //Deja que los otros robots hagan sus autos sin estorbar
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    chasisVelo = 1.0;
    FrontLeftEncoder.reset();
    FrontRightEncoder.reset();
    BackLeftEncoder.reset();
    BackRightEncoder.reset();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    MecanumMove = (-ControlCero.getLeftY()) * chasisVelo;
    MecanumStrafe = ControlCero.getLeftX() * chasisVelo;
    MecanumRotacion = ControlCero.getRightX() * chasisVelo;


    Rotation2d AnguloNavX = Rotation2d.fromDegrees(navx.getAngle());
    ChasisMecanum.driveCartesian(MecanumMove, MecanumStrafe, MecanumRotacion, AnguloNavX);

    if (ControlCero.getStartButton() == true) {
      navx.reset();
    }   

    if ((ControlCero.getLeftBumperButton() == true) || (ControlCero.getRightBumperButton() == true)) {
      chasisVelo = 0.5;
    } else {
      chasisVelo = 1;
    }

    if (ControlCero.getYButton() == true){
      Shooter.set(1);
    }
    else if (ControlCero.getXButton() == true){
      Shooter.set(-0.3);
    }
    else{
      Shooter.set(0);
    }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
