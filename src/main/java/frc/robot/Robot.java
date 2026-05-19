// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//Code by Turret2075, this is my first FRC Java code.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;


import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;



import com.studica.frc.AHRS;
/*====================================================
    Red CAN (ID . Dispositivo)
    ==========================
    0 . roboRIO
    
    2 . RearRight
    3 . RearLeft
    4 . FrontRight
    5 . FrontLeft
    8 . Shooter/Intake
    9 . Pivot/Elevador

======================================================
    Red ENCODERS (PuertoA, PuertoB . Dispositivo)
    0, 1 . FrontLeft
    8, 9 . RearLeft
    4, 5 . FrontRight
    6, 7 . RearRight



 */


public class Robot extends TimedRobot {
  //Variables
  int RearRightID = 2;
  int RearLeftID = 3;
  int FrontRightID = 4;
  int FrontLeftID = 5;
  double kMaxSpeedWheel = 6.0;

  SparkMax RearRight = new SparkMax(RearRightID, MotorType.kBrushed);
  SparkMax RearLeft = new SparkMax(RearLeftID, MotorType.kBrushed);
  SparkMax FrontRight = new SparkMax(FrontRightID, MotorType.kBrushed);
  SparkMax FrontLeft = new SparkMax(FrontLeftID, MotorType.kBrushed);

  SparkMaxConfig RearRightConfig = new SparkMaxConfig();
  SparkMaxConfig RearLeftConfig = new SparkMaxConfig();
  SparkMaxConfig FrontRightConfig = new SparkMaxConfig();
  SparkMaxConfig FrontLeftConfig = new SparkMaxConfig();

  XboxController ControlCero = new XboxController(0);
  MecanumDrive ChasisMecanum;
  MecanumDriveKinematics xRC_Kinematics;
  MecanumDriveOdometry xRC_Odometry;

  AHRS navx = new AHRS(AHRS.NavXComType.kMXP_SPI);
  Field2d canchaREBUILT = new Field2d();
  Timer kronos = new Timer(); //KORG referencia!

  double RotAuto;
  double xRC_SlowMode;
  double MecanumMove;
  double MecanumStrafe;
  double MecanumRotacionRAW;
  double MecanumRotacionPID;
  double AngleTarget;
  
  //Start Coordinates
  double StartInX = 4.525;
  double StartInY = 0.650;

  //Blue Hub Pose
  Pose2d BlueHubPose = new Pose2d(4.5, 4.035, new Rotation2d());

  SparkMax Shooter = new SparkMax(9, MotorType.kBrushless);
  SparkMax Elevator = new SparkMax(10, MotorType.kBrushless);
  SparkMaxConfig ShooterConfig = new SparkMaxConfig();
  SparkMaxConfig ElevatorConfig = new SparkMaxConfig();

  Encoder FrontLeftEncoder = new Encoder(0,1,false, Encoder.EncodingType.k4X);
  Encoder RearLeftEncoder = new Encoder(8,9,false, Encoder.EncodingType.k4X);
  Encoder FrontRightEncoder = new Encoder(4,5,true, Encoder.EncodingType.k4X);
  Encoder RearRightEncoder = new Encoder(6,7,true, Encoder.EncodingType.k4X); //SIX SEVEN...?

  //PID Chassis
  PIDController pidChassis = new PIDController(0.05, 0.0, 0.0005);

  //PID "Shooter?" y "Elevador?"
  SparkClosedLoopController OrangePID = Shooter.getClosedLoopController();
  SparkClosedLoopController GreenPID = Elevator.getClosedLoopController();

  // PID por rueda
  PIDController pidFL = new PIDController(0.005, 0.0, 0.0);
  PIDController pidFR = new PIDController(0.005, 0.0, 0.0);
  PIDController pidRL = new PIDController(0.005, 0.0, 0.0);
  PIDController pidRR = new PIDController(0.005, 0.0, 0.0);

  // Feedforward por rueda (kS, kV, kA) — valores de ejemplo: debes tunear
  SimpleMotorFeedforward ffFL = new SimpleMotorFeedforward(0.3, 2.0, 0.2);
  SimpleMotorFeedforward ffFR = new SimpleMotorFeedforward(0.3, 2.0, 0.2);
  SimpleMotorFeedforward ffRL = new SimpleMotorFeedforward(0.3, 2.0, 0.2);
  SimpleMotorFeedforward ffRR = new SimpleMotorFeedforward(0.3, 2.0, 0.2);

  //Slews
  SlewRateLimiter SlewMOVE = new SlewRateLimiter(3);
  SlewRateLimiter SlewSTRAFE = new SlewRateLimiter(5);

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
    RearRightConfig.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit(40);
    RearLeftConfig.inverted(true).idleMode(IdleMode.kBrake).smartCurrentLimit(40);
    FrontRightConfig.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit(40);
    FrontLeftConfig.inverted(true).idleMode(IdleMode.kBrake).smartCurrentLimit(40);

    ShooterConfig.inverted(false).idleMode(IdleMode.kCoast).smartCurrentLimit(30);
    ShooterConfig.closedLoop.p(0.0001).i(0.00001).d(0.001);
    ShooterConfig.closedLoop.feedForward.kS(0.0).kV(0.015).kA(0.0).kG(0.0).kCos(0.0).kCosRatio(0.0);

    ElevatorConfig.inverted(true).idleMode(IdleMode.kBrake).smartCurrentLimit(30);
    ElevatorConfig.closedLoop.p(0.1).i(0.0).d(0.0);

    //Aplicar PIDs "Shooter?" y "Elevador?"
    OrangePID = Shooter.getClosedLoopController();
    GreenPID = Elevator.getClosedLoopController();

    RearRight.configure(RearRightConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
    RearLeft.configure(RearLeftConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
    FrontRight.configure(FrontRightConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
    FrontLeft.configure(FrontLeftConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
    Shooter.configure(ShooterConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
    Elevator.configure(ElevatorConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    FrontLeftEncoder.setSamplesToAverage(10);
    FrontLeftEncoder.setDistancePerPulse(1.0 / 360 * (Math.PI * 6) * 0.0254); //Conversión
    FrontLeftEncoder.setMinRate(10);
    FrontLeftEncoder.reset();

    RearLeftEncoder.setSamplesToAverage(10);
    RearLeftEncoder.setDistancePerPulse(1.0 / 360 * (Math.PI * 6) * 0.0254); //Conversión
    RearLeftEncoder.setMinRate(10);
    RearLeftEncoder.reset();

    FrontRightEncoder.setSamplesToAverage(10);
    FrontRightEncoder.setDistancePerPulse(1.0 / 360 * (Math.PI * 6) * 0.0254); //Conversión
    FrontRightEncoder.setMinRate(10);
    FrontRightEncoder.reset();

    RearRightEncoder.setSamplesToAverage(10);
    RearRightEncoder.setDistancePerPulse(1.0 / 360 * (Math.PI * 6) * 0.0254); //Conversión
    RearRightEncoder.setMinRate(10);
    RearRightEncoder.reset();

    pidChassis.enableContinuousInput(-180.0f, 180.0f);
    pidChassis.setIntegratorRange(-1.0, 1.0);
    pidChassis.setTolerance(2.0f);
    pidChassis.isContinuousInputEnabled();

    ChasisMecanum = new MecanumDrive(FrontLeft, RearLeft, FrontRight, RearRight);
    Translation2d frontLeftLocation = new Translation2d(0.29, 0.29);
    Translation2d frontRightLocation = new Translation2d(0.29, -.29);
    Translation2d rearLeftLocation = new Translation2d(-0.29, 0.29);
    Translation2d rearRightLocation = new Translation2d(-0.29, -0.29);
    Rotation2d AnguloNavX = Rotation2d.fromDegrees(navx.getAngle());
    Pose2d initialPose = new Pose2d(StartInX,StartInY, AnguloNavX);
    MecanumDriveWheelPositions initialWheelPositions = new MecanumDriveWheelPositions(FrontLeftEncoder.getDistance(),FrontRightEncoder.getDistance(),RearLeftEncoder.getDistance(),RearRightEncoder.getDistance());

    ChasisMecanum.setDeadband(0.02);
    ChasisMecanum.setMaxOutput(1.0);
    ChasisMecanum.setSafetyEnabled(true);
    ChasisMecanum.setExpiration(0.1);


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

    // Store the angle in a variable to avoid redundant calculations
    Rotation2d AnguloNavX = Rotation2d.fromDegrees(navx.getAngle());


    // Actualiza odometría con posiciones en metros
    MecanumDriveWheelPositions wheelPositions = new MecanumDriveWheelPositions(
      FrontLeftEncoder.getDistance(),
      FrontRightEncoder.getDistance(),
      RearLeftEncoder.getDistance(),
      RearRightEncoder.getDistance());

    xRC_Odometry.update(AnguloNavX, wheelPositions);

    // Opcional: publicar valores útiles
    SmartDashboard.putNumber("Pose X (m)", xRC_Odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("Pose Y (m)", xRC_Odometry.getPoseMeters().getY());
    SmartDashboard.putData("Chasis", ChasisMecanum);
    SmartDashboard.putNumber("Elevador", Elevator.getEncoder().getPosition());
    SmartDashboard.putNumber("Shooter", Shooter.getEncoder().getVelocity());
  }


  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    navx.reset();

    kronos.start();
    kronos.reset();
    
    FrontLeftEncoder.reset();
    FrontRightEncoder.reset();
    RearLeftEncoder.reset();
    RearRightEncoder.reset();

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
    xRC_SlowMode = 1.0;
    FrontLeftEncoder.reset();
    FrontRightEncoder.reset();
    RearLeftEncoder.reset();
    RearRightEncoder.reset();
    AngleTarget = navx.getAngle();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

     //Variables Básicas Chasis y Rotaciones Estándar y PID
    double jRot = ControlCero.getRightX();
    MecanumMove = SlewMOVE.calculate(-ControlCero.getLeftY());
    MecanumStrafe = SlewSTRAFE.calculate(ControlCero.getLeftX() * xRC_SlowMode);
    MecanumRotacionRAW = MathUtil.applyDeadband(jRot, 0.03);
    Rotation2d AnguloNavX = Rotation2d.fromDegrees(navx.getAngle());
    Rotation2d Heading = Rotation2d.fromDegrees(-navx.getAngle());


    //PID Straightmove y Hub AutoAim
    if (Math.abs(MecanumRotacionRAW) > 0.03){
      AngleTarget = AnguloNavX.getDegrees();
      MecanumRotacionPID = MecanumRotacionRAW * xRC_SlowMode;
    }
    else if (ControlCero.getRightTriggerAxis()>=0.3){
      Translation2d DistanceToHub = (BlueHubPose.getTranslation()).minus((xRC_Odometry.getPoseMeters()).getTranslation());
      AngleTarget = -(DistanceToHub.getAngle()).getDegrees();
      MecanumRotacionPID = (pidChassis.calculate(navx.getAngle(), AngleTarget));
    }
    else{
      MecanumRotacionPID = +(pidChassis.calculate(navx.getAngle(), AngleTarget));
    }

    //PIDFF
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(MecanumMove*kMaxSpeedWheel, MecanumStrafe*-kMaxSpeedWheel, MecanumRotacionPID*-kMaxSpeedWheel);
    chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond, Heading);
    MecanumDriveWheelSpeeds wheelSpeeds = xRC_Kinematics.toWheelSpeeds(chassisSpeeds);
    wheelSpeeds.desaturate(kMaxSpeedWheel);

    //Velocidad Meta
    double MetaFL = wheelSpeeds.frontLeftMetersPerSecond;
    double MetaFR = wheelSpeeds.frontRightMetersPerSecond;
    double MetaRL = wheelSpeeds.rearLeftMetersPerSecond;
    double MetaRR = wheelSpeeds.rearRightMetersPerSecond;
    
    //Velocidad Actual (mps)
    double RealFL = FrontLeftEncoder.getRate();
    double RealFR = FrontRightEncoder.getRate();
    double RealRL = RearLeftEncoder.getRate();
    double RealRR = RearRightEncoder.getRate();

    //PID (Unitless por sí solo)
    double FL_PID = pidFL.calculate(RealFL, MetaFL);
    double FR_PID = pidFR.calculate(RealFR, MetaFR);
    double RL_PID = pidRL.calculate(RealRL, MetaRL);
    double RR_PID = pidRR.calculate(RealRR, MetaRR);

    //FeedForward (En VOLTS)
    double FL_FF = ffFL.calculate(MetaFL);
    double FR_FF = ffFR.calculate(MetaFR);
    double RL_FF = ffRL.calculate(MetaRL);
    double RR_FF = ffRR.calculate(MetaRR);

    //Combinación (VOLTS)
    double VoltsFL = FL_FF + FL_PID;
    double VoltsFR = FR_FF + FR_PID;
    double VoltsRL = RL_FF + RL_PID;
    double VoltsRR = RR_FF + RR_PID;

    //Compensación por estatus de bateria
    double pila = RobotController.getBatteryVoltage();
    double PercentFL = MathUtil.clamp(VoltsFL/pila, -1.0, 1.0);
    double PercentFR = MathUtil.clamp(VoltsFR/pila, -1.0, 1.0);
    double PercentRL = MathUtil.clamp(VoltsRL/pila, -1.0, 1.0);
    double PercentRR = MathUtil.clamp(VoltsRR/pila, -1.0, 1.0);

    //Establecer motores PID
    double OutputFL = PercentFL;
    double OutputFR = PercentFR;
    double OutputRL = PercentRL;
    double OutputRR = PercentRR;

    FrontLeft.set(OutputFL);
    FrontRight.set(OutputFR);
    RearLeft.set(OutputRL);
    RearRight.set(OutputRR);

    //Reset NAVX
    if (ControlCero.getStartButton() == true) {
      navx.reset();
      AngleTarget = 0.0;
      pidChassis.reset();
    }   

    SmartDashboard.putNumber("Velo Act", FrontRightEncoder.getRate());
    SmartDashboard.putNumber("Velo Meta", wheelSpeeds.frontRightMetersPerSecond);

    //SlowMode estilo xRC
    if ((ControlCero.getLeftBumperButton() == true) || (ControlCero.getRightBumperButton() == true)) {
      xRC_SlowMode = 0.5;
    } else {
      xRC_SlowMode = 1;
    }

    if (ControlCero.getYButton() == true){
      Shooter.set(1);
    }
    else if (ControlCero.getXButton() == true){
      OrangePID.setSetpoint(-1800, ControlType.kVelocity);
    }
    else{
      Shooter.set(0);
    }

    if (ControlCero.getBButton() == true){
      GreenPID.setSetpoint(0, ControlType.kPosition);
    }
    else if (ControlCero.getAButton() == true){
      GreenPID.setSetpoint(240, ControlType.kPosition);
    }
    else if (ControlCero.getLeftStickButton() == true){
      GreenPID.setSetpoint(675, ControlType.kPosition);
    }
    else if (ControlCero.getRightStickButton() == true){
      GreenPID.setSetpoint(1000, ControlType.kPosition);
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
