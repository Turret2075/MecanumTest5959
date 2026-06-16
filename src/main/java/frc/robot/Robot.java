// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//Code by Turret2075, this is my first FRC Java code.

package frc.robot;

//Librerias WPI estandar
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
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

//Librerias SparkMax
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

//Libreria NavX
import com.studica.frc.AHRS;

//Librerias Simulacion
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.RobotBase;


/*====================================================
    Red CAN (ID . Dispositivo)
    ==========================
    00 . roboRIO
    
    02 . RearRight
    03 . RearLeft
    04 . FrontRight
    05 . FrontLeft
    06 . VICTOR SPX LIBRE
    07 . VICTOR SPX LIBRE
    08 . VICTOR SPX LIBRE
    09 . VICTOR SPX LIBRE
    10 . Intake TITANIUM
    11 . Intake TURRET

======================================================
    Red ENCODERS (PuertoA, PuertoB . Dispositivo)
    0, 1 . FrontLeft
    8, 9 . RearLeft
    4, 5 . FrontRight
    6, 7 . RearRight



 */


public class Robot extends TimedRobot {
  //Variables CAN_ID Chasis
  int RearRightID = 2;
  int RearLeftID = 3;
  int FrontRightID = 4;
  int FrontLeftID = 5;

  //Variables AimLOCK
  int LockInPOV; //Guarda los grados de la cruceta (Si, se mide en grados)

  //Variables PIDrive
  double kMaxSpeedWheel = 3.2; //Velocidad pico de una llanta del chasis en la alfombra en M/s
  double MaxVolts = 12.0; //Voltaje maximo meta a los sparks

  //Sparkmaxes chasis
  SparkMax RearRight = new SparkMax(RearRightID, MotorType.kBrushed);
  SparkMax RearLeft = new SparkMax(RearLeftID, MotorType.kBrushed);
  SparkMax FrontRight = new SparkMax(FrontRightID, MotorType.kBrushed);
  SparkMax FrontLeft = new SparkMax(FrontLeftID, MotorType.kBrushed);

  //SparkMaxes intakes
  SparkMax IntakeFijo = new SparkMax(11, MotorType.kBrushless);

  //Ajustes SparkMax chasis
  SparkMaxConfig RearRightConfig = new SparkMaxConfig();
  SparkMaxConfig RearLeftConfig = new SparkMaxConfig();
  SparkMaxConfig FrontRightConfig = new SparkMaxConfig();
  SparkMaxConfig FrontLeftConfig = new SparkMaxConfig();

  //Configuracion SparkMaxes Intakes
  SparkMaxConfig IntakeFijoConfig = new SparkMaxConfig();
  SparkMaxConfig intakeTITANIUMConfig = new SparkMaxConfig();

  ///Control y Chasis
  XboxController ControlCero = new XboxController(0);
  MecanumDrive ChasisMecanum;
  MecanumDriveKinematics xRC_Kinematics;
  MecanumDriveOdometry xRC_Odometry;
  MecanumDrivePoseEstimator xRC_PoseEstimator;

  //Giroscopio y su angulo para pose y calculos
  AHRS navx = new AHRS(AHRS.NavXComType.kMXP_SPI);
  Rotation2d Heading;

  //Giroscopio Virtual
  AnalogGyroSim gSimNavX;

  //Importar Campo
  Field2d canchaREBUILT = new Field2d();
  //Declarar Timer para Autonomos
  Timer kronos = new Timer(); //KORG referencia!

  //Valores decimales:

  //Rotacion en autonomo
  double RotAuto;
  //Modo lento como el del xRC Simulator
  double xRC_SlowMode;
  //Chasis Joystick RAW
  double jMove;
  double jStrafe;
  double jRot;
  //Chasis Joystick Real
  double MecanumMove;
  double MecanumStrafe;
  double MecanumRotacionRAW;
  //Rotacion hipertuneada
  double MecanumRotacionPID;
  //Angulo para autocorrector de pose
  double AngleTarget;
  
  //Coordenadas de Inicio
  double StartInX = 4.525;
  double StartInY = 0.650;
  
  //Coordenadas de simulacion
  double SimX = 4.525;
  double SimY = 0.650;

  //Encoders Reales
  Encoder FrontLeftEncoder = new Encoder(0,1,false, Encoder.EncodingType.k4X);
  Encoder RearLeftEncoder = new Encoder(8,9,false, Encoder.EncodingType.k4X);
  Encoder FrontRightEncoder = new Encoder(4,5,true, Encoder.EncodingType.k4X);
  Encoder RearRightEncoder = new Encoder(6,7,true, Encoder.EncodingType.k4X); //SIX SEVEN...?

  //Encoders Virtuales
  EncoderSim
    eSimFL,
    eSimFR,
    eSimRL,
    eSimRR
    ;

  //Valores PID Llantas
  double kP_wheel = 0.01;
  double kI_wheel = 0.0;
  double kD_wheel = 0.0;
  double kS_wheel = 0.4;
  double kV_wheel = 3.2;
  double kA_wheel = 0.0;

  //Valores PID Rotacion
  double kP_chassis = 0.028;
  double kI_chassis = 0.0;
  double kD_chassis = 0.0032;

  //PID Chassis
  PIDController pidChassis = new PIDController(kP_chassis, kI_chassis, kD_chassis);

  // PID por rueda
  PIDController pidFL = new PIDController(kP_wheel, kI_wheel, kD_wheel);
  PIDController pidFR = new PIDController(kP_wheel, kI_wheel, kD_wheel);
  PIDController pidRL = new PIDController(kP_wheel, kI_wheel, kD_wheel);
  PIDController pidRR = new PIDController(kP_wheel, kI_wheel, kD_wheel);

  // Feedforward por rueda (kS, kV, kA) — valores de ejemplo: debes tunear
  SimpleMotorFeedforward ffFL = new SimpleMotorFeedforward(kS_wheel, kV_wheel, kA_wheel);
  SimpleMotorFeedforward ffFR = new SimpleMotorFeedforward(kS_wheel, kV_wheel, kA_wheel);
  SimpleMotorFeedforward ffRL = new SimpleMotorFeedforward(kS_wheel, kV_wheel, kA_wheel);
  SimpleMotorFeedforward ffRR = new SimpleMotorFeedforward(kS_wheel, kV_wheel, kA_wheel);

  //Variables para telemetria de velocidad
  //Velocidades meta (explicado mas adelante)
  double MetaFL;
  double MetaFR;
  double MetaRL;
  double MetaRR;
  //Velocidades reales (explicado mas adelante)
  double RealFL;
  double RealFR;
  double RealRL;
  double RealRR;

  //Slews
  SlewRateLimiter SlewMOVE = new SlewRateLimiter(8);
  SlewRateLimiter SlewSTRAFE = new SlewRateLimiter(4);
  SlewRateLimiter SlewROTATE = new SlewRateLimiter(8);

  //Velocidades de las ruedas y chasis para simulacion y calculos
  MecanumDriveWheelSpeeds wheelSpeeds = new MecanumDriveWheelSpeeds(0,0,0,0);
  ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0,0,0);

  //UI Escojer Autonomos
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
    //Configurar los Sparks
    RearRightConfig.inverted(false).idleMode(IdleMode.kBrake)
    .smartCurrentLimit(40).voltageCompensation(MaxVolts);
    RearLeftConfig.inverted(true).idleMode(IdleMode.kBrake)
    .smartCurrentLimit(40).voltageCompensation(MaxVolts);
    FrontRightConfig.inverted(false).idleMode(IdleMode.kBrake)
    .smartCurrentLimit(40).voltageCompensation(MaxVolts);
    FrontLeftConfig.inverted(true).idleMode(IdleMode.kBrake)
    .smartCurrentLimit(40).voltageCompensation(MaxVolts);

    IntakeFijoConfig.inverted(false).idleMode(IdleMode.kBrake)
    .smartCurrentLimit(30);

    //Configurar Sparks
    RearRight.configure(
      RearRightConfig, 
      SparkBase.ResetMode.kResetSafeParameters,
      SparkBase.PersistMode.kPersistParameters
    );
    RearLeft.configure(
      RearLeftConfig, 
      SparkBase.ResetMode.kResetSafeParameters,
      SparkBase.PersistMode.kPersistParameters
    );
    FrontRight.configure(
      FrontRightConfig, 
      SparkBase.ResetMode.kResetSafeParameters,
      SparkBase.PersistMode.kPersistParameters
    );
    FrontLeft.configure(
      FrontLeftConfig, 
      SparkBase.ResetMode.kResetSafeParameters,
      SparkBase.PersistMode.kPersistParameters
    );

    //Configurar Intakes
    IntakeFijo.configure(
      IntakeFijoConfig, 
      SparkBase.ResetMode.kResetSafeParameters,
      SparkBase.PersistMode.kPersistParameters
    );
    
    //Configurar Encoders
    FrontLeftEncoder.setSamplesToAverage(10);
    FrontLeftEncoder.setDistancePerPulse(1.0 / 360 * (Math.PI * 6) * 0.0254); //Conversion a metros
    FrontLeftEncoder.setMinRate(10);
    FrontLeftEncoder.reset();

    RearLeftEncoder.setSamplesToAverage(10);
    RearLeftEncoder.setDistancePerPulse(1.0 / 360 * (Math.PI * 6) * 0.0254); //Conversion a metros
    RearLeftEncoder.setMinRate(10);
    RearLeftEncoder.reset();

    FrontRightEncoder.setSamplesToAverage(10);
    FrontRightEncoder.setDistancePerPulse(1.0 / 360 * (Math.PI * 6) * 0.0254); //Conversion a metros
    FrontRightEncoder.setMinRate(10);
    FrontRightEncoder.reset();

    RearRightEncoder.setSamplesToAverage(10);
    RearRightEncoder.setDistancePerPulse(1.0 / 360 * (Math.PI * 6) * 0.0254); //Conversion a metros
    RearRightEncoder.setMinRate(10);
    RearRightEncoder.reset();

    //Configurar VirtualCoders y VirtualGyro
    if (RobotBase.isSimulation()) {
      eSimFL = new EncoderSim(FrontLeftEncoder);
      eSimFR = new EncoderSim(FrontRightEncoder);
      eSimRL = new EncoderSim(RearLeftEncoder);
      eSimRR = new EncoderSim(RearRightEncoder);
      gSimNavX = new AnalogGyroSim(0); 
    }

    //Ajustes PID Rotacion (code snippet took from Bea's code)
    pidChassis.enableContinuousInput(-180.0f, 180.0f);
    pidChassis.setIntegratorRange(-1.0, 1.0);
    pidChassis.setTolerance(2.0f);
    pidChassis.isContinuousInputEnabled();

    //Declarar chasis
    ChasisMecanum = new MecanumDrive(
      FrontLeft, 
      RearLeft, 
      FrontRight, 
      RearRight
    );

    //Ubicacion llantas
    Translation2d frontLeftLocation = new Translation2d(0.29, 0.29);
    Translation2d frontRightLocation = new Translation2d(0.29, -0.29);
    Translation2d rearLeftLocation = new Translation2d(-0.29, 0.29);
    Translation2d rearRightLocation = new Translation2d(-0.29, -0.29);

    //Giroscopio CCW o Virtual para iniciar odometria
    if (RobotBase.isReal()){
      Heading = Rotation2d.fromDegrees(-navx.getAngle());
    }
    else{
      Heading = Rotation2d.fromDegrees(gSimNavX.getAngle());
    }

    //Pose inicial del robot
    Pose2d initialPose = new Pose2d(StartInX, StartInY, Heading);

    //Declarar posiciones de las llantas al iniciar
    MecanumDriveWheelPositions initialWheelPositions = new MecanumDriveWheelPositions(
      FrontLeftEncoder.getDistance(),
      FrontRightEncoder.getDistance(),
      RearLeftEncoder.getDistance(),
      RearRightEncoder.getDistance()
    );

    //Ajustes chasis
    ChasisMecanum.setDeadband(0.0125);
    ChasisMecanum.setMaxOutput(1.0);
    ChasisMecanum.setExpiration(0.1);
    ChasisMecanum.setSafetyEnabled(false);
    ChasisMecanum.feed();

    //Declarar Cinematicas, Odometria y estimacion de posicion
    xRC_Kinematics = new MecanumDriveKinematics(
      frontLeftLocation, 
      frontRightLocation, 
      rearLeftLocation, 
      rearRightLocation
    );

    xRC_Odometry = new MecanumDriveOdometry(
      xRC_Kinematics, 
      Heading, 
      initialWheelPositions, 
      initialPose
    );

    xRC_PoseEstimator = new MecanumDrivePoseEstimator(
      xRC_Kinematics, 
      Heading, 
      initialWheelPositions, 
      initialPose
    );
   
    //Configurar lista de Autonomos (TIMER)
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

    //Giroscopio CCW o Virtualgyro para actualizar la odometria
    if (RobotBase.isReal()){
      Heading = Rotation2d.fromDegrees(-navx.getAngle());
    }
    else{
      Heading = Rotation2d.fromDegrees(gSimNavX.getAngle());
    }

    // Actualiza odometria con posiciones en metros
    MecanumDriveWheelPositions wheelPositions = new MecanumDriveWheelPositions(
      FrontLeftEncoder.getDistance(),
      FrontRightEncoder.getDistance(),
      RearLeftEncoder.getDistance(),
      RearRightEncoder.getDistance());

    //Actualiza la Odometria
    xRC_Odometry.update(Heading, wheelPositions);

    //Actualizar estimador de pose
    xRC_PoseEstimator.update(Heading, wheelPositions);

    //Proyectamos el robot en la cancha virtual de la UI
    canchaREBUILT.setRobotPose(xRC_Odometry.getPoseMeters());

    //------------------------Publicar valores utiles------------------------------

    //Pose XY del robot
    SmartDashboard.putNumber("RobotPose/Pose X (m)", xRC_Odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("RobotPose/Pose Y (m)", xRC_Odometry.getPoseMeters().getY());

    //Objeto Chasis
    SmartDashboard.putData("Mecanum", ChasisMecanum);

    //Proyectar la cancha
    SmartDashboard.putData("Cancha", canchaREBUILT);

    //Obtener voltaje de pila
    SmartDashboard.putNumber("Bateria", RobotController.getBatteryVoltage());

    //Mostrar control
    SmartDashboard.putData("Control", ControlCero);

        
    //--------------------------Telemetria teleop-------------------------------

    //Angulo actual y meta
    SmartDashboard.putNumber("AngulosChasis/Heading", Heading.getDegrees());
    SmartDashboard.putNumber("AngulosChasis/AngleTarget", AngleTarget);
    
    //Velocidades Meta
    SmartDashboard.putNumber("TargetSpeeds/Meta_FL", MetaFL);
    SmartDashboard.putNumber("TargetSpeeds/Meta_FR", MetaFR);
    SmartDashboard.putNumber("TargetSpeeds/Meta_RL", MetaRL);
    SmartDashboard.putNumber("TargetSpeeds/Meta_RR", MetaRR);
    
    //Velocidades Reales
    SmartDashboard.putNumber("RealSpeeds/Real_FL", RealFL);
    SmartDashboard.putNumber("RealSpeeds/Real_FR", RealFR);
    SmartDashboard.putNumber("RealSpeeds/Real_RL", RealRL);
    SmartDashboard.putNumber("RealSpeeds/Real_RR", RealRR);
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
          //Desplegar Cajon e Intake
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
          //Desplegar Cajon e Intake
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
        }
        break;

      case kEncodedAutoIzquierda:
        break;

      case kCenterAuto: //Autonomo Retroceder y Disparar ~6-8 Fuel
        if (kronos.get()<=2.2) {
          ChasisMecanum.driveCartesian(-0.2, 0, 0);
        }
        //Disparar Fuel
        else if (kronos.get()<=8){
        }
        break;

      case kAutoMuyPro:
      default:
      System.out.println("Dejaselo a los pros");
      //Deja que los otros robots hagan sus autos sin estorbar
    }
  }

  @Override
  public void teleopInit() {
    //Reseteamos modo lento para evitar errores
    xRC_SlowMode = 1.0;

    //Reseteo Encoders
    FrontLeftEncoder.reset();
    FrontRightEncoder.reset();
    RearLeftEncoder.reset();
    RearRightEncoder.reset();

    //Ajustamos el angulo actual para evitar problemas tras autonomo
    AngleTarget = -navx.getAngle();

    //SOLO POR PRUEBAS - Reseteamos pose y odom cada vez que arranca teleop
    Pose2d startPose = new Pose2d(StartInX, StartInY, Rotation2d.fromDegrees(-navx.getAngle()));
    xRC_Odometry.resetPose(startPose);
  }

  @Override
  public void teleopPeriodic() {

    //Datos RAW para calculos
    jMove = -(ControlCero.getLeftY());
    jStrafe = ControlCero.getLeftX() * xRC_SlowMode;
    jRot = ControlCero.getRightX();

    //Asignamos los angulos de la cruceta
    LockInPOV = ControlCero.getPOV();

    //Los datos buenos para el control
    MecanumMove = SlewMOVE.calculate(jMove);
    MecanumStrafe = SlewSTRAFE.calculate(jStrafe);
    MecanumRotacionRAW = SlewROTATE.calculate(jRot);

    //Rotacion normal actualizando setpoint
    if (Math.abs(MecanumRotacionRAW) > 0.02 && LockInPOV == -1){
      AngleTarget = Heading.getDegrees();
      MecanumRotacionPID = MecanumRotacionRAW * xRC_SlowMode;
    }
    // Rotacion AutoAIM con cruceta
    else if (LockInPOV != -1){
      AngleTarget = -LockInPOV; 
      //Obtenemos angulo coterminal, mate 2 TecMi!
      if (AngleTarget < -180){
        AngleTarget = AngleTarget + 360;
      }
      MecanumRotacionPID = -(pidChassis.calculate(Heading.getDegrees(), AngleTarget));
    }
    else
    { //Counter rotacion con PID
      MecanumRotacionPID = -(pidChassis.calculate(Heading.getDegrees(), AngleTarget));
    }

    //Crear velocidades del chasis
    chassisSpeeds = new ChassisSpeeds(
      MecanumMove*kMaxSpeedWheel, 
      MecanumStrafe*-kMaxSpeedWheel, 
      MecanumRotacionPID*-4.5);

    //Toggle para modo Robot Centric
    if (!ControlCero.getLeftBumperButton() == true){  
    chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      chassisSpeeds.vxMetersPerSecond, 
      chassisSpeeds.vyMetersPerSecond, 
      chassisSpeeds.omegaRadiansPerSecond, 
      Heading);
    }

    //Leer velocidades de las llantas y desaturar para no pasar el maximo
    wheelSpeeds = xRC_Kinematics.toWheelSpeeds(chassisSpeeds);
    wheelSpeeds.desaturate(kMaxSpeedWheel);

    //Velocidad Meta en M/s (A cuanto queremos ir)
    MetaFL = wheelSpeeds.frontLeftMetersPerSecond;
    MetaFR = wheelSpeeds.frontRightMetersPerSecond;
    MetaRL = wheelSpeeds.rearLeftMetersPerSecond;
    MetaRR = wheelSpeeds.rearRightMetersPerSecond;
    
    //Velocidad Actual en M/s (A cuanto realmente vamos)
    RealFL = FrontLeftEncoder.getRate();
    RealFR = FrontRightEncoder.getRate();
    RealRL = RearLeftEncoder.getRate();
    RealRR = RearRightEncoder.getRate();

    //PID (Unitless por si solo)
    double FL_PID = pidFL.calculate(RealFL, MetaFL);
    double FR_PID = pidFR.calculate(RealFR, MetaFR);
    double RL_PID = pidRL.calculate(RealRL, MetaRL);
    double RR_PID = pidRR.calculate(RealRR, MetaRR);

    //FeedForward (En VOLTS)
    double FL_FF = ffFL.calculate(MetaFL);
    double FR_FF = ffFR.calculate(MetaFR);
    double RL_FF = ffRL.calculate(MetaRL);
    double RR_FF = ffRR.calculate(MetaRR);

    //Combinacion (VOLTS)
    double VoltsFL = FL_FF + FL_PID;
    double VoltsFR = FR_FF + FR_PID;
    double VoltsRL = RL_FF + RL_PID;
    double VoltsRR = RR_FF + RR_PID;

    //Ajustar limites de voltaje al motor
    double PercentFL = MathUtil.clamp(VoltsFL, -MaxVolts, MaxVolts);
    double PercentFR = MathUtil.clamp(VoltsFR, -MaxVolts, MaxVolts);
    double PercentRL = MathUtil.clamp(VoltsRL, -MaxVolts, MaxVolts);
    double PercentRR = MathUtil.clamp(VoltsRR, -MaxVolts, MaxVolts);

    //Establecer control de motores ya con PID a otra variable mas logica
    double OutputFL = PercentFL;
    double OutputFR = PercentFR;
    double OutputRL = PercentRL;
    double OutputRR = PercentRR;

    //Mandar el dato a cada motor
    FrontLeft.setVoltage(OutputFL);
    FrontRight.setVoltage(OutputFR);
    RearLeft.setVoltage(OutputRL);
    RearRight.setVoltage(OutputRR);

    //Resetear Giroscopio
    if (ControlCero.getStartButton() == true) {
      navx.reset();
      AngleTarget = 0.0;
      pidChassis.reset();
    }

    //Toggle intake
    if (ControlCero.getXButton() == true){
      IntakeFijo.set(1.0);
    }
    else if (ControlCero.getBButton() == true){
      IntakeFijo.set(-0.6);
    }
    else{
      IntakeFijo.set(0);
    }
  }

  @Override
  public void disabledInit() {
    //Codigo al iniciar el deshabilitar
  }

  @Override
  public void disabledPeriodic() {
    //Codigo cuando esta deshabilitado
  }

  @Override
  public void testInit() {
    //Codigo al iniciar el modo prueba (algo como un system check)
  }

  @Override
  public void testPeriodic() {
    //Codigo cuando esta en modo prueba
  }

  @Override
  public void simulationInit() {
    //Codigo al iniciar el modo simulacion (NO MAPLE, ESTO NO ES SWERVE)
  }

  @Override
  public void simulationPeriodic() {
    //Verificar que los objetos de simulacion esten inicializados
    if (
      eSimFL == null || 
      eSimFR == null || 
      eSimRL == null || 
      eSimRR == null || 
      gSimNavX == null) 
      {
        return; 
    }

    //Velocidades de las ruedas para simulacion (en M/s)
    double VeloSimFL = wheelSpeeds.frontLeftMetersPerSecond;
    double VeloSimFR = wheelSpeeds.frontRightMetersPerSecond;
    double VeloSimRL = wheelSpeeds.rearLeftMetersPerSecond;
    double VeloSimRR = wheelSpeeds.rearRightMetersPerSecond;

    //Inyectar velocidades a los encoders virtuales
    eSimFL.setRate(VeloSimFL);
    eSimFR.setRate(VeloSimFR);
    eSimRL.setRate(VeloSimRL);
    eSimRR.setRate(VeloSimRR);

    //Angulo del giroscopio para simulacion y convertirlo a grados
    double DeltaDegrees = chassisSpeeds.omegaRadiansPerSecond * 0.02 * (180.0 / Math.PI);
    
    //Actualizar el VirtualGyro
    gSimNavX.setAngle(gSimNavX.getAngle() + DeltaDegrees);

    //Cinematica inversa para mover el robot en la simulacion
    double VeloX = chassisSpeeds.vxMetersPerSecond * 0.02; // Convertir a distancia por ciclo de la RoboRIO
    double VeloY = chassisSpeeds.vyMetersPerSecond * 0.02; // Convertir a distancia por ciclo de la RoboRIO
    
    //Ajustamos VeloX y VeloY para simular
    VeloX = VeloX * 0.95;
    VeloY = VeloY * 0.55;

    //Obtener angulo actual del robot en la simulacion
    double RadiansSim = Heading.getRadians();

    //Cinematica inversa que calcula velocidad Field Centric
    double KinematicsInX = VeloX * Math.cos(RadiansSim) - VeloY * Math.sin(RadiansSim);
    double KinematicsInY = VeloX * Math.sin(RadiansSim) + VeloY * Math.cos(RadiansSim);

    //Actualizar la posicion del robot en la simulacion
    SimX += KinematicsInX;
    SimY += KinematicsInY;

    //Actualizar la pose del robot en la simulacion
    Pose2d SimPose = new Pose2d(SimX, SimY, Heading);
    canchaREBUILT.setRobotPose(SimPose);

    //Publicar Veldades y Poses a Dashboard
    SmartDashboard.putNumber("SimSpeeds/SimSpeedX", chassisSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("SimSpeeds/SimSpeedY", chassisSpeeds.vyMetersPerSecond);
    SmartDashboard.putNumber("SimPose/SimPoseX", SimX);
    SmartDashboard.putNumber("SimPose/SimPoseY", SimY);
    SmartDashboard.putNumber("GiroRadianes", RadiansSim);
  } 
}
