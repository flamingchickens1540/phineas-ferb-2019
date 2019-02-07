package org.team1540.robot2019;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.*;
import org.apache.log4j.Logger;
import org.team1540.rooster.wrappers.ChickenTalon;

/**
 * This is my fancy replacement for the RobotMap class. Now instead of centralizing motor numbers, I
 * centralize the motors themselves. This also means we don't have to redo loads of config when
 * making alternative robots or funky testing code.
 */
public class Hardware {

  private static final Logger logger = Logger.getLogger(Hardware.class);

  public static final int DRIVE_POSITION_SLOT_IDX = 0;
  public static final int DRIVE_VELOCITY_SLOT_IDX = 1;

  // these aren't final for initialization but don't change them mkay

  public static ChickenTalon driveLeftMotorA;
  public static ChickenTalon driveLeftMotorB;
  public static ChickenTalon driveLeftMotorC;
  public static ChickenTalon[] driveLeftMotors;
  public static ChickenTalon driveRightMotorA;
  public static ChickenTalon driveRightMotorB;
  public static ChickenTalon driveRightMotorC;
  public static ChickenTalon[] driveRightMotors;
  public static ChickenTalon[] driveMotorAll;
  public static ChickenTalon[] driveMotorMasters;


  // positive setpoint is up
  public static CANSparkMax elevatorA;
  public static CANSparkMax elevatorB;

  public static DigitalInput elevatorLimitSensor;

  public static DigitalInput wristMidSwitch;
  public static DigitalInput wristBtmSwitch;


  // solenoid on is wrist extended/down
  public static ChickenTalon wristMotor;


  // positive setpoint is outtaking
  public static ChickenTalon intakeTop;
  public static ChickenTalon intakeBtm;

  public static DigitalInput intakeSensor;

  public static Solenoid hatchSlide;
  public static Solenoid hatchGrabber;

  public static ChickenTalon climberArmLeft;
  public static ChickenTalon climberArmRight;

  public static DoubleSolenoid climberCylinder;


  public static AnalogInput pressureSensor;

  public static AHRS navx;

  static void initAll() {
    logger.info("Initializing robot hardware...");
    double start = RobotController.getFPGATime() / 1000.0; // getFPGATime returns microseconds

    initDrive();
    initElevator();
    initWrist();
    initIntake();
    initHatchMech();
    initClimber();
    initPressureSensor();
    initNavX();

    double end = RobotController.getFPGATime() / 1000.0;
    logger.info("Initialized robot hardware in " + (end - start) + " ms");
  }

  public static void initDrive() {
    logger.info("Initializing drive...");
    double start = RobotController.getFPGATime() / 1000.0; // getFPGATime returns microseconds

    driveLeftMotorA = new ChickenTalon(RobotMap.DRIVE_LEFT_A);
    driveLeftMotorB = new ChickenTalon(RobotMap.DRIVE_LEFT_B);
    driveLeftMotorC = new ChickenTalon(RobotMap.DRIVE_LEFT_C);

    driveRightMotorA = new ChickenTalon(RobotMap.DRIVE_RIGHT_A);
    driveRightMotorB = new ChickenTalon(RobotMap.DRIVE_RIGHT_B);
    driveRightMotorC = new ChickenTalon(RobotMap.DRIVE_RIGHT_C);

    driveMotorAll = new ChickenTalon[]{driveLeftMotorA, driveLeftMotorB, driveLeftMotorC,
        driveRightMotorA, driveRightMotorB, driveRightMotorC};
    driveMotorMasters = new ChickenTalon[]{driveLeftMotorA, driveRightMotorA};
    driveLeftMotors = new ChickenTalon[]{driveLeftMotorA, driveLeftMotorB, driveLeftMotorC};
    driveRightMotors = new ChickenTalon[]{driveRightMotorA, driveRightMotorB, driveRightMotorC};

    for (ChickenTalon talon : driveMotorAll) {
      talon.configFactoryDefault();
      talon.setBrake(true);
      talon.configVoltageCompSaturation(12);
      talon.enableVoltageCompensation(true);
      // at the moment, this hard caps to driveCurrentLimit; we might implement peak limiting
      // instead
      talon.configPeakCurrentLimit(0);
      talon.configPeakOutputForward(1);
      talon.configPeakOutputReverse(-1);
      talon.configContinuousCurrentLimit(Tuning.driveCurrentLimit);
      talon.configOpenloopRamp(Tuning.driveOpenLoopRamp);
    }

    for (ChickenTalon talon : driveMotorMasters) {
      talon.config_kP(DRIVE_POSITION_SLOT_IDX, Tuning.drivePositionP);
      talon.config_kI(DRIVE_POSITION_SLOT_IDX, Tuning.drivePositionI);
      talon.config_kD(DRIVE_POSITION_SLOT_IDX, Tuning.drivePositionD);
      talon.config_kF(DRIVE_POSITION_SLOT_IDX, Tuning.drivePositionF);
      talon.config_kP(DRIVE_VELOCITY_SLOT_IDX, Tuning.driveVelocityP);
      talon.config_kI(DRIVE_VELOCITY_SLOT_IDX, Tuning.driveVelocityI);
      talon.config_kD(DRIVE_VELOCITY_SLOT_IDX, Tuning.driveVelocityD);
      talon.config_kF(DRIVE_VELOCITY_SLOT_IDX, Tuning.driveVelocityF);
    }

    for (ChickenTalon talon : driveLeftMotors) {
      talon.setInverted(Tuning.invertDriveLeft);
    }

    for (ChickenTalon talon : driveRightMotors) {
      talon.setInverted(Tuning.invertDriveRight);
    }

    driveLeftMotorA.setSensorPhase(Tuning.invertDriveLeftSensor);
    driveRightMotorA.setSensorPhase(Tuning.invertDriveRightSensor);

    driveLeftMotorB.set(ControlMode.Follower, driveLeftMotorA.getDeviceID());
    driveLeftMotorC.set(ControlMode.Follower, driveLeftMotorA.getDeviceID());
    driveRightMotorB.set(ControlMode.Follower, driveRightMotorA.getDeviceID());
    driveRightMotorC.set(ControlMode.Follower, driveRightMotorA.getDeviceID());

    double end = RobotController.getFPGATime() / 1000.0;
    logger.info("Initialized drive in " + (end - start) + " ms");
  }

  public static void initElevator() {
    logger.info("Initializing elevator...");
    double start = RobotController.getFPGATime() / 1000.0; // getFPGATime returns microseconds

    elevatorA = new CANSparkMax(RobotMap.ELEVATOR_L, MotorType.kBrushless);
    elevatorB = new CANSparkMax(RobotMap.ELEVATOR_R, MotorType.kBrushless);

    elevatorA.setIdleMode(IdleMode.kBrake);
    elevatorB.setIdleMode(IdleMode.kBrake);

    elevatorA.setInverted(Tuning.invertElevatorA);
    elevatorB.follow(elevatorA, Tuning.invertElevatorB);

    elevatorA.getPIDController().setP(Tuning.elevatorP);
    elevatorA.getPIDController().setI(Tuning.elevatorI);
    elevatorA.getPIDController().setD(Tuning.elevatorD);
    elevatorA.getPIDController().setOutputRange(-1, 1);

    elevatorLimitSensor = new DigitalInput(RobotMap.ELEVATOR_LIMIT_SENSOR);

    double end = RobotController.getFPGATime() / 1000.0;
    logger.info("Initialized elevator in " + (end - start) + " ms");
  }

  public static void initWrist() {
    logger.info("Initializing wrist...");
    double start = RobotController.getFPGATime() / 1000.0; // getFPGATime returns microseconds

    wristMotor = new ChickenTalon(RobotMap.INTAKE_WRIST);

    wristMotor.setInverted(Tuning.wristInvertMotor);
    wristMotor.setBrake(true);

    wristMotor.configVoltageCompSaturation(12);
    wristMotor.enableVoltageCompensation(true);

    wristMidSwitch = new DigitalInput(RobotMap.WRIST_MID_SW);
    wristBtmSwitch = new DigitalInput(RobotMap.WRIST_BTM_SW);

    double end = RobotController.getFPGATime() / 1000.0;
    logger.info("Initialized wrist in " + (end - start) + " ms");
  }

  public static void initIntake() {
    logger.info("Initializing intake...");
    double start = RobotController.getFPGATime() / 1000.0; // getFPGATime returns microseconds

    intakeTop = new ChickenTalon(RobotMap.INTAKE_TOP);
    intakeBtm = new ChickenTalon(RobotMap.INTAKE_BTM);

    intakeTop.configFactoryDefault();
    intakeBtm.configFactoryDefault();

    intakeTop.setInverted(Tuning.intakeInvertTop);
    intakeBtm.setInverted(Tuning.intakeInvertBtm);

    intakeTop.setBrake(true);
    intakeBtm.setBrake(true);

    intakeTop.configVoltageCompSaturation(12);
    intakeTop.enableVoltageCompensation(true);

    intakeBtm.configVoltageCompSaturation(12);
    intakeBtm.enableVoltageCompensation(true);

    intakeSensor = new DigitalInput(RobotMap.INTAKE_SENSOR);

    double end = RobotController.getFPGATime() / 1000.0;
    logger.info("Initialized intake in " + (end - start) + " ms");
  }

  public static void initHatchMech() {
    logger.info("Initializing hatch mech...");
    double start = RobotController.getFPGATime() / 1000.0; // getFPGATime returns microseconds

    hatchSlide = new Solenoid(RobotMap.HATCH_SLIDE);
    hatchGrabber = new Solenoid(RobotMap.HATCH_GRABBER);

    double end = RobotController.getFPGATime() / 1000.0;
    logger.info("Initialized hatch mech in " + (end - start) + " ms");
  }

  public static void initClimber() {
    logger.info("Initializing climber...");
    double start = RobotController.getFPGATime() / 1000.0; // getFPGATime returns microseconds

    climberArmLeft = new ChickenTalon(RobotMap.CLIMBER_ARM_L);
    climberArmRight = new ChickenTalon(RobotMap.CLIMBER_ARM_R);

    climberArmLeft.configFactoryDefault();
    climberArmRight.configFactoryDefault();

    climberArmLeft.setBrake(true);
    climberArmRight.setBrake(true);

    climberArmLeft.setInverted(false);
    climberArmRight.setInverted(true);

    climberArmLeft.configPeakOutputForward(1);
    climberArmLeft.configPeakOutputReverse(-1);

    climberArmLeft.configVoltageCompSaturation(12);
    climberArmLeft.enableVoltageCompensation(true);
    climberArmRight.configVoltageCompSaturation(12);
    climberArmRight.enableVoltageCompensation(true);

    climberArmRight.setControlMode(ControlMode.Follower);
    climberArmRight.set(climberArmLeft.getDeviceID());

    climberArmLeft.config_kP(0, Tuning.climberP);
    climberArmLeft.config_kI(0, Tuning.climberI);
    climberArmLeft.config_kD(0, Tuning.climberD);
    climberArmLeft.config_kF(0, Tuning.climberF);
    climberArmLeft.configMotionCruiseVelocity(Tuning.climberMaxVel);
    climberArmLeft.configMotionAcceleration(Tuning.climberMaxAcc);
    climberArmLeft.setSensorPhase(true);
    climberArmLeft.setSelectedSensorPosition(0);

    climberCylinder = new DoubleSolenoid(RobotMap.CLIMBER_CYLINDER_1, RobotMap.CLIMBER_CYLINDER_2);

    double end = RobotController.getFPGATime() / 1000.0;
    logger.info("Initialized climber in " + (end - start) + " ms");
  }

  public static void initPressureSensor() {
    logger.info("Initializing pressure sensor...");
    double start = RobotController.getFPGATime() / 1000.0; // getFPGATime returns microseconds

    pressureSensor = new AnalogInput(RobotMap.PRESSURE_SENSOR);

    double end = RobotController.getFPGATime() / 1000.0;
    logger.info("Initialized pressure sensor in " + (end - start) + " ms");
  }

  public static void initNavX() {
    logger.info("Initializing NavX-MXP...");
    double start = RobotController.getFPGATime() / 1000.0; // getFPGATime returns microseconds

    navx = new AHRS(RobotMap.NAVX);

    double end = RobotController.getFPGATime() / 1000.0;
    logger.info("Initialized NavX-MXP in " + (end - start) + " ms");
  }

  public static void checkStickyFaults() {
    PhineasUtilities.processStickyFaults("Drivetrain", "left A", driveLeftMotorA);
    PhineasUtilities.processStickyFaults("Drivetrain", "left B", driveLeftMotorB);
    PhineasUtilities.processStickyFaults("Drivetrain", "left C", driveLeftMotorC);
    PhineasUtilities.processStickyFaults("Drivetrain", "right A", driveRightMotorA);
    PhineasUtilities.processStickyFaults("Drivetrain", "right B", driveRightMotorB);
    PhineasUtilities.processStickyFaults("Drivetrain", "right C", driveRightMotorC);

    PhineasUtilities.processStickyFaults("Wrist", "motor", wristMotor);

    PhineasUtilities.processStickyFaults("Intake", "top", intakeTop);
    PhineasUtilities.processStickyFaults("Intake", "bottom", intakeBtm);
  }
}
