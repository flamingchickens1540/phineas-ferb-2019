package org.team1540.robot2019;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Solenoid;
import org.team1540.rooster.wrappers.ChickenTalon;

/**
 * This is my fancy replacement for the RobotMap class. Now instead of centralizing motor numbers, I
 * centralize the motors themselves. This also means we don't have to redo loads of config when
 * making alternative robots or funky testing code.
 */
public class Hardware {

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

  public static Solenoid elevatorBrake;

  public static DigitalInput elevatorTopSwitch;
  public static DigitalInput elevatorBtmSwitch;

  public static DigitalInput armTopSwitch;
  public static DigitalInput armBtmSwitch;


  // solenoid on is arm extended/down
  public static Solenoid armCylinder;
  public static ChickenTalon armMotor;


  // positive setpoint is outtaking
  public static ChickenTalon intakeTop;
  public static ChickenTalon intakeBtm;
  // TODO add intake sensor

  static void initAll() {
    System.out.println("Initializing robot hardware...");
    double start = RobotController.getFPGATime() / 1000.0; // getFPGATime returns microseconds

    initDrive();
    initElevator();
    initArm();
    initIntake();

    double end = RobotController.getFPGATime() / 1000.0;
    System.out.println("Initialized robot hardware in " + (end - start) + " ms");
  }

  public static void initDrive() {
    System.out.println("Initializing drive...");
    double start = RobotController.getFPGATime() / 1000.0; // getFPGATime returns microseconds

    driveLeftMotorA = new ChickenTalon(RobotMap.DRIVE_LEFT_A);
    driveLeftMotorB = new ChickenTalon(RobotMap.DRIVE_LEFT_B);
    driveLeftMotorC = new ChickenTalon(RobotMap.DRIVE_LEFT_C);

    driveRightMotorA = new ChickenTalon(RobotMap.DRIVE_RIGHT_A);
    driveRightMotorB = new ChickenTalon(RobotMap.DRIVE_RIGHT_B);
    driveRightMotorC = new ChickenTalon(RobotMap.DRIVE_RIGHT_C);

    for (ChickenTalon talon : driveMotorAll) {
      talon.configFactoryDefault();
      talon.setBrake(true);
      talon.configVoltageCompSaturation(12);
      talon.enableVoltageCompensation(true);
      // at the moment, this hard caps to driveCurrentLimit; we might implement peak limiting
      // instead
      talon.configPeakCurrentLimit(0);
      talon.configContinuousCurrentLimit(Tuning.driveCurrentLimit);
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
    System.out.println("Initialized drive in " + (end - start) + " ms");
  }

  public static void initElevator() {
    System.out.println("Initializing elevator...");
    double start = RobotController.getFPGATime() / 1000.0; // getFPGATime returns microseconds

    elevatorA = new CANSparkMax(RobotMap.ELEVATOR_L, MotorType.kBrushless);
    elevatorB = new CANSparkMax(RobotMap.ELEVATOR_R, MotorType.kBrushless);

    elevatorBrake = new Solenoid(RobotMap.ELEVATOR_BRAKE);

    elevatorTopSwitch = new DigitalInput(RobotMap.ELEVATOR_TOP_SW);
    elevatorBtmSwitch = new DigitalInput(RobotMap.ELEVATOR_BTM_SW);

    double end = RobotController.getFPGATime() / 1000.0;
    System.out.println("Initialized elevator in " + (end - start) + " ms");
  }

  public static void initArm() {
    System.out.println("Initializing arm...");
    double start = RobotController.getFPGATime() / 1000.0; // getFPGATime returns microseconds

    armCylinder = new Solenoid(RobotMap.INTAKE_CYLINDER);

    armMotor = new ChickenTalon(RobotMap.INTAKE_WRIST);

    armMotor.setInverted(Tuning.armInvertMotor);
    armMotor.setBrake(true);

    armTopSwitch = new DigitalInput(RobotMap.ARM_TOP_SW);
    armBtmSwitch = new DigitalInput(RobotMap.ARM_BTM_SW);

    double end = RobotController.getFPGATime() / 1000.0;
    System.out.println("Initialized arm in " + (end - start) + " ms");
  }

  public static void initIntake() {
    System.out.println("Initializing intake...");
    double start = RobotController.getFPGATime() / 1000.0; // getFPGATime returns microseconds

    intakeTop = new ChickenTalon(RobotMap.INTAKE_TOP);
    intakeBtm = new ChickenTalon(RobotMap.INTAKE_BTM);

    intakeTop.configFactoryDefault();
    intakeBtm.configFactoryDefault();

    intakeTop.setInverted(Tuning.intakeInvertTop);
    intakeBtm.setInverted(Tuning.intakeInvertBtm);

    intakeTop.setBrake(true);
    intakeBtm.setBrake(true);

    double end = RobotController.getFPGATime() / 1000.0;
    System.out.println("Initialized intake in " + (end - start) + " ms");
  }

  public static void checkStickyFaults() {
    PhineasUtilities.processStickyFaults("Drivetrain", "left A", driveLeftMotorA);
    PhineasUtilities.processStickyFaults("Drivetrain", "left B", driveLeftMotorB);
    PhineasUtilities.processStickyFaults("Drivetrain", "left C", driveLeftMotorC);
    PhineasUtilities.processStickyFaults("Drivetrain", "right A", driveRightMotorA);
    PhineasUtilities.processStickyFaults("Drivetrain", "right B", driveRightMotorB);
    PhineasUtilities.processStickyFaults("Drivetrain", "right C", driveRightMotorC);

    PhineasUtilities.processStickyFaults("Arm", "motor", armMotor);

    PhineasUtilities.processStickyFaults("Intake", "top", intakeTop);
    PhineasUtilities.processStickyFaults("Intake", "bottom", intakeBtm);
  }
}
