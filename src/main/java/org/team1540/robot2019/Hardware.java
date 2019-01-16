package org.team1540.robot2019;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import org.team1540.rooster.wrappers.ChickenTalon;

/**
 * This is my fancy replacement for the RobotMap class. Now instead of centralizing motor numbers, I
 * centralize the motors themselves. This also means we don't have to redo loads of config when
 * making alternative robots or funky testing code.
 */
public class Hardware {

  // motors
  public static final int DRIVE_LEFT_A = 1;
  public static final int DRIVE_LEFT_B = 2;
  public static final int DRIVE_LEFT_C = 3;

  public static final int DRIVE_RIGHT_A = 4;
  public static final int DRIVE_RIGHT_B = 5;
  public static final int DRIVE_RIGHT_C = 6;

  public static final int ELEVATOR_A = 7;
  public static final int ELEVATOR_B = 8;

  public static final int INTAKE_TOP = 9;
  public static final int INTAKE_BTM = 10;


  // pneumatics
  public static final int ELEVATOR_BRAKE = 0;

  public static final int ARM_ACTUATOR = 0;

  public static final int HATCH_ACTUATOR = 0;

  public static final int CLIMBER_REAR_PISTON = 0;

  // sensors and switches
  public static final int ELEVATOR_TOP_SW = 0;
  public static final int ELEVATOR_BTM_SW = 0;

  public static final int INTAKE_SENSOR = 0;

  public static final int GROUND_PROXIMITY_SENSOR = 0;

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


  // solenoid on is arm extended/down
  public static Solenoid armActuator;


  // positive setpoint is outtaking
  public static ChickenTalon intakeTop;
  public static ChickenTalon intakeBtm;
  // TODO add intake sensor

  static void initAll() {
    initDrive();
    initElevator();
    initArm();
    initIntake();
  }

  public static void initDrive() {
    driveLeftMotorA = new ChickenTalon(DRIVE_LEFT_A);
    driveLeftMotorB = new ChickenTalon(DRIVE_LEFT_B);
    driveLeftMotorC = new ChickenTalon(DRIVE_LEFT_C);

    driveRightMotorA = new ChickenTalon(DRIVE_RIGHT_A);
    driveRightMotorB = new ChickenTalon(DRIVE_RIGHT_B);
    driveRightMotorC = new ChickenTalon(DRIVE_RIGHT_C);

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
  }

  public static void initElevator() {
    elevatorA = new CANSparkMax(ELEVATOR_A, MotorType.kBrushless);
    elevatorB = new CANSparkMax(ELEVATOR_B, MotorType.kBrushless);

    elevatorBrake = new Solenoid(ELEVATOR_BRAKE);

    elevatorTopSwitch = new DigitalInput(ELEVATOR_TOP_SW);
    elevatorBtmSwitch = new DigitalInput(ELEVATOR_BTM_SW);
  }

  public static void initArm() {
    armActuator = new Solenoid(ARM_ACTUATOR);
  }

  public static void initIntake() {
    intakeTop = new ChickenTalon(INTAKE_TOP);
    intakeBtm = new ChickenTalon(INTAKE_BTM);

    intakeTop.configFactoryDefault();
    intakeBtm.configFactoryDefault();

    intakeTop.setInverted(Tuning.intakeInvertTop);
    intakeBtm.setInverted(Tuning.intakeInvertBtm);

    intakeTop.setBrake(true);
    intakeBtm.setBrake(true);
  }

  public static void checkStickyFaults() {
    PhineasUtilities.processStickyFaults("Drivetrain", "left A", driveLeftMotorA);
    PhineasUtilities.processStickyFaults("Drivetrain", "left B", driveLeftMotorB);
    PhineasUtilities.processStickyFaults("Drivetrain", "left C", driveLeftMotorC);
    PhineasUtilities.processStickyFaults("Drivetrain", "right A", driveRightMotorA);
    PhineasUtilities.processStickyFaults("Drivetrain", "right B", driveRightMotorB);
    PhineasUtilities.processStickyFaults("Drivetrain", "right C", driveRightMotorC);

    PhineasUtilities.processStickyFaults("Intake", "top", intakeTop);
    PhineasUtilities.processStickyFaults("Intake", "bottom", intakeBtm);
  }
}
