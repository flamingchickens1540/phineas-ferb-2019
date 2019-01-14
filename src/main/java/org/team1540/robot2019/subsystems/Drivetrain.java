package org.team1540.robot2019.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.IMotorController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.jetbrains.annotations.NotNull;
import org.team1540.robot2019.OI;
import org.team1540.robot2019.RobotMap;
import org.team1540.rooster.drive.pipeline.AdvancedArcadeJoystickInput;
import org.team1540.robot2019.Tuning;
import org.team1540.robot2019.PhineasUtilities;
import org.team1540.rooster.drive.pipeline.DriveData;
import org.team1540.rooster.drive.pipeline.TankDriveData;
import org.team1540.rooster.functional.Output;
import org.team1540.rooster.functional.Processor;
import org.team1540.rooster.wrappers.ChickenTalon;
import org.team1540.rooster.util.SimpleLoopCommand;

public class Drivetrain extends Subsystem {

  private static final int POSITION_SLOT_IDX = 0;
  private static final int VELOCITY_SLOT_IDX = 1;

  private ChickenTalon driveLeftMotorA;
  private ChickenTalon driveLeftMotorB;
  private ChickenTalon driveLeftMotorC;
  private ChickenTalon[] driveLeftMotors;
  private ChickenTalon driveRightMotorA;
  private ChickenTalon driveRightMotorB;
  private ChickenTalon driveRightMotorC;
  private ChickenTalon[] driveRightMotors;
  private ChickenTalon[] driveMotorAll;
  private ChickenTalon[] driveMotorMasters;

  double leftRampAccumulator;
  double rightRampAccumulator;

  private NetworkTable table = NetworkTableInstance.getDefault().getTable("drivetrain");
  private NetworkTableEntry leftPositionEntry = table.getEntry("leftPos");
  private NetworkTableEntry rightPositionEntry = table.getEntry("rightPos");

  private NetworkTableEntry leftVelocityEntry = table.getEntry("leftVel");
  private NetworkTableEntry rightVelocityEntry = table.getEntry("rightVel");

  private NetworkTableEntry leftThrottleEntry = table.getEntry("leftThrot");
  private NetworkTableEntry rightThrottleEntry = table.getEntry("rightThrot");

  private NetworkTableEntry leftCurrentAEntry = table.getEntry("leftCurrA");
  private NetworkTableEntry leftCurrentBEntry = table.getEntry("leftCurrB");
  private NetworkTableEntry leftCurrentCEntry = table.getEntry("leftCurrC");
  
  private NetworkTableEntry rightCurrentAEntry = table.getEntry("rightCurrA");
  private NetworkTableEntry rightCurrentBEntry = table.getEntry("rightCurrB");
  private NetworkTableEntry rightCurrentCEntry = table.getEntry("rightCurrC");

  public Drivetrain() {

    driveLeftMotorA = new ChickenTalon(RobotMap.DRIVE_LEFT_A);
    driveLeftMotorB = new ChickenTalon(RobotMap.DRIVE_LEFT_B);
    driveLeftMotorC = new ChickenTalon(RobotMap.DRIVE_LEFT_C);

    driveRightMotorA = new ChickenTalon(RobotMap.DRIVE_RIGHT_A);
    driveRightMotorB = new ChickenTalon(RobotMap.DRIVE_RIGHT_B);
    driveRightMotorC = new ChickenTalon(RobotMap.DRIVE_RIGHT_C);

    driveLeftMotors = new ChickenTalon[]{driveLeftMotorA, driveLeftMotorB, driveLeftMotorC};
    driveRightMotors = new ChickenTalon[]{driveRightMotorA, driveRightMotorB, driveRightMotorC};
    driveMotorAll = new ChickenTalon[]{driveLeftMotorA, driveLeftMotorB, driveLeftMotorC,
        driveRightMotorA, driveRightMotorB, driveRightMotorC};
    driveMotorMasters = new ChickenTalon[]{driveLeftMotorA, driveRightMotorA};

    reset();
  }

  public void reset() {
    checkStickyFaults();

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
      talon.config_kP(POSITION_SLOT_IDX, Tuning.drivePositionP);
      talon.config_kI(POSITION_SLOT_IDX, Tuning.drivePositionI);
      talon.config_kD(POSITION_SLOT_IDX, Tuning.drivePositionD);
      talon.config_kF(POSITION_SLOT_IDX, Tuning.drivePositionF);
      talon.config_kP(VELOCITY_SLOT_IDX, Tuning.driveVelocityP);
      talon.config_kI(VELOCITY_SLOT_IDX, Tuning.driveVelocityI);
      talon.config_kD(VELOCITY_SLOT_IDX, Tuning.driveVelocityD);
      talon.config_kF(VELOCITY_SLOT_IDX, Tuning.driveVelocityF);
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

  @Override
  protected void initDefaultCommand() {
    setDefaultCommand(new SimpleLoopCommand("Drive",
        new AdvancedArcadeJoystickInput(true, OI::getDriveThrottle, OI::getDriveSoftTurn,
            OI::getDriveHardTurn)
            .then((Processor<TankDriveData, TankDriveData>) tankDriveData -> tankDriveData
                .withAdditionalFeedForwards(leftRampAccumulator += Math.signum(
                    tankDriveData.left.additionalFeedForward.getAsDouble()
                        - leftRampAccumulator) * Tuning.driveControlRamp,
                    rightRampAccumulator += Math.signum(
                        tankDriveData.right.additionalFeedForward.getAsDouble()
                            - rightRampAccumulator) * Tuning.driveControlRamp))
            .then(getPipelineOutput()), this));
  }

  public Output<TankDriveData> getPipelineOutput() {
    return getPipelineOutput(true);
  }

  public Output<TankDriveData> getPipelineOutput(boolean useClosedLoop) {
    return new Output<>() {
      public void accept(@NotNull TankDriveData tankDriveData) {
        processSide(tankDriveData.left, driveLeftMotorA);
        processSide(tankDriveData.right, driveRightMotorA);
      }

      private void processSide(DriveData data, IMotorController controller) {
        if (data.position.isPresent() && useClosedLoop) {
          controller.selectProfileSlot(POSITION_SLOT_IDX, 0);
          if (data.additionalFeedForward.isPresent()) {
            controller.set(ControlMode.Position, data.position.getAsDouble(),
                DemandType.ArbitraryFeedForward, data.additionalFeedForward.getAsDouble());
          } else {
            controller.set(ControlMode.Position, data.position.getAsDouble());
          }
        } else if (data.velocity.isPresent() && useClosedLoop) {
          controller.selectProfileSlot(VELOCITY_SLOT_IDX, 0);
          if (data.additionalFeedForward.isPresent()) {
            controller.set(ControlMode.Velocity, data.velocity.getAsDouble(),
                DemandType.ArbitraryFeedForward, data.additionalFeedForward.getAsDouble());
          } else {
            controller.set(ControlMode.Velocity, data.velocity.getAsDouble());
          }
        } else {
          controller.set(ControlMode.PercentOutput, data.additionalFeedForward.orElse(0));
        }
      }
    };
  }

  private void configTalonsForPosition() {
    for (ChickenTalon t : driveMotorMasters) {
      t.selectProfileSlot(POSITION_SLOT_IDX);
    }
  }

  private void configTalonsForVelocity() {
    for (ChickenTalon t : driveMotorMasters) {
      t.selectProfileSlot(VELOCITY_SLOT_IDX);
    }
  }

  public void setPosition(double left, double right) {
    setLeftPosition(left);
    setRightPosition(right);
  }

  public void setLeftPosition(double position) {
    configTalonsForPosition();
    driveLeftMotorA.set(ControlMode.Position, position);
  }

  public void setRightPosition(double position) {
    configTalonsForPosition();
    driveRightMotorA.set(ControlMode.Position, position);
  }

  public void setPosition(double left, double right, double leftFeedForward,
      double rightFeedForward) {
    setLeftPosition(left, leftFeedForward);
    setRightPosition(right, rightFeedForward);
  }

  public void setLeftPosition(double position, double feedForward) {
    configTalonsForPosition();
    driveLeftMotorA
        .set(ControlMode.Position, position, DemandType.ArbitraryFeedForward, feedForward);
  }

  public void setRightPosition(double position, double feedForward) {
    configTalonsForPosition();
    driveRightMotorA
        .set(ControlMode.Position, position, DemandType.ArbitraryFeedForward, feedForward);
  }


  public void setVelocity(double left, double right) {
    setLeftVelocity(left);
    setRightVelocity(right);
  }

  public void setLeftVelocity(double velocity) {
    configTalonsForVelocity();
    driveLeftMotorA.set(ControlMode.Velocity, velocity);
  }

  public void setRightVelocity(double velocity) {
    configTalonsForVelocity();
    driveRightMotorA.set(ControlMode.Velocity, velocity);
  }

  public void setVelocity(double left, double right, double leftFeedForward,
      double rightFeedForward) {
    setLeftVelocity(left, leftFeedForward);
    setRightVelocity(right, rightFeedForward);
  }

  public void setLeftVelocity(double velocity, double feedForward) {
    configTalonsForVelocity();
    driveLeftMotorA
        .set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedForward);
  }

  public void setRightVelocity(double velocity, double feedForward) {
    configTalonsForVelocity();
    driveRightMotorA
        .set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedForward);
  }

  public void setThrottle(double left, double right) {
    setLeftThrottle(left);
    setRightThrottle(right);
  }

  public void setLeftThrottle(double throttle) {
    driveLeftMotorA.set(ControlMode.PercentOutput, throttle);
  }

  public void setRightThrottle(double throttle) {
    driveRightMotorA.set(ControlMode.PercentOutput, throttle);
  }

  public double getLeftPosition() {
    return driveLeftMotorA.getSelectedSensorVelocity();
  }

  public double getLeftVelocity() {
    return driveLeftMotorA.getSelectedSensorVelocity();
  }

  public double getLeftThrottle() {
    return driveLeftMotorA.getMotorOutputPercent();
  }

  public double getRightPosition() {
    return driveRightMotorA.getSelectedSensorVelocity();
  }

  public double getRightVelocity() {
    return driveRightMotorA.getSelectedSensorVelocity();
  }

  public double getRightThrottle() {
    return driveRightMotorA.getMotorOutputPercent();
  }

  public NetworkTableEntry getLeftPositionEntry() {
    return leftPositionEntry;
  }

  public NetworkTableEntry getRightPositionEntry() {
    return rightPositionEntry;
  }

  public NetworkTableEntry getLeftVelocityEntry() {
    return leftVelocityEntry;
  }

  public NetworkTableEntry getRightVelocityEntry() {
    return rightVelocityEntry;
  }

  public NetworkTableEntry getLeftThrottleEntry() {
    return leftThrottleEntry;
  }

  public NetworkTableEntry getRightThrottleEntry() {
    return rightThrottleEntry;
  }

  public NetworkTableEntry getLeftCurrentAEntry() {
    return leftCurrentAEntry;
  }

  public NetworkTableEntry getLeftCurrentBEntry() {
    return leftCurrentBEntry;
  }

  public NetworkTableEntry getLeftCurrentCEntry() {
    return leftCurrentCEntry;
  }

  public NetworkTableEntry getRightCurrentAEntry() {
    return rightCurrentAEntry;
  }

  public NetworkTableEntry getRightCurrentBEntry() {
    return rightCurrentBEntry;
  }

  public NetworkTableEntry getRightCurrentCEntry() {
    return rightCurrentCEntry;
  }

  @Override
  public void periodic() {
    leftPositionEntry.setNumber(getLeftPosition());
    rightPositionEntry.setNumber(getRightPosition());

    leftVelocityEntry.setNumber(getLeftVelocity());
    rightVelocityEntry.setNumber(getRightVelocity());

    leftThrottleEntry.setNumber(getLeftThrottle());
    rightThrottleEntry.setNumber(getRightThrottle());

    leftCurrentAEntry.setNumber(driveLeftMotorA.getOutputCurrent());
    leftCurrentBEntry.setNumber(driveLeftMotorB.getOutputCurrent());
    leftCurrentCEntry.setNumber(driveLeftMotorC.getOutputCurrent());

    rightCurrentAEntry.setNumber(driveRightMotorA.getOutputCurrent());
    rightCurrentBEntry.setNumber(driveRightMotorB.getOutputCurrent());
    rightCurrentCEntry.setNumber(driveRightMotorC.getOutputCurrent());

    if (DriverStation.getInstance().isDisabled()) {
      leftRampAccumulator = 0;
      rightRampAccumulator = 0;
    }
  }

  public void checkStickyFaults() {
    PhineasUtilities.processStickyFaults("Drivetrain", "left A", driveLeftMotorA);
    PhineasUtilities.processStickyFaults("Drivetrain", "left B", driveLeftMotorB);
    PhineasUtilities.processStickyFaults("Drivetrain", "left C", driveLeftMotorC);
    PhineasUtilities.processStickyFaults("Drivetrain", "right A", driveRightMotorA);
    PhineasUtilities.processStickyFaults("Drivetrain", "right B", driveRightMotorB);
    PhineasUtilities.processStickyFaults("Drivetrain", "right C", driveRightMotorC);
  }
}

