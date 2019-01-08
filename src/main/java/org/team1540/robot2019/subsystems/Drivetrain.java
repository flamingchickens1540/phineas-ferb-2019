package org.team1540.robot2019.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.IMotorController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.jetbrains.annotations.NotNull;
import org.team1540.robot2019.RobotMap;
import org.team1540.robot2019.Tuning;
import org.team1540.robot2019.Utilities;
import org.team1540.rooster.drive.pipeline.DriveData;
import org.team1540.rooster.drive.pipeline.Output;
import org.team1540.rooster.drive.pipeline.TankDriveData;
import org.team1540.rooster.wrappers.ChickenTalon;

public class Drivetrain extends Subsystem {

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

    checkStickyFaults();

    for (ChickenTalon talon : driveMotorAll) {
      talon.configFactoryDefault();
      // TODO: configure ramping, etc
    }

    for (ChickenTalon talon : driveMotorMasters) {
      talon.config_kP(0, Tuning.drivePositionP);
      talon.config_kI(0, Tuning.drivePositionI);
      talon.config_kD(0, Tuning.drivePositionD);
      talon.config_kF(0, Tuning.drivePositionF);
      talon.config_kP(1, Tuning.driveVelocityP);
      talon.config_kI(1, Tuning.driveVelocityI);
      talon.config_kD(1, Tuning.driveVelocityD);
      talon.config_kF(1, Tuning.driveVelocityF);
    }
  }

  @Override
  protected void initDefaultCommand() {
    // TODO: set up drive code
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
          controller.selectProfileSlot(0, 0);
          if (data.additionalFeedForward.isPresent()) {
            controller.set(ControlMode.Position, data.position.getAsDouble(),
                DemandType.ArbitraryFeedForward, data.additionalFeedForward.getAsDouble());
          } else {
            controller.set(ControlMode.Position, data.position.getAsDouble());
          }
        } else if (data.velocity.isPresent() && useClosedLoop) {
          controller.selectProfileSlot(1, 0);
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
      t.selectProfileSlot(0);
    }
  }

  private void configTalonsForVelocity() {
    for (ChickenTalon t : driveMotorMasters) {
      t.selectProfileSlot(1);
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

  public void setPosition(double left, double right, double leftFeedForward, double rightFeedForward) {
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
  }

  public void checkStickyFaults() {
    Utilities.processStickyFaults("Drivetrain", "left A", driveLeftMotorA);
    Utilities.processStickyFaults("Drivetrain", "left B", driveLeftMotorB);
    Utilities.processStickyFaults("Drivetrain", "left C", driveLeftMotorC);
    Utilities.processStickyFaults("Drivetrain", "right A", driveRightMotorA);
    Utilities.processStickyFaults("Drivetrain", "right B", driveRightMotorB);
    Utilities.processStickyFaults("Drivetrain", "right C", driveRightMotorC);
  }

  public Sendable getDifferentialDriveSendable() {
    return new SendableBase() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("DifferentialDrive");
        builder.setActuator(true);
        builder.setSafeState(Drivetrain.this::stop);
        builder.addDoubleProperty(
            "Left Motor Speed",
            Drivetrain.this::getLeftThrottle,
            Drivetrain.this::setLeftThrottle);
        builder.addDoubleProperty(
            "Right Motor Speed",
            Drivetrain.this::getRightThrottle,
            Drivetrain.this::setRightThrottle);
      }
    };
  }
}

