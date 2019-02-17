package org.team1540.robot2019.subsystems;

import static org.team1540.robot2019.Hardware.DRIVE_POSITION_SLOT_IDX;
import static org.team1540.robot2019.Hardware.DRIVE_VELOCITY_SLOT_IDX;
import static org.team1540.robot2019.Hardware.driveLeftMotorA;
import static org.team1540.robot2019.Hardware.driveMotorMasters;
import static org.team1540.robot2019.Hardware.driveRightMotorA;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.IMotorController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.apache.log4j.Logger;
import org.jetbrains.annotations.NotNull;
import org.team1540.robot2019.Hardware;
import org.team1540.robot2019.OI;
import org.team1540.robot2019.Tuning;
import org.team1540.rooster.drive.pipeline.AdvancedArcadeJoystickInput;
import org.team1540.rooster.drive.pipeline.DriveData;
import org.team1540.rooster.drive.pipeline.FeedForwardProcessor;
import org.team1540.rooster.drive.pipeline.FeedForwardToVelocityProcessor;
import org.team1540.rooster.drive.pipeline.TankDriveData;
import org.team1540.rooster.functional.Output;
import org.team1540.rooster.util.SimpleLoopCommand;
import org.team1540.rooster.wrappers.ChickenController;
import org.team1540.rooster.wrappers.ChickenTalon;

public class Drivetrain extends Subsystem {

  public static final Logger logger = Logger.getLogger(Drivetrain.class);

  double leftRampAccum;
  double rightRampAccum;

  boolean inFineDrive = false;

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

  @Override
  protected void initDefaultCommand() {
    setDefaultCommand(new SimpleLoopCommand("Drive",
        new AdvancedArcadeJoystickInput(true, OI::getDriveThrottle, OI::getDriveSoftTurn,
            OI::getDriveHardTurn)
            .then(new FeedForwardToVelocityProcessor(Tuning.driveMaxVel))
            .then(new FeedForwardProcessor(Tuning.driveKV, Tuning.driveVIntercept, 0))
            .then(getPipelineOutput(false)), this));
  }

  private static double calcRamp(double throttle, double rampAccum) {
    double ramp;
    if (throttle == 0) {
      if (rampAccum < -Tuning.driveControlRampDown) {
        ramp = Tuning.driveControlRampDown;
      } else if (rampAccum <= 0) {
        ramp = rampAccum; // this will make the accumulator 0
      } else if (rampAccum < Tuning.driveControlRampDown) {
        ramp = -rampAccum;
      } else {
        ramp = -Tuning.driveControlRampDown;
      }
    } else if (throttle < 0) {
      if (rampAccum < throttle - Tuning.driveControlRampDown) {
        ramp = Tuning.driveControlRampDown;
      } else if (rampAccum < throttle + Tuning.driveControlRampUp) {
        ramp = rampAccum - throttle;
      } else if (rampAccum <= 0) {
        ramp = -Tuning.driveControlRampUp;
      } else {
        ramp = -Tuning.driveControlRampDown;
      }
    } else { // throttle is greater than 0
      if (rampAccum > throttle + Tuning.driveControlRampDown) {
        ramp = -Tuning.driveControlRampDown;
      } else if (rampAccum > throttle - Tuning.driveControlRampUp) {
        ramp = rampAccum - throttle;
      } else if (rampAccum >= 0) {
        ramp = Tuning.driveControlRampUp;
      } else {
        ramp = Tuning.driveControlRampDown;
      }
    }
    return ramp;
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
          controller.selectProfileSlot(DRIVE_POSITION_SLOT_IDX, 0);
          if (data.additionalFeedForward.isPresent()) {
            controller.set(ControlMode.Position, data.position.getAsDouble(),
                DemandType.ArbitraryFeedForward, data.additionalFeedForward.getAsDouble());
          } else {
            controller.set(ControlMode.Position, data.position.getAsDouble());
          }
        } else if (data.velocity.isPresent() && useClosedLoop) {
          controller.selectProfileSlot(DRIVE_VELOCITY_SLOT_IDX, 0);
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
      t.selectProfileSlot(DRIVE_POSITION_SLOT_IDX);
    }
  }

  private void configTalonsForVelocity() {
    for (ChickenTalon t : driveMotorMasters) {
      t.selectProfileSlot(DRIVE_VELOCITY_SLOT_IDX);
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

  public void setBrake(boolean brake) {
    for (ChickenController talon : driveMotorMasters) {
      talon.setBrake(brake);
    }
  }

  public double getLeftPosition() {
    return driveLeftMotorA.getSelectedSensorPosition();
  }

  public double getLeftVelocity() {
    return driveLeftMotorA.getSelectedSensorVelocity();
  }

  public double getLeftThrottle() {
    return driveLeftMotorA.getMotorOutputPercent();
  }

  public double getRightPosition() {
    return driveRightMotorA.getSelectedSensorPosition();
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
    if (Tuning.publishData) {
      leftPositionEntry.setNumber(getLeftPosition());
      rightPositionEntry.setNumber(getRightPosition());

      leftVelocityEntry.setNumber(getLeftVelocity());
      rightVelocityEntry.setNumber(getRightVelocity());

      leftThrottleEntry.setNumber(getLeftThrottle());
      rightThrottleEntry.setNumber(getRightThrottle());

      leftCurrentAEntry.setNumber(Hardware.getDriveLeftACurrent());
      leftCurrentBEntry.setNumber(Hardware.getDriveLeftBCurrent());
      leftCurrentCEntry.setNumber(Hardware.getDriveLeftCCurrent());

      rightCurrentAEntry.setNumber(Hardware.getDriveRightACurrent());
      rightCurrentBEntry.setNumber(Hardware.getDriveRightBCurrent());
      rightCurrentCEntry.setNumber(Hardware.getDriveRightCCurrent());
    }
    if (DriverStation.getInstance().isDisabled()) {
      leftRampAccum = 0;
      rightRampAccum = 0;
    }

    if (OI.getDriveFine() && !inFineDrive) {
      logger.debug("Fine drive engaged");
      inFineDrive = true;
      Hardware.driveLeftMotorA.configOpenloopRamp(0);
      Hardware.driveRightMotorA.configOpenloopRamp(0);
    } else if (!OI.getDriveFine() && inFineDrive) {
      logger.debug("Fine drive disengaged");
      inFineDrive = false;
      Hardware.driveLeftMotorA.configOpenloopRamp(Tuning.driveOpenLoopRamp);
      Hardware.driveRightMotorA.configOpenloopRamp(Tuning.driveOpenLoopRamp);
    }
  }
}

