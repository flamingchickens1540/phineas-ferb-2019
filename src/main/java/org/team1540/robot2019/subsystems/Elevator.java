package org.team1540.robot2019.subsystems;

import static org.team1540.robot2019.Hardware.elevatorA;
import static org.team1540.robot2019.Hardware.elevatorB;
import static org.team1540.robot2019.Hardware.elevatorLimitSensor;
import static org.team1540.robot2019.Tuning.elevatorInPerRotation;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.ControlType;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.apache.log4j.Logger;
import org.team1540.robot2019.Tuning;

public class Elevator extends Subsystem {

  private static final Logger logger = Logger.getLogger(Elevator.class);

  private NetworkTable table = NetworkTableInstance.getDefault().getTable("elevator");

  private NetworkTableEntry positionEntry = table.getEntry("pos");

  private NetworkTableEntry velocityEntry = table.getEntry("vel");

  private NetworkTableEntry throttleEntry = table.getEntry("throt");

  private NetworkTableEntry currentAEntry = table.getEntry("currA");
  private NetworkTableEntry currentBEntry = table.getEntry("currB");

  private NetworkTableEntry targetPosEntry = table.getEntry("tgtPos");
  private NetworkTableEntry targetVelEntry = table.getEntry("tgtVel");

  private NetworkTableEntry limEntry = table.getEntry("limit");
  private NetworkTableEntry offsetEntry = table.getEntry("offset");

  private double positionOffset;

  @Override
  protected void initDefaultCommand() {
    // no default command - the move to position command leaves the elevator PID on and at the
    // setpoint, and the zero command stops the motors when it finishes.
  }

  public boolean isAtLimit() {
    return elevatorLimitSensor.get();
  }

  public void setPosition(double position) {
    double positionRaw = (position / elevatorInPerRotation) + positionOffset;
    logger.debug("Setting elevator to raw position " + positionRaw);
    elevatorA.getPIDController()
        .setReference(positionRaw, ControlType.kPosition, 0,
            Tuning.elevatorStaticFeedForward);
  }

  public void zero() {
    positionOffset = elevatorA.getEncoder().getPosition();
  }

  public void setRaw(double throttle) {
    elevatorA.set(throttle);
  }

  public double getVelocity() {
    return elevatorA.getEncoder().getVelocity() * (elevatorInPerRotation / 60);
  }

  public double getThrottle() {
    return elevatorA.getAppliedOutput();
  }

  public double getVoltage() {
    return elevatorA.getBusVoltage() * elevatorA.getAppliedOutput();
  }

  public double getCurrentA() {
    return elevatorA.getOutputCurrent();
  }

  public double getCurrentB() {
    return elevatorB.getOutputCurrent();
  }

  @Override
  public void periodic() {
    positionEntry.forceSetNumber(getPosition());
    velocityEntry.forceSetNumber(getVelocity());

    throttleEntry.forceSetNumber(elevatorA.getAppliedOutput());

    currentAEntry.forceSetNumber(elevatorA.getOutputCurrent());
    currentBEntry.forceSetNumber(elevatorB.getOutputCurrent());

    offsetEntry.forceSetNumber(getOffset());

    limEntry.forceSetBoolean(isAtLimit());
  }

  public double getPosition() {
    return -(elevatorA.getEncoder().getPosition() * elevatorInPerRotation)
        + positionOffset;
  }

  public double getOffset() {
    return positionOffset;
  }

  public void setBrake(boolean brake) {
    IdleMode mode = brake ? IdleMode.kBrake : IdleMode.kCoast;
    elevatorA.setIdleMode(mode);
    elevatorB.setIdleMode(mode);
  }

  public void stop() {
    setRaw(0);
  }
}
