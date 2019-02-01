package org.team1540.robot2019.subsystems;

import static org.team1540.robot2019.Hardware.elevatorA;
import static org.team1540.robot2019.Hardware.elevatorB;
import static org.team1540.robot2019.Hardware.elevatorLimitSensor;
import static org.team1540.robot2019.Tuning.inPerRotation;

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


  public Elevator() {
    elevatorA.setInverted(Tuning.invertElevatorA);
    elevatorB.follow(elevatorA, Tuning.invertElevatorB);
  }

  @Override
  protected void initDefaultCommand() {

  }

  public boolean isAtLimit() {
    return elevatorLimitSensor.get();
  }

  public void setPosition(double position) {
    elevatorA.getPIDController()
        .setReference((position / inPerRotation) + positionOffset, ControlType.kPosition, 0,
            Tuning.elevatorStaticFeedForward);
  }

  public void zero() {
    positionOffset = elevatorA.getEncoder().getPosition();
  }

  public void setRaw(double throttle) {
    elevatorA.set(throttle);
  }

  public double getVelocity() {
    return elevatorA.getEncoder().getVelocity() * (inPerRotation / 60);
  }

  public double getThrottle() {
    return elevatorA.getAppliedOutput();
  }

  public double getVoltage() {
    return elevatorA.getBusVoltage() * elevatorA.getAppliedOutput();
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
    return (elevatorA.getEncoder().getPosition() * inPerRotation)
        + positionOffset;
  }

  public double getOffset() {
    return positionOffset;
  }
}
