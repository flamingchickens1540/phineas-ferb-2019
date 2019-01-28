package org.team1540.robot2019.subsystems;

import static org.team1540.robot2019.Hardware.wristBtmSwitch;
import static org.team1540.robot2019.Hardware.wristCylinder;
import static org.team1540.robot2019.Hardware.wristMotor;
import static org.team1540.robot2019.Hardware.wristTopSwitch;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Wrist extends Subsystem {

  private NetworkTable table = NetworkTableInstance.getDefault().getTable("wrist");

  private NetworkTableEntry isAtTopEntry = table.getEntry("atTop");
  private NetworkTableEntry isAtBtmEntry = table.getEntry("atBtm");
  private NetworkTableEntry cylinderEntry = table.getEntry("cylinder");
  private NetworkTableEntry motorEntry = table.getEntry("motorThrot");

  public void moveDown() {
    wristCylinder.set(true);
    wristMotor.set(ControlMode.PercentOutput, 1);
  }

  public void moveUp() {
    wristCylinder.set(false);
    wristMotor.set(ControlMode.PercentOutput, -1);
  }

  public void stop() {
    wristMotor.set(ControlMode.PercentOutput, 0);
  }

  public void set(double throttle) {
    wristMotor.set(ControlMode.PercentOutput, throttle);
  }

  public void setCylinder(boolean value) {
    wristCylinder.set(value);
  }

  public boolean isAtTop() {
    return wristTopSwitch.get();
  }

  public boolean isAtBtm() {
    return wristBtmSwitch.get();
  }

  @Override
  protected void initDefaultCommand() {

  }

  @Override
  public void periodic() {
    isAtTopEntry.forceSetBoolean(isAtTop());
    isAtBtmEntry.forceSetBoolean(isAtBtm());
    cylinderEntry.forceSetBoolean(wristCylinder.get());
    motorEntry.forceSetNumber(wristMotor.getMotorOutputPercent());
  }
}
