package org.team1540.robot2019.subsystems;

import static org.team1540.robot2019.Hardware.armBtmSwitch;
import static org.team1540.robot2019.Hardware.armCylinder;
import static org.team1540.robot2019.Hardware.armMotor;
import static org.team1540.robot2019.Hardware.armTopSwitch;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Arm extends Subsystem {

  public void moveDown() {
    armCylinder.set(true);
    armMotor.set(ControlMode.PercentOutput, 1);
  }

  public void moveUp() {
    armCylinder.set(false);
    armMotor.set(ControlMode.PercentOutput, -1);
  }

  public void stop() {
    armMotor.set(ControlMode.PercentOutput, 0);
  }

  public boolean isAtTop() {
    return armTopSwitch.get();
  }

  public boolean isAtBtm() {
    return armBtmSwitch.get();
  }

  @Override
  protected void initDefaultCommand() {

  }
}
