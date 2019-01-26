package org.team1540.robot2019.subsystems;

import static org.team1540.robot2019.Hardware.wristBtmSwitch;
import static org.team1540.robot2019.Hardware.wristCylinder;
import static org.team1540.robot2019.Hardware.wristMotor;
import static org.team1540.robot2019.Hardware.wristTopSwitch;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Wrist extends Subsystem {

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

  public boolean isAtTop() {
    return wristTopSwitch.get();
  }

  public boolean isAtBtm() {
    return wristBtmSwitch.get();
  }

  @Override
  protected void initDefaultCommand() {

  }
}
