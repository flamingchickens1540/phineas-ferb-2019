package org.team1540.robot2019.subsystems;

import static org.team1540.robot2019.Hardware.armActuator;

import edu.wpi.first.wpilibj.command.Subsystem;

public class Arm extends Subsystem {

  public void moveDown() {
    armActuator.set(true);
  }

  public void moveUp() {
    armActuator.set(false);
  }

  @Override
  protected void initDefaultCommand() {

  }
}
