package org.team1540.robot2019.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.team1540.robot2019.RobotMap;

public class Arm extends Subsystem {

  // solenoid on is arm down
  private Solenoid actuator = new Solenoid(RobotMap.ARM_ACTUATOR);

  public void moveDown() {
    actuator.set(true);
  }

  public void moveUp() {
    actuator.set(false);
  }

  @Override
  protected void initDefaultCommand() {

  }
}
