package org.team1540.robot2019.subsystems;

import static org.team1540.robot2019.Hardware.elevatorBrake;
import static org.team1540.robot2019.Hardware.elevatorBtmSwitch;
import static org.team1540.robot2019.Hardware.elevatorA;
import static org.team1540.robot2019.Hardware.elevatorB;
import static org.team1540.robot2019.Hardware.elevatorTopSwitch;

import edu.wpi.first.wpilibj.command.Subsystem;
import org.team1540.robot2019.Tuning;

public class Elevator extends Subsystem {
  public Elevator() {
    elevatorA.setInverted(Tuning.invertElevatorA);
    elevatorB.follow(elevatorA, Tuning.invertElevatorB);
  }

  @Override
  protected void initDefaultCommand() {

  }

  public void startMovingUp() {
    elevatorBrake.set(false);
    elevatorA.set(Tuning.elevatorUpSpeed);
  }

  public void startMovingDown() {
    elevatorBrake.set(false);
    elevatorA.set(-Tuning.elevatorDownSpeed);
  }

  public void stop() {
    elevatorBrake.set(true);
    elevatorA.set(0);
  }

  public boolean isAtTop() {
    return elevatorTopSwitch.get();
  }

  public boolean isAtBottom() {
    return elevatorBtmSwitch.get();
  }
}
