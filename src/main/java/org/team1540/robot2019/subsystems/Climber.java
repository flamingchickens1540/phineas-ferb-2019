package org.team1540.robot2019.subsystems;

import static org.team1540.robot2019.Hardware.climberArmLeft;
import static org.team1540.robot2019.Hardware.climberArmRight;
import static org.team1540.robot2019.Hardware.climberCylinder1;
import static org.team1540.robot2019.Hardware.climberCylinder2;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Climber extends Subsystem {

  public void cylinderDown() {
    climberCylinder1.set(true);
    climberCylinder2.set(false);
  }

  public void cylinderUp() {
    climberCylinder1.set(false);
    climberCylinder2.set(true);
  }

  public void setArms(double value) {
    climberArmLeft.set(ControlMode.PercentOutput, -value);
//        climberArmRight.set(ControlMode.PercentOutput, value);
  }

  public double getCurrentLeft() {
    return climberArmLeft.getOutputCurrent();
  }

  public double getCurrentRight() {
    return climberArmRight.getOutputCurrent();
  }

  @Override
  protected void initDefaultCommand() {
  }

}
