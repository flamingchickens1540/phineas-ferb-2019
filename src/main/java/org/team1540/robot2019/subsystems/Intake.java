package org.team1540.robot2019.subsystems;

import static org.team1540.robot2019.Hardware.intakeBtm;
import static org.team1540.robot2019.Hardware.intakeTop;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.team1540.robot2019.Tuning;

public class Intake extends Subsystem {

  @Override
  protected void initDefaultCommand() {

  }

  public void startIntaking() {
    intakeTop.set(ControlMode.PercentOutput, -Tuning.intakeIntakeSpeedTop);
    intakeBtm.set(ControlMode.PercentOutput, -Tuning.intakeIntakeSpeedBtm);
  }

  public void startEjecting() {
    intakeTop.set(ControlMode.PercentOutput, Tuning.intakeEjectSpeedTop);
    intakeBtm.set(ControlMode.PercentOutput, Tuning.intakeEjectSpeedBtm);
  }

  public void stop() {
    intakeTop.set(ControlMode.PercentOutput, 0);
    intakeBtm.set(ControlMode.PercentOutput, 0);
  }

  public boolean hasBall() {
    // TODO placeholder
    return false;
  }
}
