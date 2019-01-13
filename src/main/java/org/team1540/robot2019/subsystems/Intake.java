package org.team1540.robot2019.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.team1540.robot2019.PhineasUtilities;
import org.team1540.robot2019.RobotMap;
import org.team1540.robot2019.Tuning;
import org.team1540.rooster.wrappers.ChickenVictor;

public class Intake extends Subsystem {

  // positive setpoint is outtaking
  // victor features are a subset of talons so this can be changed later
  private ChickenVictor top = new ChickenVictor(RobotMap.INTAKE_TOP);
  private ChickenVictor btm = new ChickenVictor(RobotMap.INTAKE_BTM);

  // TODO add specific sensor

  public Intake() {
    PhineasUtilities.processStickyFaults("Intake", "top", top);
    PhineasUtilities.processStickyFaults("Intake", "bottom", btm);

    top.configFactoryDefault();
    btm.configFactoryDefault();

    top.setInverted(Tuning.intakeInvertTop);
    btm.setInverted(Tuning.intakeInvertBtm);

    top.setBrake(true);
    btm.setBrake(true);
  }

  @Override
  protected void initDefaultCommand() {

  }

  public void startIntaking() {
    top.set(ControlMode.PercentOutput, -Tuning.intakeIntakeSpeedTop);
    btm.set(ControlMode.PercentOutput, -Tuning.intakeIntakeSpeedBtm);
  }

  public void startEjecting() {
    top.set(ControlMode.PercentOutput, Tuning.intakeEjectSpeedTop);
    btm.set(ControlMode.PercentOutput, Tuning.intakeEjectSpeedBtm);
  }

  public void stop() {
    top.set(ControlMode.PercentOutput, 0);
    btm.set(ControlMode.PercentOutput, 0);
  }

  public boolean hasBall() {
    // TODO placeholder
    return false;
  }
}
