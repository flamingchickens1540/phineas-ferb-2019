package org.team1540.robot2019.subsystems;

import static org.team1540.robot2019.Hardware.climberArmLeft;
import static org.team1540.robot2019.Hardware.climberCylinder;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.team1540.robot2019.Hardware;
import org.team1540.robot2019.Tuning;

public class Climber extends Subsystem {

  private NetworkTable table = NetworkTableInstance.getDefault().getTable("climber");
  private NetworkTableEntry posEntry = table.getEntry("pos");

  public void cylinderDown() {
    climberCylinder.set(Value.kReverse);
  }

  public void cylinderUp() {
    climberCylinder.set(Value.kForward);
  }

  public void setArms(double value) {
    climberArmLeft.set(ControlMode.PercentOutput, value);
//        climberArmRight.set(ControlMode.PercentOutput, value);
  }

  public void setArmPosition(double pos) {
    climberArmLeft.set(ControlMode.MotionMagic, pos);
  }

  public void setArmsConstant(double speed) {
    climberArmLeft.set(ControlMode.Velocity, speed);
  }

  public void startClimbing() {
    cylinderDown();
    setArmsConstant(Tuning.climberArmSpeed);
  }

  public void onPlatform() {
    cylinderUp();
    setArmsConstant(Tuning.climberArmHoldSpeed);
  }

  public double getCurrentLeft() {
    return Hardware.getClimberLCurrent();
  }

  public double getCurrentRight() {
    return Hardware.getClimberRCurrent();
  }

  @Override
  protected void initDefaultCommand() {
  }

  public double getPosition() {
    return climberArmLeft.getSelectedSensorPosition();
  }

  @Override
  public void periodic() {
    posEntry.forceSetNumber(getPosition());
  }

  public void setArmBrake(boolean brake) {
    climberArmLeft.setBrake(brake);
    climberArmRight.setBrake(brake);
  }
}
