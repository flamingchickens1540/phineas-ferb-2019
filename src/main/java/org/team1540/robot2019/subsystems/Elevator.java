package org.team1540.robot2019.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.team1540.robot2019.RobotMap;
import org.team1540.robot2019.Tuning;

public class Elevator extends Subsystem {

  // positive setpoint is moving up
  private CANSparkMax motorA = new CANSparkMax(RobotMap.ELEVATOR_A, MotorType.kBrushless);
  private CANSparkMax motorB = new CANSparkMax(RobotMap.ELEVATOR_B, MotorType.kBrushless);

  private Solenoid brake = new Solenoid(RobotMap.ELEVATOR_BRAKE);

  private DigitalInput topSwitch = new DigitalInput(RobotMap.ELEVATOR_TOP_SW);
  private DigitalInput btmSwitch = new DigitalInput(RobotMap.ELEVATOR_BTM_SW);

  public Elevator() {
    motorA.setInverted(Tuning.invertElevatorA);
    motorB.follow(motorA, Tuning.invertElevatorB);
  }

  @Override
  protected void initDefaultCommand() {

  }

  public void startMovingUp() {
    brake.set(false);
    motorA.set(Tuning.elevatorUpSpeed);
  }

  public void startMovingDown() {
    brake.set(false);
    motorA.set(-Tuning.elevatorDownSpeed);
  }

  public void stop() {
    brake.set(true);
    motorA.set(0);
  }

  public boolean isAtTop() {
    return topSwitch.get();
  }

  public boolean isAtBottom() {
    return topSwitch.get();
  }
}
