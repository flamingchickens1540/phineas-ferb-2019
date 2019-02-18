package org.team1540.robot2019.subsystems;

import static org.team1540.robot2019.Hardware.wristBtmSwitch;
import static org.team1540.robot2019.Hardware.wristMidSwitch;
import static org.team1540.robot2019.Hardware.wristMotor;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.InterruptHandlerFunction;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.apache.log4j.Level;
import org.apache.log4j.Logger;
import org.squirrelframework.foundation.fsm.StateMachineLogger;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.commands.wrist.WristUpOrHold;

public class Wrist extends Subsystem {

  private static final Logger logger = Logger.getLogger(Wrist.class);

  private volatile boolean btmFlag = false;
  private volatile boolean midFlag = false;

  private NetworkTable table = NetworkTableInstance.getDefault().getTable("wrist");

  private NetworkTableEntry isAtMidEntry = table.getEntry("atMid");
  private NetworkTableEntry isAtBtmEntry = table.getEntry("atBtm");
  private NetworkTableEntry motorEntry = table.getEntry("motorThrot");

  public Wrist() {
    // configure state machine logger
    Logger.getLogger(StateMachineLogger.class)
        .setLevel(Level.INFO); // this can be changed as needed

    wristMidSwitch.requestInterrupts(new InterruptHandlerFunction<>() {
      @Override
      public void interruptFired(int i, Object o) {
        Wrist.logger.debug("Mid Switch Interrupt");
        midFlag = true;
      }
    });

    wristBtmSwitch.requestInterrupts(new InterruptHandlerFunction<>() {
      @Override
      public void interruptFired(int i, Object o) {
        Wrist.logger.debug("Btm Switch Interrupt");
        btmFlag = true;
      }
    });

    wristMidSwitch.setUpSourceEdge(false, true);

    wristBtmSwitch.setUpSourceEdge(false, true);

    wristMidSwitch.enableInterrupts();
    wristBtmSwitch.enableInterrupts();
  }

  public void set(double throttle) {
    wristMotor.set(ControlMode.PercentOutput, throttle);
  }

  public boolean isAtMid() {
    return !wristMidSwitch.get();
  }

  public boolean isAtBtm() {
    return !wristBtmSwitch.get();
  }

  public boolean getBtmFlag() {
    return btmFlag;
  }

  public boolean getMidFlag() {
    return midFlag;
  }

  public boolean clearBtmFlag() {
    if (btmFlag) {
      btmFlag = false;
      return true;
    } else {
      return false;
    }
  }

  public boolean clearMidFlag() {
    if (midFlag) {
      midFlag = false;
      return true;
    } else {
      return false;
    }
  }

  @Override
  protected void initDefaultCommand() {
    /*
    The command logic for this subsystem exploits various weird behaviors in how WPILib handles
    command groups to ensure that the wrist is only ever down when we are doing something with it.
    Essentially, if WristDown is used in a command group (such as IntakeSequence), that command
    group "blocks" the wrist (i.e. prevents the default WristUpOrHold from running) for the entire
    duration of the command group, not just the portions where wrist-related commands are actually
    used. As soon as the command group finishes, the default command engages and moves the wrist up.
     */
    setDefaultCommand(new WristUpOrHold());
  }

  @Override
  public void periodic() {
    if (Robot.debugMode) {
      isAtMidEntry.forceSetBoolean(isAtMid());
      isAtBtmEntry.forceSetBoolean(isAtBtm());
      motorEntry.forceSetNumber(wristMotor.getMotorOutputPercent());
    }
  }

  public void setBrake(boolean brake) {
    wristMotor.setBrake(brake);
  }
}
