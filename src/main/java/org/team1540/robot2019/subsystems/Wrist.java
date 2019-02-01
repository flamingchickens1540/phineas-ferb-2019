package org.team1540.robot2019.subsystems;

import static org.team1540.robot2019.Hardware.wristBtmSwitch;
import static org.team1540.robot2019.Hardware.wristCylinder;
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
import org.squirrelframework.foundation.fsm.StateMachineBuilderFactory;
import org.squirrelframework.foundation.fsm.StateMachineLogger;
import org.squirrelframework.foundation.fsm.UntypedStateMachine;
import org.squirrelframework.foundation.fsm.UntypedStateMachineBuilder;
import org.squirrelframework.foundation.fsm.annotation.StateMachineParameters;
import org.squirrelframework.foundation.fsm.impl.AbstractUntypedStateMachine;
import org.team1540.robot2019.Tuning;

public class Wrist extends Subsystem {

  private static final Logger logger = Logger.getLogger(Wrist.class);

  private NetworkTable table = NetworkTableInstance.getDefault().getTable("wrist");

  private NetworkTableEntry isAtMidEntry = table.getEntry("atMid");
  private NetworkTableEntry isAtBtmEntry = table.getEntry("atBtm");
  private NetworkTableEntry currentStateEntry = table.getEntry("state");
  private NetworkTableEntry motorEntry = table.getEntry("motorThrot");

  public UntypedStateMachine stateMachine;

  public Wrist() {
    UntypedStateMachineBuilder builder = StateMachineBuilderFactory.create(WristStateMachine.class);

    builder.onEntry(WristState.OFF_UP).callMethod("stop");
    builder.onEntry(WristState.OFF_DOWN).callMethod("stop");
    builder.onEntry(WristState.DOWN_TRAVEL_POWER).callMethod("downTravelPwr");
    builder.onEntry(WristState.DOWN_TRAVEL_BRAKE).callMethod("downTravelBrake");
    builder.onEntry(WristState.UP_TRAVEL).callMethod("upTravel");

    builder.externalTransition().from(WristState.OFF_UP).to(WristState.DOWN_TRAVEL_POWER)
        .on(WristEvent.DOWN_CMD);
    builder.externalTransition().from(WristState.OFF_DOWN).to(WristState.UP_TRAVEL)
        .on(WristEvent.UP_CMD);

    builder.externalTransition().from(WristState.DOWN_TRAVEL_POWER).to(WristState.DOWN_TRAVEL_BRAKE)
        .on(WristEvent.MID_SENSOR);
    builder.externalTransition().from(WristState.UP_TRAVEL).to(WristState.OFF_UP)
        .on(WristEvent.MID_SENSOR);

    builder.externalTransition().from(WristState.DOWN_TRAVEL_BRAKE).to(WristState.OFF_DOWN)
        .on(WristEvent.BTM_SENSOR);

    builder.externalTransition().from(WristState.OFF_UP).to(WristState.OFF_DOWN)
        .on(WristEvent.BTM_SENSOR).callMethod("stop");
    builder.externalTransition().from(WristState.OFF_DOWN).to(WristState.OFF_UP)
        .on(WristEvent.MID_SENSOR).callMethod("stop");

    // disabling-caused transitions

    builder.externalTransition().from(WristState.DOWN_TRAVEL_POWER).to(WristState.OFF_UP)
        .on(WristEvent.DISABLE);
    builder.externalTransition().from(WristState.DOWN_TRAVEL_BRAKE).to(WristState.OFF_DOWN)
        .on(WristEvent.DISABLE);
    builder.externalTransition().from(WristState.UP_TRAVEL).to(WristState.OFF_DOWN)
        .on(WristEvent.DISABLE);

    stateMachine = builder
        .newStateMachine(wristBtmSwitch.get() ? WristState.OFF_DOWN : WristState.OFF_UP);
    stateMachine.start();

    // configure state machine logger
    Logger.getLogger(StateMachineLogger.class)
        .setLevel(Level.INFO); // this can be changed as needed

    StateMachineLogger logger = new StateMachineLogger(stateMachine);
    logger.startLogging();

    wristMidSwitch.requestInterrupts(new InterruptHandlerFunction<>() {
      @Override
      public void interruptFired(int i, Object o) {
        Wrist.logger.debug("Mid Switch Interrupt");
        stateMachine.fire(WristEvent.MID_SENSOR);
      }
    });

    wristBtmSwitch.requestInterrupts(new InterruptHandlerFunction<>() {
      @Override
      public void interruptFired(int i, Object o) {
        Wrist.logger.debug("Btm Switch Interrupt");
        stateMachine.fire(WristEvent.BTM_SENSOR);
      }
    });

    wristMidSwitch.setUpSourceEdge(false, true);

    wristBtmSwitch.setUpSourceEdge(false, true);

    wristMidSwitch.enableInterrupts();
    wristBtmSwitch.enableInterrupts();
  }

  public void moveDown() {
    stateMachine.fire(WristEvent.DOWN_CMD);
  }

  public void moveUp() {
    stateMachine.fire(WristEvent.UP_CMD);
  }

  public WristState getState() {
    return (WristState) stateMachine.getCurrentState();
  }

  public void handleDisable() {
    stateMachine.fire(WristEvent.DISABLE);
  }

  public void set(double throttle) {
    wristMotor.set(ControlMode.PercentOutput, throttle);
  }

  public void setCylinder(boolean value) {
    wristCylinder.set(value);
  }

  public boolean isAtMid() {
    return !wristMidSwitch.get();
  }

  public boolean isAtBtm() {
    return !wristBtmSwitch.get();
  }

  @Override
  protected void initDefaultCommand() {

  }

  @Override
  public void periodic() {
    isAtMidEntry.forceSetBoolean(isAtMid());
    isAtBtmEntry.forceSetBoolean(isAtBtm());
    motorEntry.forceSetNumber(wristMotor.getMotorOutputPercent());
    currentStateEntry.forceSetString(stateMachine.getCurrentState().toString());
  }

  public enum WristEvent {UP_CMD, DOWN_CMD, BTM_SENSOR, MID_SENSOR, DISABLE}

  public enum WristState {OFF_UP, OFF_DOWN, DOWN_TRAVEL_POWER, DOWN_TRAVEL_BRAKE, UP_TRAVEL}

  @StateMachineParameters(stateType = WristState.class, eventType = WristEvent.class, contextType = Integer.class)
  private static class WristStateMachine extends AbstractUntypedStateMachine {

    protected void stop(WristState from, WristState to, WristEvent event, Integer context) {
      if (wristMotor != null) {
        wristMotor.set(ControlMode.PercentOutput, 0);
      }
    }

    protected void downTravelPwr(WristState from, WristState to, WristEvent event,
        Integer context) {
      if (wristMotor != null) {
        wristMotor.set(ControlMode.PercentOutput, -Tuning.wristDownTravelPwrThrot);
      }
    }

    protected void downTravelBrake(WristState from, WristState to, WristEvent event,
        Integer context) {
      if (wristMotor != null) {
        wristMotor.set(ControlMode.PercentOutput, Tuning.wristDownTravelBrakeThrot);
      }
    }

    protected void upTravel(WristState from, WristState to, WristEvent event, Integer context) {
      if (wristMotor != null) {
        wristMotor.set(ControlMode.PercentOutput, Tuning.wristUpTravelThrot);
      }
    }
  }
}
