package org.team1540.robot2019.subsystems;

import static org.team1540.robot2019.Hardware.elevatorA;
import static org.team1540.robot2019.Hardware.elevatorB;
import static org.team1540.robot2019.Tuning.inPerRotation;

import com.revrobotics.ControlType;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.apache.log4j.Logger;
import org.team1540.robot2019.Tuning;

public class Elevator extends Subsystem {

  private static final Logger logger = Logger.getLogger(Elevator.class);

  private final Object controllerLock = new Object();
  private ElevatorController controller = new ElevatorController(inPerRotation);
  private Notifier controllerNotifier = new Notifier(controller);

  private volatile boolean enableController = true;

  private NetworkTable table = NetworkTableInstance.getDefault().getTable("elevator");

  private NetworkTableEntry positionEntry = table.getEntry("pos");

  private NetworkTableEntry velocityEntry = table.getEntry("vel");

  private NetworkTableEntry throttleEntry = table.getEntry("throt");

  private NetworkTableEntry currentAEntry = table.getEntry("currA");
  private NetworkTableEntry currentBEntry = table.getEntry("currB");

  private NetworkTableEntry targetPosEntry = table.getEntry("tgtPos");
  private NetworkTableEntry targetVelEntry = table.getEntry("tgtVel");

  private NetworkTableEntry statusEntry = table.getEntry("status");
  private NetworkTableEntry limEntry = table.getEntry("limit");


  public Elevator() {
    elevatorA.setInverted(Tuning.invertElevatorA);
    elevatorB.follow(elevatorA, Tuning.invertElevatorB);
    updateController();
    controllerNotifier.startPeriodic(0.01);
  }

  @Override
  protected void initDefaultCommand() {

  }

  public boolean isAtLimit() {
    return elevatorLimitSensor.get();
  }

  public void setWantedPosition(double position) {
    synchronized (controllerLock) {
      controller.setpoint = position;
    }
  }

  public void setRaw(double throttle) {
    elevatorA.set(throttle);
  }

  public boolean isEnableController() {
    return enableController;
  }

  public void setEnableController(boolean enableController) {
    this.enableController = enableController;
  }

  public double getVelocity() {
    return elevatorA.getEncoder().getVelocity() * (inPerRotation / 60);
  }

  public double getThrottle() {
    return elevatorA.getAppliedOutput();
  }

  public double getVoltage() {
    return elevatorA.getBusVoltage() * elevatorA.getAppliedOutput();
  }

  @Override
  public void periodic() {
    updateController();

    positionEntry.forceSetNumber(getPosition());
    velocityEntry.forceSetNumber(getVelocity());

    throttleEntry.forceSetNumber(elevatorA.getAppliedOutput());

    currentAEntry.forceSetNumber(elevatorA.getOutputCurrent());
    currentBEntry.forceSetNumber(elevatorB.getOutputCurrent());

    targetPosEntry.forceSetNumber(controller.posSetpoint);
    targetVelEntry.forceSetNumber(controller.velSetpoint);

    statusEntry.forceSetString(controller.status.toString());
    limEntry.forceSetBoolean(isAtLimit());
  }

  public double getPosition() {
    return (elevatorA.getEncoder().getPosition() * inPerRotation)
        + controller.positionOffset;
  }

  private void updateController() {
    // These are updated in the main robot loop because we can't (shouldn't) be accessing them from
    // inside the controller due to thread-safety issues with the PreferenceManager changing things
    // around. So, they get synchronized every tick.
    synchronized (controllerLock) {
      controller.inPerRotations = inPerRotation;
      controller.maxVel = Tuning.elevatorMaxVel;
      controller.maxAccelDown = Tuning.elevatorMaxAccelDown;
      controller.maxAccelUp = Tuning.elevatorMaxAccelUp;
      controller.minTrapezoidalRange = Tuning.elevatorMinTrapezoidalRange;
      controller.kV = Tuning.elevatorKV;
      controller.kA = Tuning.elevatorKA;
      controller.vIntercept = Tuning.elevatorVIntercept;
      controller.height = Tuning.elevatorDelta;
    }
  }

  // Sufficiently advanced motion technology is indistinguishable from motion magic.
  private class ElevatorController implements Runnable {

    private static final double GRAV_ACCEL = 386.220472; // in/s^2

    volatile double inPerRotations;
    volatile double maxAccelUp;
    volatile double maxVel;
    volatile double maxAccelDown;
    volatile double setpoint;
    volatile double minTrapezoidalRange;
    volatile double kV;
    volatile double kA;
    volatile double vIntercept;
    volatile double height;

    volatile double positionOffset;

    volatile double posSetpoint;
    volatile double velSetpoint;

    volatile Status status = Status.DISABLE;

    Status lastStatus = status;

    ElevatorController(double elevatorRotationsPerIn) {
      positionOffset = -(elevatorA.getEncoder().getPosition() / elevatorRotationsPerIn);
    }

    @Override
    public void run() {
      synchronized (controllerLock) {
        double currentPosition =
            (elevatorA.getEncoder().getPosition() * inPerRotations) + positionOffset;
        double currentVelocity = (elevatorA.getEncoder().getVelocity() * (inPerRotations / 60));
        if (isAtLimit()) {
          if (currentPosition > (height / 2)) {
            // we're probably at the top

            positionOffset = height - (elevatorA.getEncoder().getPosition() * inPerRotations);
            setpoint = height;
            currentPosition = height;

            status = Status.ZERO_HIGH;
          } else {
            // we're probably at the bottom

            positionOffset = -(elevatorA.getEncoder().getPosition() * inPerRotations);
            setpoint = 0;
            currentPosition = 0;

            status = Status.ZERO_LOW;
            // this will then cause the motors to stop and brake to be turned on
          }
        }

        if (DriverStation.getInstance().isEnabled() && enableController) {
          if (Math.abs(currentPosition - setpoint) < minTrapezoidalRange) {
            // we're very close to the target, just use position PID
            status = Status.MOVE_PID;

            setPositionAndVelocity((setpoint + positionOffset) / inPerRotations, 0);

            // clear (set to current values) setpoints
            posSetpoint = currentPosition;
            velSetpoint = currentVelocity;
          } else {
            // trapezoid time
            status = Status.MOVE_TRAPEZOIDAL;

            // figure out what acceleration we need at our current velocity in order to hit 0
            // velocity at the distance we want
            // this is derived from kinematics equations
            double d = currentPosition - setpoint;
            double a = (Math.pow(currentVelocity, 2)) / (2 * d);
            double appliedAccel;
            if (Math.abs(a) < ((currentPosition < setpoint) ? maxAccelUp : maxAccelDown)) {
              // we can accelerate
              appliedAccel = ((currentPosition < setpoint) ? maxAccelUp : -maxAccelDown);
            } else {
              // we need to slow down
              // note that this does ignore max acceleration in favor of bringing the mechanism to a
              // stop on the setpoint; max accel is more of a guideline
              appliedAccel = -a;
            }

            velSetpoint += appliedAccel;

            // cap velocity setpoint
            if (velSetpoint > maxVel) {
              velSetpoint = maxVel;
            }

            posSetpoint += velSetpoint;

            setPositionAndVelocity((posSetpoint + positionOffset) / inPerRotations, velSetpoint);
          }
        } else {
          // clear (set to current values) setpoints
          posSetpoint = currentPosition;
          velSetpoint = currentVelocity;

          status = Status.DISABLE;
        }

        // status change listening
        if (status != lastStatus) {
          logger.debug("Elevator state changed from " + lastStatus + " to " + status);

          lastStatus = status;
        }
      }
    }

    private void setPositionAndVelocity(double pos, double vel) {
      elevatorA.getPIDController()
          .setReference(pos, ControlType.kPosition, 0, vel * kV + vIntercept);
    }
  }

  private enum Status {
    DISABLE, MOVE_TRAPEZOIDAL, MOVE_PID, STOP, ZERO_HIGH, ZERO_LOW
  }
}
