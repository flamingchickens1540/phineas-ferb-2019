package org.team1540.robot2019.subsystems;

import static org.team1540.robot2019.Hardware.elevatorA;
import static org.team1540.robot2019.Hardware.elevatorB;
import static org.team1540.robot2019.Hardware.elevatorBrake;
import static org.team1540.robot2019.Hardware.elevatorBtmSwitch;
import static org.team1540.robot2019.Hardware.elevatorTopSwitch;
import static org.team1540.robot2019.Tuning.elevatorRotationsPerIn;

import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Subsystem;
import java.util.concurrent.atomic.AtomicBoolean;
import org.team1540.robot2019.Tuning;

public class Elevator extends Subsystem {

  private final Object controllerLock = new Object();
  private ElevatorController controller = new ElevatorController(elevatorRotationsPerIn);
  private Notifier controllerNotifier = new Notifier(controller);

  private AtomicBoolean enableController = new AtomicBoolean();

  public Elevator() {
    elevatorA.setInverted(Tuning.invertElevatorA);
    elevatorB.follow(elevatorA, Tuning.invertElevatorB);
    updateController();
    controllerNotifier.startPeriodic(0.01);
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

  public void setWantedPosition(double position) {
    synchronized (controllerLock) {
      controller.setpoint = position;
    }
  }

  @Override
  public void periodic() {
    updateController();
  }

  private void updateController() {
    // These are updated in the main robot loop because we can't (shouldn't) be accessing them from
    // inside the controller due to thread-safety issues with the PreferenceManager changing things
    // around. So, they get synchronized every tick.
    synchronized (controllerLock) {
      controller.rotationsPerIn = elevatorRotationsPerIn;
      controller.maxVel = Tuning.elevatorMaxVel;
      controller.maxAccelDown = Tuning.elevatorMaxAccelDown;
      controller.maxAccelUp = Tuning.elevatorMaxAccelUp;
      controller.tolerance = Tuning.elevatorTolerance;
      controller.minTrapezoidalRange = Tuning.elevatorMinTrapezoidalRange;
      controller.holdThrot = Tuning.elevatorHoldThrottle;
      controller.velCoeff = Tuning.elevatorVelCoeff;
      controller.accelCoeff = Tuning.elevatorAccelCoeff;
      controller.height = Tuning.elevatorDelta;
    }
  }

  // Sufficiently advanced motion technology is indistinguishable from motion magic.
  private class ElevatorController implements Runnable {

    private static final double GRAV_ACCEL = 386.220472; // in/s^2

    double rotationsPerIn;
    double maxAccelUp;
    double maxVel;
    double maxAccelDown;
    double setpoint;
    double tolerance;
    double minTrapezoidalRange;
    double holdThrot;
    double velCoeff;
    double accelCoeff;
    double height;

    double positionOffset;

    double posSetpoint;
    double velSetpoint;

    ElevatorController(double elevatorRotationsPerIn) {
      positionOffset = -(elevatorA.getEncoder().getPosition() / elevatorRotationsPerIn);
    }

    @Override
    public void run() {
      synchronized (controllerLock) {
        double currentPosition =
            (elevatorA.getEncoder().getPosition() / rotationsPerIn) + positionOffset;
        double currentVelocity = (elevatorA.getEncoder().getVelocity() / (rotationsPerIn * 60));
        if (isAtTop() && setpoint >= currentPosition) {
          positionOffset = height - (elevatorA.getEncoder().getPosition() / elevatorRotationsPerIn);
          setpoint = height;
          currentPosition = height;
        }
        if (isAtBottom() && setpoint <= currentPosition) {
          positionOffset = -(elevatorA.getEncoder().getPosition() / elevatorRotationsPerIn);
          setpoint = 0;
          currentPosition = 0;
          // this will then cause the motors to stop and brake to be turned on
        }

        double absError = Math.abs(currentPosition - setpoint);

        if (DriverStation.getInstance().isEnabled() && enableController.get()) {
          if (absError < tolerance) {
            // we're at our target, just engage the brake and sit there
            elevatorBrake.set(true);
            elevatorA.stopMotor();

            // clear (set to current values) setpoints
            posSetpoint = currentPosition;
            velSetpoint = currentVelocity;
          } else {
            // we need to move
            elevatorBrake.set(false);
            if (absError < minTrapezoidalRange) {
              // we're very close to the target, just use position PID
              // holdThrot is in throttle percentages but this method takes volts for some reason
              elevatorA.getPIDController()
                  .setReference((setpoint + positionOffset) * rotationsPerIn, ControlType.kPosition,
                      0, holdThrot * 12);

              // clear (set to current values) setpoints
              posSetpoint = currentPosition;
              velSetpoint = currentVelocity;
            } else {
              // trapezoid time

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

              double feedForward = 0;

              feedForward += (velSetpoint) * velCoeff;

              feedForward += (appliedAccel + GRAV_ACCEL) * accelCoeff;

              elevatorA.getPIDController()
                  .setReference((posSetpoint + positionOffset) * rotationsPerIn,
                      ControlType.kPosition, 0, feedForward);
            }
          }
        } else {
          // clear (set to current values) setpoints
          posSetpoint = currentPosition;
          velSetpoint = currentVelocity;
        }
      }
    }
  }
}
