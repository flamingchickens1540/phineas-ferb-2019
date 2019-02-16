package org.team1540.robot2019.tuners;

import static org.team1540.robot2019.OI.LB;
import static org.team1540.robot2019.OI.RB;
import static org.team1540.robot2019.Robot.drivetrain;
import static org.team1540.robot2019.Robot.udpReceiver;
import static org.team1540.robot2019.Robot.udpSender;
import static org.team1540.robot2019.Robot.wheelOdometry;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.io.IOException;
import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.team1540.robot2019.Hardware;
import org.team1540.robot2019.OI;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.Tuning;
import org.team1540.robot2019.datastructures.Odometry;
import org.team1540.robot2019.datastructures.threed.Transform3D;
import org.team1540.robot2019.networking.UDPOdometryGoalSender;
import org.team1540.robot2019.networking.UDPTwistReceiver;
import org.team1540.robot2019.subsystems.Drivetrain;
import org.team1540.robot2019.utils.LimelightLocalization;
import org.team1540.robot2019.utils.StateChangeDetector;
import org.team1540.robot2019.utils.TankDriveOdometryRunnable;
import org.team1540.robot2019.vision.commands.UDPAutoLineup;
import org.team1540.robot2019.vision.commands.UDPVelocityTwistDrive;
import org.team1540.rooster.power.PowerManager;
import org.team1540.rooster.util.SimpleCommand;

public class AutoAlignTuningRobot extends TimedRobot {

  private static Joystick joystick = new Joystick(0);

//  public static RevBlinken leds = new RevBlinken(9);


  static JoystickButton autoAlignButton = new JoystickButton(joystick, RB);
  static JoystickButton autoAlignCancelButton = new JoystickButton(joystick, LB);

  static Command alignCommand = null;


  @Override
  public void robotInit() {
    AdjustableManager.getInstance().add(new Tuning());
    PowerManager.getInstance().interrupt();
//    PreferenceManager.getInstance().add(new Tuning());
      Scheduler.getInstance().run();
      Hardware.initDrive();
      OI.initJoysticks();

    SmartDashboard.putNumber("angleMultiplier", 1);

    drivetrain = new Drivetrain();

    Robot.navx.zeroYaw();
    drivetrain.zeroEncoders();

    wheelOdometry = new TankDriveOdometryRunnable(
        drivetrain::getLeftPositionMeters,
        drivetrain::getRightPositionMeters,
        () -> Math.toRadians(-Robot.navx.getAngle())
    );

    udpReceiver = new UDPTwistReceiver(5801, () -> {
      new Notifier(udpReceiver::attemptConnection).startSingle(1);
    });

    udpSender = new UDPOdometryGoalSender("10.15.40.201", 5800, () -> {
      new Notifier(udpSender::attemptConnection).startSingle(1);
    });

    Robot.limelightLocalization = new LimelightLocalization("limelight-a");

    StateChangeDetector limelightStateDetector = new StateChangeDetector(false);

    new Notifier(() -> {
      wheelOdometry.run();
      Robot.odom_to_base_link = wheelOdometry.getOdomToBaseLink();
      udpSender.setOdometry(new Odometry(Robot.odom_to_base_link, drivetrain.getTwist()));
      Robot.odom_to_base_link.toTransform2D().putToNetworkTable("Odometry/Debug/WheelOdometry");
      boolean targetFound = Robot.limelightLocalization.attemptUpdatePose();
      if (targetFound) {
        Robot.limelightLocalization.getBaseLinkToVisionTarget().toTransform2D().putToNetworkTable("LimelightLocalization/Debug/BaseLinkToVisionTarget");
        Transform3D goal = wheelOdometry.getOdomToBaseLink()
            .add(Robot.limelightLocalization.getBaseLinkToVisionTarget())
            .add(new Transform3D(new Vector3D(-0.65, 0, 0), Rotation.IDENTITY));

        Robot.lastOdomToLimelight = goal;
        goal.toTransform2D().putToNetworkTable("LimelightLocalization/Debug/BaseLinkToGoal");
      }
      if (alignCommand == null || !alignCommand.isRunning()) {
        if (targetFound) {
          OI.driver.setRumble(RumbleType.kLeftRumble, 1);
        } else {
          OI.driver.setRumble(RumbleType.kLeftRumble, 0);
        }
      }
      try {
        udpSender.sendIt();
      } catch (IOException e) {
        DriverStation.reportWarning("Unable to send Odometry packet!", false);
      }
    }).startPeriodic(0.011);

    // Testing code
    Command testTEB = new SimpleCommand("Test TEB", () -> {
      new UDPVelocityTwistDrive().start();
    });
    SmartDashboard.putData(testTEB);

    SmartDashboard.putNumber("test-goal/position/x", 2);
    SmartDashboard.putNumber("test-goal/position/y", 0);
    SmartDashboard.putNumber("test-goal/orientation/z", 0);

      // Testing code
      Command resetWheelOdom = new SimpleCommand("Update PID Values", () -> {
          drivetrain.updatePIDValues();
        System.out.println(Tuning.driveVelocityP);
      });
      resetWheelOdom.setRunWhenDisabled(true);
      SmartDashboard.putData(resetWheelOdom);

    autoAlignButton.whenPressed(new SimpleCommand("Start Lineup", () -> {
      alignCommand = new UDPAutoLineup(drivetrain, udpSender, udpReceiver, Robot.limelightLocalization, wheelOdometry, Robot.lastOdomToLimelight, Robot.navx);
//        alignCommand = new UDPVelocityTwistDrive();
      alignCommand.start();
    }));
    autoAlignCancelButton.whenPressed(new SimpleCommand("Cancel Lineup", () -> {
      if (alignCommand != null) {
        alignCommand.cancel();
      }
    }));

    NetworkTable tebConfigTable = NetworkTableInstance.getDefault().getTable("TEBPlanner/Config");
    tebConfigTable.getEntry("ResetTuningVals").setBoolean(true);
  }

  @Override
  public void robotPeriodic() {

    Scheduler.getInstance().run();

      NetworkTableInstance nt = NetworkTableInstance.getDefault();
      nt.getTable("Drivetrain/Debug/Position").getEntry("Left").setNumber(drivetrain.getLeftPosition());
      nt.getTable("Drivetrain/Debug/Position").getEntry("Right").setNumber(drivetrain.getRightPosition());
  }

  @Override
  public void disabledInit() {
    drivetrain.setBrake(false);
  }

  @Override
  public void teleopInit() {
  }
}
