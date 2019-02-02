package org.team1540.robot2019.tuners;

import static org.team1540.robot2019.OI.LB;
import static org.team1540.robot2019.OI.RB;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import java.io.IOException;
import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.team1540.robot2019.Hardware;
import org.team1540.robot2019.OI;
import org.team1540.robot2019.Tuning;
import org.team1540.robot2019.commands.elevator.MoveElevatorToPosition;
import org.team1540.robot2019.datastructures.Odometry;
import org.team1540.robot2019.datastructures.threed.Transform3D;
import org.team1540.robot2019.networking.UDPOdometryGoalSender;
import org.team1540.robot2019.networking.UDPTwistReceiver;
import org.team1540.robot2019.subsystems.Drivetrain;
import org.team1540.robot2019.utils.LimelightLocalization;
import org.team1540.robot2019.utils.StateChangeDetector;
import org.team1540.robot2019.utils.TankDriveOdometryRunnable;
import org.team1540.robot2019.vision.commands.UDPAutoLineup;
import org.team1540.rooster.preferencemanager.PreferenceManager;
import org.team1540.rooster.util.SimpleCommand;
import org.team1540.rooster.wrappers.RevBlinken;

public class AutoAlignTuningRobot extends TimedRobot {

  private static Drivetrain drivetrain;
  private static Joystick joystick = new Joystick(0);

  public static AHRS navx = new AHRS(Port.kMXP);
//  public static RevBlinken leds = new RevBlinken(9);

  private static Transform3D map_to_odom = Transform3D.IDENTITY;
  private static Transform3D odom_to_base_link = Transform3D.IDENTITY;

  public static TankDriveOdometryRunnable wheelOdometry;

  public static UDPOdometryGoalSender udpSender;
  public static UDPTwistReceiver udpReceiver;
  public static LimelightLocalization limelightLocalization;

  public static Transform3D lastOdomToLimelight;

  static JoystickButton autoAlignButton = new JoystickButton(joystick, RB);
  static JoystickButton autoAlignCancelButton = new JoystickButton(joystick, LB);

  static Command alignCommand = null;


  @Override
  public void robotInit() {
    PreferenceManager.getInstance().add(new Tuning());
    Hardware.initDrive();
    OI.initJoysticks();

    drivetrain = new Drivetrain();


    navx.zeroYaw();
    drivetrain.zeroEncoders();

    wheelOdometry = new TankDriveOdometryRunnable(
        drivetrain::getLeftPositionMeters,
        drivetrain::getRightPositionMeters,
        () -> Math.toRadians(-navx.getAngle())
    );

    udpReceiver = new UDPTwistReceiver(5801, () -> {
      new Notifier(udpReceiver::attemptConnection).startSingle(1);
    });

    udpSender = new UDPOdometryGoalSender("10.15.40.202", 5800, () -> {
      new Notifier(udpSender::attemptConnection).startSingle(1);
    });

    limelightLocalization = new LimelightLocalization("limelight-a");

    StateChangeDetector limelightStateDetector = new StateChangeDetector(false);

    new Notifier(() -> {
      wheelOdometry.run();
      odom_to_base_link = wheelOdometry.getOdomToBaseLink();
      udpSender.setOdometry(new Odometry(odom_to_base_link, drivetrain.getTwist()));
      odom_to_base_link.toTransform2D().putToNetworkTable("Odometry/Debug/WheelOdometry");
      boolean targetFound = limelightLocalization.attemptUpdatePose();
      if (targetFound) {
        limelightLocalization.getBaseLinkToVisionTarget().toTransform2D().putToNetworkTable("LimelightLocalization/Debug/BaseLinkToVisionTarget");
        Transform3D goal = wheelOdometry.getOdomToBaseLink()
            .add(limelightLocalization.getBaseLinkToVisionTarget())
            .add(new Transform3D(new Vector3D(-0.65, 0, 0), Rotation.IDENTITY));

        lastOdomToLimelight = goal;
        goal.toTransform2D().putToNetworkTable("LimelightLocalization/Debug/BaseLinkToGoal");
      }
      try {
        udpSender.sendIt();
      } catch (IOException e) {
        DriverStation.reportWarning("Unable to send Odometry packet!", false);
      }
    }).startPeriodic(0.01);
//
//    // Testing code
//    Command testTEB = new SimpleCommand("Test TEB", () -> {
//      new UDPVelocityTwistDrive().start();
//    });
//    SmartDashboard.putData(testTEB);

    autoAlignButton.whenPressed(new SimpleCommand("Start Lineup", () -> {
      alignCommand = new UDPAutoLineup(udpSender, udpReceiver, limelightLocalization, wheelOdometry, lastOdomToLimelight, navx);
      alignCommand.start();
    }));
    autoAlignCancelButton.whenPressed(new SimpleCommand("Cancel Lineup", () -> {
      if (alignCommand != null) {
        alignCommand.cancel();
      }
    }));
  }

  @Override
  public void robotPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    drivetrain.setBrake(false);
  }

  @Override
  public void teleopInit() {
    drivetrain.configTalonsForVelocity();
  }
}
