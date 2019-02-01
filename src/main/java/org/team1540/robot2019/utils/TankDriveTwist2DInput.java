package org.team1540.robot2019.utils;

import java.util.OptionalDouble;
import org.team1540.robot2019.datastructures.twod.Twist2D;
import org.team1540.rooster.drive.pipeline.DriveData;
import org.team1540.rooster.drive.pipeline.TankDriveData;
import org.team1540.rooster.functional.Input;

public class TankDriveTwist2DInput implements Input<TankDriveData> {
  private Twist2D twist;
  private double tankDriveRadius;

  /**
   * @param tankDriveRadius Radius of the tank drive in meters
   */
  public TankDriveTwist2DInput(double tankDriveRadius) {
    this.tankDriveRadius = tankDriveRadius;
  }

  public void setTwist(Twist2D twist) {
    this.twist = twist;
  }

  @Override
  public TankDriveData get() {
    double leftSetpoint = (twist.getX() - twist.getOmega() * tankDriveRadius);
    double rightSetpoint = (twist.getX() + twist.getOmega() * tankDriveRadius);
    return new TankDriveData(new DriveData(OptionalDouble.of(leftSetpoint)), new DriveData(OptionalDouble.of(rightSetpoint)), OptionalDouble.empty(), OptionalDouble.empty());
    // return new TankDriveData(new DriveData(OptionalDouble.of(leftSetpoint)), new DriveData(OptionalDouble.of(rightSetpoint)), OptionalDouble.empty(), OptionalDouble.of(twist.getOmega()));
  }
}
