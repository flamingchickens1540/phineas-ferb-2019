package org.team1540.robot2019.odometry.tankdrive;

import edu.wpi.first.wpilibj.Notifier;
import java.util.function.DoubleSupplier;
import org.team1540.robot2019.datastructures.threed.Transform3D;

/**
 * Runnable wrapper class for the TankDriveOdometryAccumulator.
 */
public class TankDriveOdometryRunnable implements Runnable {

    private TankDriveOdometryAccumulator odometryAccumulator = new TankDriveOdometryAccumulator();

    private DoubleSupplier leftPosSupplier;
    private DoubleSupplier rightPosSupplier;
    private DoubleSupplier angleSupplier;

    private Transform3D odomToBaseLink = Transform3D.IDENTITY;

    /**
     * @param leftPosSupplier Supplier for left tank drive position in meters
     * @param rightPosSupplier Supplier for right tank drive position in meters
     * @param angleSupplier Supplier for continuous angle measurement in radians
     */
    public TankDriveOdometryRunnable(
        DoubleSupplier leftPosSupplier,
        DoubleSupplier rightPosSupplier,
        DoubleSupplier angleSupplier,
        double period) {

        this.leftPosSupplier = leftPosSupplier;
        this.rightPosSupplier = rightPosSupplier;
        this.angleSupplier = angleSupplier;

        new Notifier(this).startPeriodic(period);
    }

    /**
     * @param leftPosSupplier Supplier for left tank drive position in meters
     * @param rightPosSupplier Supplier for right tank drive position in meters
     * @param angleSupplier Supplier for continuous angle measurement in radians
     */
    public TankDriveOdometryRunnable(
        DoubleSupplier leftPosSupplier,
        DoubleSupplier rightPosSupplier,
        DoubleSupplier angleSupplier) {

        this.leftPosSupplier = leftPosSupplier;
        this.rightPosSupplier = rightPosSupplier;
        this.angleSupplier = angleSupplier;
    }

    @Override
    public void run() {
        odometryAccumulator.update(
            leftPosSupplier.getAsDouble(),
            rightPosSupplier.getAsDouble(),
            angleSupplier.getAsDouble());

        odomToBaseLink = odometryAccumulator.getTransform();
    }

    public Transform3D getOdomToBaseLink() {
        return odomToBaseLink;
    }
}
