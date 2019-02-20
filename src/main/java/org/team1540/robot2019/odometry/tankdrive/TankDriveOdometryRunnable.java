package org.team1540.robot2019.odometry.tankdrive;

import edu.wpi.first.wpilibj.Notifier;
import java.util.function.DoubleSupplier;
import org.team1540.robot2019.datastructures.Transform3DStamped;
import org.team1540.robot2019.datastructures.TransformManager;
import org.team1540.robot2019.datastructures.threed.Transform3D;

/**
 * Runnable wrapper class for the TankDriveOdometryAccumulator.
 */
public class TankDriveOdometryRunnable implements Runnable {

    private TankDriveOdometryAccumulator odometryAccumulator = new TankDriveOdometryAccumulator();

    private DoubleSupplier leftPosSupplier;
    private DoubleSupplier rightPosSupplier;
    private DoubleSupplier angleSupplier;
    private final String sourceFrame;
    private final String destFrame;
    private final TransformManager tf;

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
        String sourceFrame,
        String destFrame,
        TransformManager tf,
        double period) {

        this.leftPosSupplier = leftPosSupplier;
        this.rightPosSupplier = rightPosSupplier;
        this.angleSupplier = angleSupplier;
        this.sourceFrame = sourceFrame;
        this.destFrame = destFrame;
        this.tf = tf;

        new Notifier(this).startPeriodic(period);
    }

    @Override
    public void run() {
        odometryAccumulator.update(
            leftPosSupplier.getAsDouble(),
            rightPosSupplier.getAsDouble(),
            angleSupplier.getAsDouble());

        odomToBaseLink = odometryAccumulator.getTransform();

        tf.sendTransform(new Transform3DStamped(sourceFrame, destFrame, odomToBaseLink));
    }

    public Transform3D getOdomToBaseLink() {
        return odomToBaseLink;
    }
}
