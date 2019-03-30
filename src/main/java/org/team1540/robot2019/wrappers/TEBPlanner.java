package org.team1540.robot2019.wrappers;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.function.Supplier;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.team1540.robot2019.datastructures.Odometry;
import org.team1540.robot2019.datastructures.twod.Transform2D;
import org.team1540.robot2019.datastructures.twod.Twist2D;
import org.team1540.robot2019.networking.UDPOdometryGoalSender;
import org.team1540.robot2019.networking.UDPTwistReceiver;

public class TEBPlanner {

    private final NetworkTable tebConfigTable;
    private final UDPTwistReceiver udpReceiver;
    private final UDPOdometryGoalSender udpSender;

    public TEBPlanner(Supplier<Odometry> odometrySupplier, int recievePort, int sendPort, String address, double sendPeriod) {
        tebConfigTable = NetworkTableInstance.getDefault().getTable("TEBPlanner/Config");
        udpReceiver = new UDPTwistReceiver(recievePort);
        udpSender = new UDPOdometryGoalSender(address, sendPort, sendPeriod, odometrySupplier);
        reset();
    }

    private void reset() {
        tebConfigTable.getEntry("ResetTuningVals").setBoolean(true);
    }

    public void setGoal(Transform2D goal) {
        udpSender.setGoal(goal);
    }

    public void setViaPoint(Vector2D positionVector) {
        udpSender.setViaPoint(positionVector);
    }

    public Twist2D getCmdVel() {
        return udpReceiver.getCmdVel();
    }
}
