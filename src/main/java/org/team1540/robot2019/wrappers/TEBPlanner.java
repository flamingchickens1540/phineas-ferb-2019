package org.team1540.robot2019.wrappers;

import edu.wpi.first.wpilibj.command.Subsystem;
import java.io.IOException;
import java.util.List;
import java.util.function.Supplier;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.team1540.robot2019.datastructures.Odometry;
import org.team1540.robot2019.datastructures.twod.Transform2D;
import org.team1540.robot2019.datastructures.twod.Twist2D;
import org.team1540.robot2019.networking.TEBConfig;
import org.team1540.robot2019.networking.UDPGoalConfigSender;
import org.team1540.robot2019.networking.UDPOdometrySender;
import org.team1540.robot2019.networking.UDPTwistReceiver;

public class TEBPlanner extends Subsystem {

    private final UDPTwistReceiver udpReceiver;
    private final UDPGoalConfigSender udpGoalSender;
    private final UDPOdometrySender udpSender;

    public TEBPlanner(Supplier<Odometry> odometrySupplier, int twistPortIn, int configPortOut, int odomPortOut, String address, double sendPeriod) {
        udpReceiver = new UDPTwistReceiver(twistPortIn);
        udpGoalSender = new UDPGoalConfigSender(address, configPortOut);
        udpSender = new UDPOdometrySender(address, odomPortOut, sendPeriod, odometrySupplier);
    }

    public void setGoal(Transform2D goal) {
        udpGoalSender.setGoal(goal);
    }

    public Transform2D getGoal() {
        return udpGoalSender.getGoal();
    }

    public void setCfg(TEBConfig cfg) {
        udpGoalSender.setCfg(cfg);
    }

    public TEBConfig getCfg() {
        return udpGoalSender.getCfg();
    }

    public void setViaPoints(List<Vector2D> positionVector) {
        udpGoalSender.setViaPoints(positionVector);
    }

    public List<Vector2D> getViaPoints() {
        return udpGoalSender.getViaPoints();
    }

    public void sendGoalAndConfig() throws IOException {
        udpGoalSender.sendIt();
    }

    public Twist2D getCmdVel() {
        return udpReceiver.getCmdVel();
    }

    @Override
    protected void initDefaultCommand() {}
}
