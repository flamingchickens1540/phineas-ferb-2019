package org.team1540.robot2019.networking;

import edu.wpi.first.wpilibj.Notifier;
import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.nio.ByteBuffer;
import java.util.List;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.apache.log4j.Logger;
import org.team1540.robot2019.datastructures.twod.Transform2D;

// TODO: Use logging class
public class UDPGoalConfigSender {

    public static final Logger logger = Logger.getLogger(UDPGoalConfigSender.class);

    private DatagramSocket clientSocket;
    private InetAddress address;
    private String addressString;
    private int port;

    private final double period;

    private Transform2D goal;
    private TEBConfig cfg = new TEBConfig();
    private List<Vector2D> viaPoints;

    public UDPGoalConfigSender(String address, int port, double period) {
        addressString = address;
        this.port = port;
        this.period = period;
        attemptConnection();
    }

    public void attemptConnection() {
        logger.info("Attempting to connect... Address: " + addressString);
        try {
            this.address = InetAddress.getByName(addressString);
            this.clientSocket = new DatagramSocket();
            logger.info("Connected to address: " + addressString);
        } catch (UnknownHostException | SocketException e) {
            logger.warn("Unable to connect to address: " + addressString);
            e.printStackTrace();
            autoReconnect();
        }
    }

    public void setGoal(Transform2D goal) {
        this.goal = goal;
    }

    public void setCfg(TEBConfig cfg) {
        this.cfg = cfg;
    }

    public void setViaPoints(List<Vector2D> viaPoints) {
        this.viaPoints = viaPoints;
    }

    public void sendIt() throws IOException {
        if (clientSocket == null || goal == null || cfg == null) {
            return;
        }
        int numViaPoints = viaPoints == null ? 0 : viaPoints.size();
        ByteBuffer buffer = ByteBuffer.allocate(Double.BYTES * (11 + numViaPoints * 2))
            .putDouble(goal.getX())
            .putDouble(goal.getY())
            .putDouble(goal.getTheta())
            .putDouble(cfg.getMaxVelX())
            .putDouble(cfg.getMaxVelXBackwards())
            .putDouble(cfg.getMaxVelTheta())
            .putDouble(cfg.getAccLimX())
            .putDouble(cfg.getAccLimTheta())
            .putDouble(cfg.getMinTurningRadius())
            .putDouble(cfg.getWeightKinematicsForwardDrive());
        buffer.putDouble(numViaPoints);
        if (viaPoints != null) {
            viaPoints.forEach((point) -> {
                buffer
                    .putDouble(point.getX())
                    .putDouble(point.getY());
            });
        }
        byte[] data = buffer // TODO: Send a timestamp/counter
            .array();
        DatagramPacket sendPacket = new DatagramPacket(data, data.length, address, port);
        clientSocket.send(sendPacket);
    }

    private void autoReconnect() {
        new Notifier(this::attemptConnection).startSingle(1);
    }
}
