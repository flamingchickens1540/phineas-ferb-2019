package org.team1540.robot2019.networking;

import edu.wpi.first.wpilibj.Notifier;
import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.nio.ByteBuffer;
import java.util.function.Supplier;
import org.apache.commons.math3.geometry.euclidean.threed.RotationConvention;
import org.apache.commons.math3.geometry.euclidean.threed.RotationOrder;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.apache.log4j.Logger;
import org.team1540.robot2019.datastructures.Odometry;
import org.team1540.robot2019.datastructures.twod.Transform2D;

// TODO: Use logging class
public class UDPOdometryGoalSender {

    public static final Logger logger = Logger.getLogger(UDPOdometryGoalSender.class);

    private DatagramSocket clientSocket;
    private InetAddress address;
    private String addressString;
    private int port;

    private final double period;
    private final Supplier<Odometry> odometrySupplier;

    private Transform2D goal;
    private Odometry odometry;
    private Vector2D viaPoint;

    public UDPOdometryGoalSender(String address, int port, double period, Supplier<Odometry> odometrySupplier) {
        addressString = address;
        this.port = port;
        this.period = period;
        this.odometrySupplier = odometrySupplier;
        attemptConnection();
    }

    public void attemptConnection() {
        logger.info("Attempting to connect... Address: " + addressString);
        try {
            this.address = InetAddress.getByName(addressString);
            this.clientSocket = new DatagramSocket();
            logger.info("Connected to address: " + addressString);
            new Notifier(this::updateData).startPeriodic(period);
        } catch (UnknownHostException | SocketException e) {
            logger.warn("Unable to connect connect to address: " + addressString);
            e.printStackTrace();
            autoReconnect();
        }
    }

    private void updateData() {
        setOdometry(odometrySupplier.get());
        try {
            this.sendIt();
        } catch (IOException e) {
            logger.warn("Unable to send Odometry packet!");
        }
    }

    public void setGoal(Transform2D goal) {
        this.goal = goal;
    }

    public void setViaPoint(Vector2D viaPoint) {
        this.viaPoint = viaPoint;
    }

    private void setOdometry(Odometry odometry) {
        this.odometry = odometry;
    }

    public void sendIt() throws IOException {
        if (clientSocket == null || goal == null || viaPoint == null || odometry == null) {
            return;
        }
        byte[] data = ByteBuffer.allocate(Double.BYTES * 10) // TODO: Send a timestamp/counter
            .putDouble(odometry.getPose().getPosition().getX())
            .putDouble(odometry.getPose().getPosition().getY())
            .putDouble(odometry.getPose().getOrientation().getAngles(RotationOrder.XYZ, RotationConvention.FRAME_TRANSFORM)[2])
            .putDouble(odometry.getTwist().getX())
            .putDouble(odometry.getTwist().getOmega())
            .putDouble(goal.getX())
            .putDouble(goal.getY())
            .putDouble(goal.getTheta())
            .putDouble(viaPoint.getX())
            .putDouble(viaPoint.getY())
            .array();
        DatagramPacket sendPacket = new DatagramPacket(data, data.length, address, port);
        clientSocket.send(sendPacket);
    }

    private void autoReconnect() {
        new Notifier(this::attemptConnection).startSingle(1);
    }
}
