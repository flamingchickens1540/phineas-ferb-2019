package org.team1540.robot2019.networking;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.nio.ByteBuffer;
import org.apache.commons.math3.geometry.euclidean.threed.RotationConvention;
import org.apache.commons.math3.geometry.euclidean.threed.RotationOrder;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.team1540.robot2019.datastructures.Odometry;
import org.team1540.robot2019.datastructures.twod.Transform2D;

public class UDPOdometryGoalSender {

  private DatagramSocket clientSocket;
  private InetAddress address;
  private String addressString;
  private int port;
  private Runnable onException;

  private Transform2D goal;
  private Odometry odometry;
  private Vector2D viaPoint;

  public UDPOdometryGoalSender(String address, int port, Runnable onException) {
    addressString = address;
    this.port = port;
    this.onException = onException;
    attemptConnection();
  }

  public void attemptConnection() {
    try {
      this.address = InetAddress.getByName(addressString);
      this.clientSocket = new DatagramSocket();
      System.out.println("[UDP Sender]: Connected to " + addressString);
    } catch (UnknownHostException | SocketException e) {
      System.out.println("[UDP Sender]: Couldn't connect to " + addressString);
      e.printStackTrace();
      onException.run();
    }
  }

  public void setGoal(Transform2D goal) {
    this.goal = goal;
  }

  public void setViaPoint(Vector2D viaPoint) {
    this.viaPoint = viaPoint;
  }

  public void setOdometry(Odometry odometry) {
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
}
