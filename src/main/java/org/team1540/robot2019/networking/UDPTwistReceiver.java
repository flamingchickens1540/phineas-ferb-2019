package org.team1540.robot2019.networking;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import javax.annotation.Nullable;
import org.team1540.robot2019.Tuning;
import org.team1540.robot2019.datastructures.twod.Twist2D;

public class UDPTwistReceiver implements Runnable {

  private Runnable onException;
  private int port;

  private long lastReceivedTime = 0;

  private double cmdVelX = 0;
  private double cmdVelTheta = 0;

  public UDPTwistReceiver(int port, Runnable onException) {
    this.onException = onException;
    this.port = port;
    attemptConnection();
  }

  public void attemptConnection() {
    new Thread(this).start();
  }

  @Override
  public void run() {
    DatagramSocket serverSocket;
    try {
      serverSocket = new DatagramSocket(port);
    } catch (SocketException e) {
      onException.run();
      return;
    }
    byte[] data = new byte[Double.SIZE * 2];
      while (true) {
        DatagramPacket receivePacket = new DatagramPacket(data, data.length);
        try {
          serverSocket.receive(receivePacket);
        } catch (IOException e) {
          onException.run();
          return;
        }
        data = receivePacket.getData();
        ByteBuffer buf = ByteBuffer.wrap(data).order(ByteOrder.LITTLE_ENDIAN); // TODO: Get a timestamp/counter
        cmdVelX = buf.getDouble();
        cmdVelTheta = buf.getDouble();
        lastReceivedTime = System.currentTimeMillis();
      }
  }

  @Nullable
  public Twist2D getCmdVel() {
    if (System.currentTimeMillis() - lastReceivedTime < Tuning.drivetrainUDPTimeout) {
      return new Twist2D(cmdVelX, 0, cmdVelTheta);
    } else {
      return Twist2D.ZERO;
    }
  }
}