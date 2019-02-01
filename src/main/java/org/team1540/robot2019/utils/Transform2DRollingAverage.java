package org.team1540.robot2019.utils;

import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.google.common.collect.EvictingQueue;
import java.util.Queue;
import org.team1540.robot2019.datastructures.twod.Transform2D;

public class Transform2DRollingAverage {

  private Queue<Transform2D> poseQueue;

  public Transform2DRollingAverage(int numElements) {
    this.poseQueue = EvictingQueue.create(numElements);
  }

  public void addTransform(Transform2D pose) {
    poseQueue.add(pose);
  }

  public Transform2D getAverage() {
    int listSize = poseQueue.size();
    if (listSize == 0) {
      return null;
    }
    double sumX = 0;
    double sumY = 0;
    double sumSin = 0;
    double sumCos = 0;
    for (Transform2D pose : poseQueue) {
      sumX += pose.getX();
      sumY += pose.getY();
      sumSin += sin(pose.getTheta());
      sumCos += cos(pose.getTheta());
    }
    return new Transform2D(
        sumX / listSize, sumY / listSize, atan2(sumSin / listSize, sumCos / listSize));
  }
}
