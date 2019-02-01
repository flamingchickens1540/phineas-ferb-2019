package org.team1540.robot2019.utils;

import org.apache.commons.math3.geometry.euclidean.threed.Line;
import org.apache.commons.math3.geometry.euclidean.threed.Plane;
import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.RotationConvention;
import org.apache.commons.math3.geometry.euclidean.threed.RotationOrder;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.team1540.robot2019.datastructures.threed.Transform3D;

public class DualVisionTargetLocalizationUtils {

  private static double CM_TOLERANCE = 0.0001;

  public static Vector2D anglesFromScreenSpace(Vector2D normalizedScreenPoint, double hoz_fov, double vert_fov) {
    //http://docs.limelightvision.io/en/latest/theory.html#from-pixels-to-angles
    double vpw = 2.0 * Math.tan(hoz_fov / 2);
    double vph = 2.0 * Math.tan(vert_fov / 2);

    double screenSpaceX = vpw / 2.0
        * -normalizedScreenPoint.getX(); // X is negated TODO: Move this negation elsewhere
    double screenSpaceY = vph / 2.0 * normalizedScreenPoint.getY();

    return new Vector2D(
        Math.PI / 2 - Math.atan2(1, screenSpaceX),
        Math.PI / 2 - Math.atan2(1, screenSpaceY)
    );
  }

  public static Line lineFromScreenAngles(Vector2D screenAngles, Vector3D cameraPosition, Rotation cameraRotation) {
    double yaw = screenAngles.getX();
    double pitch = screenAngles.getY();

    Vector3D pixelVector = Vector3D.PLUS_I;

    Rotation pitchRot = new Rotation(Vector3D.PLUS_J, pitch, RotationConvention.FRAME_TRANSFORM);
    Rotation yawRot = new Rotation(Vector3D.PLUS_K, yaw, RotationConvention.FRAME_TRANSFORM);

    pixelVector = pitchRot.applyTo(pixelVector);
    pixelVector = yawRot.applyTo(pixelVector);

    Vector3D pixelVectorRotated = cameraRotation.applyTo(pixelVector);

    return new Line(cameraPosition, cameraPosition.add(pixelVectorRotated), CM_TOLERANCE);
  }

  public static Vector3D getIntersection(Line line, double height) {
    return new Plane(Vector3D.PLUS_K.scalarMultiply(height), Vector3D.PLUS_K, CM_TOLERANCE)
        .intersection(line);
  }

  public static Vector3D midpoint(Vector3D a, Vector3D b) {
    return new Vector3D(
        (a.getX() + b.getX()) / 2,
        (a.getY() + b.getY()) / 2,
        (a.getZ() + b.getZ()) / 2
    );
  }

  public static double angleFromVisionTargets(Vector2D left, Vector2D right) {
    Vector2D difference = left.subtract(right);
    double atan = Math.atan(difference.getY() / difference.getX())+Math.PI/2;
    if (atan > Math.PI/2) {
      atan = atan-Math.PI;
    }
    return atan;
  }

  private static Vector2D xyFromVector3D(Vector3D vec) {
    return new Vector2D(vec.getX(), vec.getY());
  }

  public static Transform3D poseFromTwoCamPoints(Vector2D leftAngles, Vector2D rightAngles, double planeHeight, Vector3D cameraPosition, Rotation cameraRotation, double hoz_fov, double vert_fov) {

    Vector3D leftPoint = getIntersection(lineFromScreenAngles(anglesFromScreenSpace(leftAngles, hoz_fov, vert_fov), cameraPosition, cameraRotation), planeHeight);
    Vector3D rightPoint = getIntersection(lineFromScreenAngles(anglesFromScreenSpace(rightAngles, hoz_fov, vert_fov), cameraPosition, cameraRotation), planeHeight);

    return new Transform3D(
        midpoint(leftPoint, rightPoint),
        new Rotation(RotationOrder.XYZ, RotationConvention.FRAME_TRANSFORM, 0, 0, angleFromVisionTargets(
            xyFromVector3D(leftPoint),
            xyFromVector3D(rightPoint))));
  }
}
