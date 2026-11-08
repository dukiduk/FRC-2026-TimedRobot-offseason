package first.robot.util.telepathy.util;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import java.awt.geom.Area;
import java.awt.geom.Path2D;
import java.awt.geom.PathIterator;
import java.util.ArrayList;

/** Geometry utilities for working with translations, rotations, transforms, and poses. */
/** Also helps with polygon operations for Zones */
public class AreaUtil {

  private AreaUtil() {}

  /**
   * Converts a WPILib Pose2d array into a java.awt.geom.Area object.
   * Skips null separators if present in the input.
   */
  private static Area createAreaFromPoses(Pose2d[] vertices) {
    if (vertices == null || vertices.length < 3) {
      return new Area();
    }
    Path2D.Double path = new Path2D.Double();
    boolean started = false;
    
    for (Pose2d vertex : vertices) {
      if (vertex == null) {
        if (started) {
          path.closePath();
          started = false;
        }
        continue;
      }
      
      if (!started) {
        path.moveTo(vertex.getX(), vertex.getY());
        started = true;
      } else {
        path.lineTo(vertex.getX(), vertex.getY());
      }
    }
    
    if (started) {
      path.closePath();
    }
    return new Area(path);
  }

  /**
   * Converts an Area back into a single flat Pose2d[] array.
   * Inserts a 'null' element between disjoint shapes or holes so AdvantageScope 
   * renders them as distinct boundaries without ugly cross-connecting lines.
   */
  private static Pose2d[] createFlatPosesFromArea(Area area) {
    ArrayList<Pose2d> flatList = new ArrayList<>();
    PathIterator iterator = area.getPathIterator(null);
    double[] coords = new double[6];

    while (!iterator.isDone()) {
      int type = iterator.currentSegment(coords);
      switch (type) {
        case PathIterator.SEG_MOVETO:
          if (!flatList.isEmpty()) {
            flatList.add(null);
          }
          flatList.add(new Pose2d(coords[0], coords[1], Rotation2d.ZERO));
          break;
        case PathIterator.SEG_LINETO:
          flatList.add(new Pose2d(coords[0], coords[1], Rotation2d.ZERO));
          break;
        case PathIterator.SEG_CLOSE:
          break;
      }
      iterator.next();
    }
    return flatList.toArray(new Pose2d[0]);
  }

  /**
   * Combines both shapes into a unified boundary array.
   */
  public static Pose2d[] union(Pose2d[] polyA, Pose2d[] polyB) {
    Area areaA = createAreaFromPoses(polyA);
    Area areaB = createAreaFromPoses(polyB);
    areaA.add(areaB);
    return createFlatPosesFromArea(areaA);
  }

  /**
   * Finds the overlapping area between both shapes as a single array.
   */
  public static Pose2d[] intersection(Pose2d[] polyA, Pose2d[] polyB) {
    Area areaA = createAreaFromPoses(polyA);
    Area areaB = createAreaFromPoses(polyB);
    areaA.intersect(areaB);
    return createFlatPosesFromArea(areaA);
  }

  /**
   * Subtracts PolyB from PolyA (A - B) as a single array.
   */
  public static Pose2d[] difference(Pose2d[] polyA, Pose2d[] polyB) {
    Area areaA = createAreaFromPoses(polyA);
    Area areaB = createAreaFromPoses(polyB);
    areaA.subtract(areaB);
    return createFlatPosesFromArea(areaA);
  }

}
