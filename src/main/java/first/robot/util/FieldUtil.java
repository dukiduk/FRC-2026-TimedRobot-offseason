package first.robot.util;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.Alliance;
import java.util.Optional;

public class FieldUtil {

  private FieldUtil() {}

  public static final double FIELD_MAX_X = 16.518;
  public static final double FIELD_MAX_Y = 8.043;

  public static Alliance alliance = null;

  public static boolean isRedAlliance() {
    if (alliance != null) return alliance == Alliance.RED;

    Optional<Alliance> alliance = MatchState.getAlliance();
    alliance.ifPresent((all) -> FieldUtil.alliance = all);

    return alliance.isPresent() && alliance.get() == Alliance.RED;
  }

  public static Rotation2d getZeroRotation() {
    if (isRedAlliance()) {
      return Rotation2d.k180deg;
    } else {
      return Rotation2d.ZERO;
    }
  }

  public static Pose2d allianceRelativePose(Pose2d pose) {
    return isRedAlliance()
        ? new Pose2d(
            FIELD_MAX_X - pose.getX(),
            FIELD_MAX_Y - pose.getY(),
            pose.getRotation().rotateBy(Rotation2d.PI))
        : pose;
  }

  public static int allianceRelativeFiducial(int id) {
    //You could give even a red tag id and it works
    if (id > 16) {
      return isRedAlliance() ? id - 16 : id;
    } else {
      return isRedAlliance() ? id : id + 16;
    }
    
  }

  public static Pose2d facePose(Pose2d pose, Pose2d facing) {
    Translation2d relativeTrl = facing.relativeTo(pose).getTranslation();
    return new Pose2d(pose.getTranslation(), new Rotation2d(relativeTrl.getX(), relativeTrl.getY()));
  }
}