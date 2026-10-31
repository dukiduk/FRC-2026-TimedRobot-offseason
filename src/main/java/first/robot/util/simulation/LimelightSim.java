package first.robot.util.simulation;

import io.avaje.jsonb.Jsonb;
import org.wpilib.fields.FieldTag;
import org.wpilib.fields.Field;
import org.wpilib.fields.Fields;
import org.wpilib.math.util.Units;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.networktables.DoubleArrayEntry;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableEntry;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.framework.RobotBase;
import org.wpilib.system.Timer;
import org.wpilib.smartdashboard.Field2d;
import org.wpilib.smartdashboard.FieldObject2d;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Random;
import com.limelightvision.Limelight;
import com.limelightvision.LimelightResults;
import com.limelightvision.FiducialTarget;

/**
 * Simulates a {@link Limelight} running the AprilTag/fiducial pipeline...
 */
public class LimelightSim {

  @SuppressWarnings("unused")
  private final Limelight limelight;
  private final NetworkTable table;
  private Random noise;
  private LimelightSimSettings settings;
  private Transform3d robotToCamera = new Transform3d();
  private Field fieldLayout;
  @SuppressWarnings("unused")
  private Field2d field2d;
  private FieldObject2d raycasts;

  /**
   * Avaje Jsonb instance used to publish the "json" results entry.
   */
  private final Jsonb jsonb = Jsonb.builder().build();

  private long heartbeat = 0;
  private long frameIndex = 0;

  private final NetworkTableEntry tvEntry;
  private final NetworkTableEntry txEntry;
  private final NetworkTableEntry tyEntry;
  private final NetworkTableEntry txncEntry;
  private final NetworkTableEntry tyncEntry;
  private final NetworkTableEntry taEntry;
  private final NetworkTableEntry tidEntry;
  private final NetworkTableEntry tlEntry;
  private final NetworkTableEntry clEntry;
  private final NetworkTableEntry tdistEntry;
  private final NetworkTableEntry hbEntry;
  private final NetworkTableEntry getpipeEntry;
  private final NetworkTableEntry getpipetypeEntry;
  private final NetworkTableEntry jsonEntry;
  private final NetworkTableEntry pipelineIndexEntry;
  private final DoubleArrayEntry  t2dEntry;
  private final DoubleArrayEntry  rawFiducialsEntry;
  private final DoubleArrayEntry  targetPoseRobotSpaceEntry;
  private final DoubleArrayEntry  targetPoseCameraSpaceEntry;
  private final DoubleArrayEntry  cameraPoseTargetSpaceEntry;
  private final DoubleArrayEntry  botPoseTargetSpaceEntry;
  private final DoubleArrayEntry  cameraPoseRobotSpaceEntry;
  private final DoubleArrayEntry  stddevsEntry;
  private final DoubleArrayEntry  imuEntry;
  private final DoubleArrayEntry  botposeEntry;
  private final DoubleArrayEntry  botposeRedEntry;
  private final DoubleArrayEntry  botposeBlueEntry;
  private final DoubleArrayEntry  botposeOrbEntry;
  private final DoubleArrayEntry  botposeOrbBlueEntry;
  private final DoubleArrayEntry  botposeOrbRedEntry;
  private final DoubleArrayEntry  robotOrientationSetEntry;

  public LimelightSim(Limelight limelight) {
    this(limelight, new LimelightSimSettings());
  }

  public LimelightSim(Limelight limelight, LimelightSimSettings settings) {
    this.limelight = limelight;
    this.table = NetworkTableInstance.getDefault().getTable(limelight.getName());
    this.settings = settings;
    this.noise = new Random(settings.randomSeed);
    this.fieldLayout = Field.loadField(Fields.DEFAULT_FIELD);

    tvEntry = table.getEntry("tv");
    txEntry = table.getEntry("tx");
    tyEntry = table.getEntry("ty");
    txncEntry = table.getEntry("txnc");
    tyncEntry = table.getEntry("tync");
    taEntry = table.getEntry("ta");
    tidEntry = table.getEntry("tid");
    tlEntry = table.getEntry("tl");
    clEntry = table.getEntry("cl");
    tdistEntry = table.getEntry("tdist");
    hbEntry = table.getEntry("hb");
    getpipeEntry = table.getEntry("getpipe");
    getpipetypeEntry = table.getEntry("getpipetype");
    jsonEntry = table.getEntry("json");
    pipelineIndexEntry = table.getEntry("pipeline");
    t2dEntry = table.getDoubleArrayTopic("t2d").getEntry(new double[0]);
    rawFiducialsEntry = table.getDoubleArrayTopic("rawfiducials").getEntry(new double[0]);
    targetPoseRobotSpaceEntry = table.getDoubleArrayTopic("targetpose_robotspace").getEntry(new double[0]);
    targetPoseCameraSpaceEntry = table.getDoubleArrayTopic("targetpose_cameraspace").getEntry(new double[0]);
    cameraPoseTargetSpaceEntry = table.getDoubleArrayTopic("camerapose_targetspace").getEntry(new double[0]);
    botPoseTargetSpaceEntry = table.getDoubleArrayTopic("botpose_targetspace").getEntry(new double[0]);
    cameraPoseRobotSpaceEntry = table.getDoubleArrayTopic("camerapose_robotspace").getEntry(new double[0]);
    stddevsEntry = table.getDoubleArrayTopic("stddevs").getEntry(new double[0]);
    imuEntry = table.getDoubleArrayTopic("imu").getEntry(new double[10]);
    botposeEntry = table.getDoubleArrayTopic("botpose").getEntry(new double[0]);
    botposeRedEntry = table.getDoubleArrayTopic("botpose_wpired").getEntry(new double[0]);
    botposeBlueEntry = table.getDoubleArrayTopic("botpose_wpiblue").getEntry(new double[0]);
    botposeOrbEntry = table.getDoubleArrayTopic("botpose_orb").getEntry(new double[0]);
    botposeOrbBlueEntry = table.getDoubleArrayTopic("botpose_orb_wpiblue").getEntry(new double[0]);
    botposeOrbRedEntry = table.getDoubleArrayTopic("botpose_orb_wpired").getEntry(new double[0]);
    robotOrientationSetEntry = table.getDoubleArrayTopic("robot_orientation_set").getEntry(new double[0]);
  }

  public LimelightSim withRobotToCameraTransform(Transform3d robotToCamera) {
    this.robotToCamera = robotToCamera;
    return this;
  }

  public LimelightSim withField(Field fieldLayout) {
    this.fieldLayout = fieldLayout;
    return this;
  }

  public LimelightSim withField2d(Field2d field2d) {
    this.field2d = field2d;

    if (field2d == null) {
      raycasts = null;
    } else {
      raycasts = field2d.getObject("Limelight Raycasts");
      raycasts.setPoses();
    }

    return this;
  }

  public LimelightSim withSettings(LimelightSimSettings settings) {
    this.settings = settings;
    this.noise = new Random(settings.randomSeed);
    return this;
  }

  public void update(Pose2d robotPoseMeters) {
    update(new Pose3d(robotPoseMeters));
  }

  public void update(Pose3d robotPose) {
    if (!RobotBase.isSimulation()) {
      return;
    }

    Pose3d cameraPose = robotPose.plus(robotToCamera);

    List<TagObservation> visible = new ArrayList<>();

    for (FieldTag tag : fieldLayout.getTags()) {
      project(cameraPose, robotPose, tag).ifPresent(visible::add);
    }

    visible.sort((a, b) -> Double.compare(b.ta, a.ta));

    drawRaycasts(robotPose, visible);

    int tagCount = visible.size();
    TagObservation primary = tagCount > 0 ? visible.get(0) : null;

    double avgTagDist = 0;
    double avgTagArea = 0;
    double tagSpan = 0;

    for (TagObservation obs : visible) {
      avgTagDist += obs.distToCamera;
      avgTagArea += obs.ta;
    }

    if (tagCount > 0) {
      avgTagDist /= tagCount;
      avgTagArea /= tagCount;
    }

    for (int i = 0; i < visible.size(); i++) {
      for (int j = i + 1; j < visible.size(); j++) {
        tagSpan = Math.max(tagSpan, visible.get(i).pose.getTranslation().getDistance(visible.get(j).pose.getTranslation()));
      }
    }

    double tlMs = Math.max(0, settings.avgPipelineLatencyMs + noise.nextGaussian() * settings.latencyStdDevMs);
    double clMs = settings.avgCaptureLatencyMs;
    double totalLatencyMs = tlMs + clMs;

    double translationStdDev = tagCount > 0 ? settings.translationNoiseStdDevMeters * (1 + avgTagDist) / Math.sqrt(tagCount) : 0;
    double rotationStdDev = tagCount > 0 ? settings.rotationNoiseStdDevDegrees / Math.sqrt(tagCount) : 0;

    Pose3d mt1Pose = tagCount > 0 ? addNoise(robotPose, translationStdDev, rotationStdDev) : robotPose;
    Pose3d mt2Pose = tagCount > 0 ? addNoise(robotPose, translationStdDev / 2.0, 0) : robotPose;

    double[] suppliedOrientation = robotOrientationSetEntry.get();

    if (suppliedOrientation.length >= 1) {
      Rotation3d groundTruthRotation = mt2Pose.getRotation();
      mt2Pose = new Pose3d(mt2Pose.getTranslation(), new Rotation3d(groundTruthRotation.getX(), groundTruthRotation.getY(), Math.toRadians(suppliedOrientation[0])));
    }

    Pose3d mt1PoseRed = flipToRed(mt1Pose);
    Pose3d mt2PoseRed = flipToRed(mt2Pose);

    publishScalarEntries(primary, tagCount, tlMs, clMs);

    publishArrayEntries(robotPose,
                        cameraPose,
                        visible,
                        primary,
                        tagCount,
                        tagSpan,
                        avgTagDist,
                        avgTagArea,
                        totalLatencyMs,
                        translationStdDev,
                        rotationStdDev,
                        mt1Pose,
                        mt1PoseRed,
                        mt2Pose,
                        mt2PoseRed);

    publishJson(robotPose, cameraPose, visible, tagCount, tagSpan, avgTagDist, avgTagArea, tlMs, clMs, primary, mt1Pose, mt1PoseRed, mt2Pose);

    heartbeat++;
    frameIndex++;
  }

  private void drawRaycasts(Pose3d robotPose, List<TagObservation> visible) {
    if (raycasts == null) {
      return;
    }

    Pose2d robot = robotPose.toPose2d();

    if (visible.isEmpty()) {
      raycasts.setPoses();
      return;
    }

    List<Pose2d> poses = new ArrayList<>(visible.size() * 2);

    for (TagObservation observation : visible) {
      poses.add(robot);
      poses.add(observation.pose.toPose2d());
    }

    raycasts.setPoses(poses.toArray(new Pose2d[0]));
  }

  private void publishScalarEntries(TagObservation primary, int tagCount, double tlMs, double clMs) {
    tvEntry.setDouble(tagCount > 0 ? 1 : 0);
    txEntry.setDouble(primary != null ? primary.tx : 0);
    tyEntry.setDouble(primary != null ? primary.ty : 0);
    txncEntry.setDouble(primary != null ? primary.tx : 0);
    tyncEntry.setDouble(primary != null ? primary.ty : 0);
    taEntry.setDouble(primary != null ? primary.ta : 0);
    tidEntry.setDouble(primary != null ? primary.id : -1);
    tlEntry.setDouble(tlMs);
    clEntry.setDouble(clMs);
    tdistEntry.setDouble(primary != null ? primary.distToCamera : 0);
    hbEntry.setDouble(heartbeat);

    double pipelineIndex = pipelineIndexEntry.getDouble(0);

    getpipeEntry.setDouble(pipelineIndex);
    getpipetypeEntry.setString(settings.pipelineType);
  }

  private void publishArrayEntries(Pose3d robotPose, Pose3d cameraPose, List<TagObservation> visible, TagObservation primary, int tagCount, double tagSpan, double avgTagDist, double avgTagArea,
                                   double totalLatencyMs, double translationStdDev, double rotationStdDev, Pose3d mt1Pose, Pose3d mt1PoseRed, Pose3d mt2Pose, Pose3d mt2PoseRed) {
    double[] rawFiducials = new double[7 * tagCount];

    for (int i = 0; i < visible.size(); i++) {
      TagObservation obs = visible.get(i);
      int base = i * 7;

      rawFiducials[base] = obs.id;
      rawFiducials[base + 1] = obs.tx;
      rawFiducials[base + 2] = obs.ty;
      rawFiducials[base + 3] = obs.ta;
      rawFiducials[base + 4] = obs.distToCamera;
      rawFiducials[base + 5] = obs.distToRobot;
      rawFiducials[base + 6] = obs.ambiguity;
    }

    rawFiducialsEntry.set(rawFiducials);

    botposeEntry.set(botPoseArray(mt1Pose, totalLatencyMs, tagCount, tagSpan, avgTagDist, avgTagArea, rawFiducials));
    botposeBlueEntry.set(botPoseArray(mt1Pose, totalLatencyMs, tagCount, tagSpan, avgTagDist, avgTagArea, rawFiducials));
    botposeRedEntry.set(botPoseArray(mt1PoseRed, totalLatencyMs, tagCount, tagSpan, avgTagDist, avgTagArea, rawFiducials));
    botposeOrbEntry.set(botPoseArray(mt2Pose, totalLatencyMs, tagCount, tagSpan, avgTagDist, avgTagArea, rawFiducials));
    botposeOrbBlueEntry.set(botPoseArray(mt2Pose, totalLatencyMs, tagCount, tagSpan, avgTagDist, avgTagArea, rawFiducials));
    botposeOrbRedEntry.set(botPoseArray(mt2PoseRed, totalLatencyMs, tagCount, tagSpan, avgTagDist, avgTagArea, rawFiducials));

    double[] t2d = new double[]{tagCount > 0 ? 1 : 0, tagCount, totalLatencyMs, 0, primary != null ? primary.tx : 0, primary != null ? primary.ty : 0, primary != null ? primary.tx : 0,
                                primary != null ? primary.ty : 0, primary != null ? primary.ta : 0, primary != null ? primary.id : -1, -1, -1, primary != null ? primary.apparentWidthPixels : 0,
                                primary != null ? primary.apparentWidthPixels : 0, primary != null ? primary.apparentWidthPixels : 0, primary != null ? primary.apparentWidthPixels : 0, 0};

    t2dEntry.set(t2d);

    if (primary != null) {
      Pose3d tagPose = primary.pose;

      targetPoseRobotSpaceEntry.set(pose3dToArray(tagPose.relativeTo(robotPose)));
      targetPoseCameraSpaceEntry.set(pose3dToArray(tagPose.relativeTo(cameraPose)));
      cameraPoseTargetSpaceEntry.set(pose3dToArray(cameraPose.relativeTo(tagPose)));
      botPoseTargetSpaceEntry.set(pose3dToArray(robotPose.relativeTo(tagPose)));
    } else {
      targetPoseRobotSpaceEntry.set(new double[0]);
      targetPoseCameraSpaceEntry.set(new double[0]);
      cameraPoseTargetSpaceEntry.set(new double[0]);
      botPoseTargetSpaceEntry.set(new double[0]);
    }

    cameraPoseRobotSpaceEntry.set(pose3dToArray(new Pose3d(robotToCamera.getTranslation(), robotToCamera.getRotation())));

    stddevsEntry.set(new double[]{translationStdDev, translationStdDev, translationStdDev, rotationStdDev, rotationStdDev, rotationStdDev, translationStdDev / 2.0, translationStdDev / 2.0,
                                  translationStdDev / 2.0, 0, 0, 0});

    double robotYawDeg   = Math.toDegrees(robotPose.getRotation().getZ());
    double robotPitchDeg = Math.toDegrees(robotPose.getRotation().getY());
    double robotRollDeg  = Math.toDegrees(robotPose.getRotation().getX());

    imuEntry.set(new double[]{robotYawDeg, robotRollDeg, robotPitchDeg, robotYawDeg, 0, 0, 0, 0, 0, 0});
  }

  private void publishJson(Pose3d robotPose, Pose3d cameraPose, List<TagObservation> visible, int tagCount, double tagSpan, double avgTagDist, double avgTagArea, double tlMs, double clMs,
                           TagObservation primary, Pose3d mt1Pose, Pose3d mt1PoseRed, Pose3d mt2Pose) {
    double nowSeconds = Timer.getMonotonicTimestamp();

    LimelightResults results = new LimelightResults();

    results.error = "";
    results.txDegrees = primary != null ? primary.tx : 0;
    results.tyDegrees = primary != null ? primary.ty : 0;
    results.txDegreesNoCrosshair = results.txDegrees;
    results.tyDegreesNoCrosshair = results.tyDegrees;
    results.targetAreaPercent = primary != null ? primary.ta : 0;
    results.pipelineIndex = pipelineIndexEntry.getDouble(0);
    results.pipelineType = settings.pipelineType;
    results.targetingLatencyMillis = tlMs;
    results.captureLatencyMillis = clMs;
    results.timestampMillis = nowSeconds * 1000.0;
    results.timestampMicros = (long) (nowSeconds * 1_000_000);
    results.frameIndex = frameIndex;
    results.valid = tagCount > 0;
    results.robotPoseMT1 = pose3dToArray(mt1Pose);
    results.robotPoseMT1WpiBlue = pose3dToArray(mt1Pose);
    results.robotPoseMT1WpiRed = pose3dToArray(mt1PoseRed);
    results.robotPoseMT2 = pose3dToArray(mt2Pose);
    results.robotPoseMT2WpiBlue = pose3dToArray(mt2Pose);
    results.reportedTagCount = tagCount;
    results.tagSpanMeters = tagSpan;
    results.avgTagDistanceMeters = avgTagDist;
    results.avgTagAreaPercent = avgTagArea;

    results.cameraPoseRobotSpace = pose3dToArray(new Pose3d(robotToCamera.getTranslation(), robotToCamera.getRotation()));
    results.targetDistanceMeters = primary != null ? primary.distToCamera : 0;
    results.hardwareType = 6769;

    FiducialTarget[] fiducials = new FiducialTarget[visible.size()];

    for (int i = 0; i < visible.size(); i++) {
      TagObservation obs = visible.get(i);

      fiducials[i] = new FiducialTarget();
      fiducials[i].fiducialId = obs.id;
      fiducials[i].family = "36h11";
      fiducials[i].targetAreaPercent = obs.ta;
      fiducials[i].txDegrees = obs.tx;
      fiducials[i].txDegrees = obs.ty;
      fiducials[i].txPixels = obs.tx * obs.pixelsPerDegree;
      fiducials[i].tyPixels = obs.ty * obs.pixelsPerDegree;
      fiducials[i].txDegreesNoCrosshair = obs.tx;
      fiducials[i].tyDegreesNoCrosshair = obs.ty;
      fiducials[i].cameraPoseTargetSpace = pose3dToArray(cameraPose.relativeTo(obs.pose));
      fiducials[i].robotPoseFieldSpace = pose3dToArray(robotPose);
      fiducials[i].robotPoseTargetSpace = pose3dToArray(robotPose.relativeTo(obs.pose));
      fiducials[i].targetPoseCameraSpace = pose3dToArray(obs.pose.relativeTo(cameraPose));
      fiducials[i].targetPoseRobotSpace = pose3dToArray(obs.pose.relativeTo(robotPose));
    }

    results.fiducialTargets = fiducials;

    try {
      jsonEntry.setString(jsonb.toJson(results));
    } catch (Exception e) {
      jsonEntry.setString("");
    }
  }

  private double[] botPoseArray(Pose3d pose, double latencyMs, int tagCount, double tagSpan, double avgTagDist, double avgTagArea, double[] rawFiducials) {
    if (tagCount == 0) {
      return new double[0];
    }

    double[] pose6  = pose3dToArray(pose);
    double[] result = new double[11 + rawFiducials.length];

    System.arraycopy(pose6, 0, result, 0, 6);

    result[6] = latencyMs;
    result[7] = tagCount;
    result[8] = tagSpan;
    result[9] = avgTagDist;
    result[10] = avgTagArea;

    System.arraycopy(rawFiducials, 0, result, 11, rawFiducials.length);

    return result;
  }

  private Pose3d flipToRed(Pose3d bluePose) {
    Rotation3d rotation = bluePose.getRotation();

    return new Pose3d(fieldLayout.getFieldLength() - bluePose.getX(),
                      fieldLayout.getFieldWidth() - bluePose.getY(),
                      bluePose.getZ(),
                      new Rotation3d(rotation.getX(), rotation.getY(), rotation.getZ() + Math.PI));
  }

  private Pose3d addNoise(Pose3d pose, double translationStdDev, double rotationStdDevDegrees) {
    if (translationStdDev <= 0 && rotationStdDevDegrees <= 0) {
      return pose;
    }

    double x = pose.getX() + noise.nextGaussian() * translationStdDev;
    double y = pose.getY() + noise.nextGaussian() * translationStdDev;
    double z = pose.getZ() + noise.nextGaussian() * translationStdDev * 0.5;

    Rotation3d rotation = pose.getRotation();

    double yaw = rotation.getZ() + Math.toRadians(noise.nextGaussian() * rotationStdDevDegrees);

    return new Pose3d(x, y, z, new Rotation3d(rotation.getX(), rotation.getY(), yaw));
  }

  private Optional<TagObservation> project(Pose3d cameraPose, Pose3d robotPose, FieldTag tag) {
    Pose3d tagPose = tag.getPose();

    Transform3d camToTag = new Transform3d(cameraPose, tagPose);

    double x = camToTag.getX();
    double y = camToTag.getY();
    double z = camToTag.getZ();

    if (x <= 0.02) {
      return Optional.empty();
    }

    double distance = camToTag.getTranslation().getNorm();

    if (distance > settings.maxDetectionRangeMeters) {
      return Optional.empty();
    }

    double halfHFovDeg = settings.horizontalFOV.getDegrees() / 2.0;
    double halfVFovDeg = settings.verticalFOV.getDegrees() / 2.0;

    double yawDeg = Math.toDegrees(Math.atan2(-y, x));
    double pitchDeg = Math.toDegrees(Math.atan2(-z, Math.hypot(x, y)));

    if (Math.abs(yawDeg) > halfHFovDeg || Math.abs(pitchDeg) > halfVFovDeg) {
      return Optional.empty();
    }

    Translation3d tagNormal = new Translation3d(1, 0, 0).rotateBy(tagPose.getRotation());
    Translation3d tagToCam = cameraPose.getTranslation().minus(tagPose.getTranslation());

    double dot = tagNormal.getX() * tagToCam.getX() + tagNormal.getY() * tagToCam.getY() + tagNormal.getZ() * tagToCam.getZ();
    double incidenceDeg = Math.toDegrees(Math.acos(Math.clamp(dot / tagToCam.getNorm(), -1, 1)));

    if (incidenceDeg > settings.maxTagIncidenceAngleDegrees) {
      return Optional.empty();
    }

    double fx = (settings.resolutionWidth / 2.0) / Math.tan(Math.toRadians(halfHFovDeg));
    double apparentWidthPx = fx * settings.tagSizeMeters * Math.cos(Math.toRadians(incidenceDeg)) / x;
    double areaPx = apparentWidthPx * apparentWidthPx;
    double ta = Math.clamp(areaPx / (settings.resolutionWidth * settings.resolutionHeight) * 100.0, 0, 100);

    double noisyYaw = yawDeg + noise.nextGaussian() * settings.angleNoiseStdDevDegrees;
    double noisyPitch = pitchDeg + noise.nextGaussian() * settings.angleNoiseStdDevDegrees;

    double distToRobot = robotPose.getTranslation().getDistance(tagPose.getTranslation());
    double ambiguity = Math.clamp(noise.nextDouble() * 0.05 + (incidenceDeg / 90.0) * 0.15, 0, 1);
    double pixelsPerDegree = (settings.resolutionWidth / 2.0) / halfHFovDeg;

    return Optional.of(new TagObservation(tag.getID(), tagPose, ta, noisyYaw, noisyPitch, distance, distToRobot, ambiguity, apparentWidthPx, pixelsPerDegree));
  }

  public static double[] pose3dToArray(Pose3d pose) {
    double[] result = new double[6];
    result[0] = pose.getTranslation().getX();
    result[1] = pose.getTranslation().getY();
    result[2] = pose.getTranslation().getZ();
    result[3] = Units.radiansToDegrees(pose.getRotation().getX());
    result[4] = Units.radiansToDegrees(pose.getRotation().getY());
    result[5] = Units.radiansToDegrees(pose.getRotation().getZ());
    return result;
  }

  private static class TagObservation {
    final int id;
    final Pose3d pose;
    final double ta;
    final double tx;
    final double ty;
    final double distToCamera;
    final double distToRobot;
    final double ambiguity;
    final double apparentWidthPixels;
    final double pixelsPerDegree;

    TagObservation(int id, Pose3d pose, double ta, double tx, double ty, double distToCamera, double distToRobot, double ambiguity, double apparentWidthPixels, double pixelsPerDegree) {
      this.id = id;
      this.pose = pose;
      this.ta = ta;
      this.tx = tx;
      this.ty = ty;
      this.distToCamera = distToCamera;
      this.distToRobot = distToRobot;
      this.ambiguity = ambiguity;
      this.apparentWidthPixels = apparentWidthPixels;
      this.pixelsPerDegree = pixelsPerDegree;
    }
  }
}