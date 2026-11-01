package first.robot.util.simulation;

import io.avaje.jsonb.Jsonb;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Random;

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
import org.wpilib.networktables.RawPublisher;
import org.wpilib.framework.RobotBase;
import org.wpilib.system.Timer;
import org.wpilib.smartdashboard.Field2d;
import org.wpilib.smartdashboard.FieldObject2d;

import com.limelightvision.Limelight;
import com.limelightvision.LimelightResults;
import com.limelightvision.FiducialTarget;

/**
 * Simulates a Limelight running the AprilTag/fiducial pipeline.
 *
 * Publishes both legacy NetworkTables results and the MessagePack
 * results frame used by LimelightLib 2.
 * 
 * I will prob remove individual NT4 entries if this works
 */
public class LimelightSim {

  private final Limelight limelight;

  private final NetworkTable table;
  private Random noise;
  private LimelightSimSettings settings;
  private Transform3d robotToCamera = new Transform3d();
  private Field fieldLayout;

  private Field2d field2d;

  private FieldObject2d raycasts;

  /**
   * Avaje Jsonb instance used to publish the legacy "json" results entry.
   */
  private final Jsonb jsonb = Jsonb.builder().build();

  /**
   * MessagePack publisher consumed by LimelightLib 2.
   */
  private final RawPublisher resultsPublisher;

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

  private final DoubleArrayEntry t2dEntry;
  private final DoubleArrayEntry rawFiducialsEntry;
  private final DoubleArrayEntry targetPoseRobotSpaceEntry;
  private final DoubleArrayEntry targetPoseCameraSpaceEntry;
  private final DoubleArrayEntry cameraPoseTargetSpaceEntry;
  private final DoubleArrayEntry botPoseTargetSpaceEntry;
  private final DoubleArrayEntry cameraPoseRobotSpaceEntry;
  private final DoubleArrayEntry stddevsEntry;
  private final DoubleArrayEntry imuEntry;
  private final DoubleArrayEntry botposeEntry;
  private final DoubleArrayEntry botposeRedEntry;
  private final DoubleArrayEntry botposeBlueEntry;
  private final DoubleArrayEntry botposeOrbEntry;
  private final DoubleArrayEntry botposeOrbBlueEntry;
  private final DoubleArrayEntry botposeOrbRedEntry;

  /**
   * Shared Limelight robot orientation topic.
   */
  private final DoubleArrayEntry robotOrientationSetEntry;

  public LimelightSim(Limelight limelight) {
    this(limelight, new LimelightSimSettings());
  }

  public LimelightSim(Limelight limelight, LimelightSimSettings settings) {
    this.limelight = limelight;
    this.settings = settings;
    this.noise = new Random(settings.randomSeed);
    this.fieldLayout = Field.loadField(Fields.DEFAULT_FIELD);

    NetworkTableInstance nt = NetworkTableInstance.getDefault();

    table = nt.getTable(limelight.getName());

    resultsPublisher = nt
        .getRawTopic(limelight.getName() + "/results_msgpack")
        .publish("msgpack");

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

    targetPoseRobotSpaceEntry =
        table.getDoubleArrayTopic("targetpose_robotspace").getEntry(new double[0]);

    targetPoseCameraSpaceEntry =
        table.getDoubleArrayTopic("targetpose_cameraspace").getEntry(new double[0]);

    cameraPoseTargetSpaceEntry =
        table.getDoubleArrayTopic("camerapose_targetspace").getEntry(new double[0]);

    botPoseTargetSpaceEntry =
        table.getDoubleArrayTopic("botpose_targetspace").getEntry(new double[0]);

    cameraPoseRobotSpaceEntry =
        table.getDoubleArrayTopic("camerapose_robotspace").getEntry(new double[0]);

    stddevsEntry = table.getDoubleArrayTopic("stddevs").getEntry(new double[0]);
    imuEntry = table.getDoubleArrayTopic("imu").getEntry(new double[10]);

    botposeEntry = table.getDoubleArrayTopic("botpose").getEntry(new double[0]);
    botposeRedEntry = table.getDoubleArrayTopic("botpose_wpired").getEntry(new double[0]);
    botposeBlueEntry = table.getDoubleArrayTopic("botpose_wpiblue").getEntry(new double[0]);

    botposeOrbEntry = table.getDoubleArrayTopic("botpose_orb").getEntry(new double[0]);
    botposeOrbBlueEntry =
        table.getDoubleArrayTopic("botpose_orb_wpiblue").getEntry(new double[0]);
    botposeOrbRedEntry =
        table.getDoubleArrayTopic("botpose_orb_wpired").getEntry(new double[0]);

    /*
     * This is a shared Limelight topic, not a camera-specific topic.
     */
    robotOrientationSetEntry = nt
        .getDoubleArrayTopic("limelightshared/robot_orientation_set")
        .getEntry(new double[0]);
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
        tagSpan = Math.max(
            tagSpan,
            visible.get(i).pose.getTranslation()
                .getDistance(visible.get(j).pose.getTranslation()));
      }
    }

    double tlMs = Math.max(
        0,
        settings.avgPipelineLatencyMs
            + noise.nextGaussian() * settings.latencyStdDevMs);

    double clMs = settings.avgCaptureLatencyMs;
    double totalLatencyMs = tlMs + clMs;

    double translationStdDev = tagCount > 0
        ? settings.translationNoiseStdDevMeters * (1 + avgTagDist) / Math.sqrt(tagCount)
        : 0;

    double rotationStdDev = tagCount > 0
        ? settings.rotationNoiseStdDevDegrees / Math.sqrt(tagCount)
        : 0;

    Pose3d mt1Pose = tagCount > 0
        ? addNoise(robotPose, translationStdDev, rotationStdDev)
        : robotPose;

    Pose3d mt2Pose = tagCount > 0
        ? addNoise(robotPose, translationStdDev / 2.0, 0)
        : robotPose;

    /*
     * MegaTag2 uses the externally supplied robot yaw when available.
     */
    double[] suppliedOrientation = robotOrientationSetEntry.get();

    if (suppliedOrientation.length >= 1) {
      Rotation3d groundTruthRotation = mt2Pose.getRotation();

      mt2Pose = new Pose3d(
          mt2Pose.getTranslation(),
          new Rotation3d(
              groundTruthRotation.getX(),
              groundTruthRotation.getY(),
              Math.toRadians(suppliedOrientation[0])));
    }

    Pose3d mt1PoseRed = flipToRed(mt1Pose);
    Pose3d mt2PoseRed = flipToRed(mt2Pose);

    publishScalarEntries(primary, tagCount, tlMs, clMs);

    publishArrayEntries(
        robotPose,
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

    publishJson(
        robotPose,
        cameraPose,
        visible,
        tagCount,
        tagSpan,
        avgTagDist,
        avgTagArea,
        tlMs,
        clMs,
        primary,
        mt1Pose,
        mt1PoseRed,
        mt2Pose);

    publishMessagePack(
        robotPose,
        cameraPose,
        visible,
        tagCount,
        tagSpan,
        avgTagDist,
        avgTagArea,
        tlMs,
        clMs,
        primary,
        mt1Pose,
        mt1PoseRed,
        mt2Pose,
        mt2PoseRed);

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

  private void publishScalarEntries(
      TagObservation primary,
      int tagCount,
      double tlMs,
      double clMs) {

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

  private void publishArrayEntries(
      Pose3d robotPose,
      Pose3d cameraPose,
      List<TagObservation> visible,
      TagObservation primary,
      int tagCount,
      double tagSpan,
      double avgTagDist,
      double avgTagArea,
      double totalLatencyMs,
      double translationStdDev,
      double rotationStdDev,
      Pose3d mt1Pose,
      Pose3d mt1PoseRed,
      Pose3d mt2Pose,
      Pose3d mt2PoseRed) {

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

    botposeEntry.set(
        botPoseArray(mt1Pose, totalLatencyMs, tagCount, tagSpan,
            avgTagDist, avgTagArea, rawFiducials));

    botposeBlueEntry.set(
        botPoseArray(mt1Pose, totalLatencyMs, tagCount, tagSpan,
            avgTagDist, avgTagArea, rawFiducials));

    botposeRedEntry.set(
        botPoseArray(mt1PoseRed, totalLatencyMs, tagCount, tagSpan,
            avgTagDist, avgTagArea, rawFiducials));

    botposeOrbEntry.set(
        botPoseArray(mt2Pose, totalLatencyMs, tagCount, tagSpan,
            avgTagDist, avgTagArea, rawFiducials));

    botposeOrbBlueEntry.set(
        botPoseArray(mt2Pose, totalLatencyMs, tagCount, tagSpan,
            avgTagDist, avgTagArea, rawFiducials));

    botposeOrbRedEntry.set(
        botPoseArray(mt2PoseRed, totalLatencyMs, tagCount, tagSpan,
            avgTagDist, avgTagArea, rawFiducials));

    double[] t2d = new double[] {
        tagCount > 0 ? 1 : 0,
        tagCount,
        totalLatencyMs,
        0,
        primary != null ? primary.tx : 0,
        primary != null ? primary.ty : 0,
        primary != null ? primary.tx : 0,
        primary != null ? primary.ty : 0,
        primary != null ? primary.ta : 0,
        primary != null ? primary.id : -1,
        -1,
        -1,
        primary != null ? primary.apparentWidthPixels : 0,
        primary != null ? primary.apparentWidthPixels : 0,
        primary != null ? primary.apparentWidthPixels : 0,
        primary != null ? primary.apparentWidthPixels : 0,
        0
    };

    t2dEntry.set(t2d);

    if (primary != null) {
      Pose3d tagPose = primary.pose;

      targetPoseRobotSpaceEntry.set(
          pose3dToArray(tagPose.relativeTo(robotPose)));

      targetPoseCameraSpaceEntry.set(
          pose3dToArray(tagPose.relativeTo(cameraPose)));

      cameraPoseTargetSpaceEntry.set(
          pose3dToArray(cameraPose.relativeTo(tagPose)));

      botPoseTargetSpaceEntry.set(
          pose3dToArray(robotPose.relativeTo(tagPose)));
    } else {
      targetPoseRobotSpaceEntry.set(new double[0]);
      targetPoseCameraSpaceEntry.set(new double[0]);
      cameraPoseTargetSpaceEntry.set(new double[0]);
      botPoseTargetSpaceEntry.set(new double[0]);
    }

    cameraPoseRobotSpaceEntry.set(
        pose3dToArray(
            new Pose3d(robotToCamera.getTranslation(), robotToCamera.getRotation())));

    stddevsEntry.set(new double[] {
        translationStdDev,
        translationStdDev,
        translationStdDev,
        rotationStdDev,
        rotationStdDev,
        rotationStdDev,
        translationStdDev / 2.0,
        translationStdDev / 2.0,
        translationStdDev / 2.0,
        0,
        0,
        0
    });

    double robotYawDeg = Math.toDegrees(robotPose.getRotation().getZ());
    double robotPitchDeg = Math.toDegrees(robotPose.getRotation().getY());
    double robotRollDeg = Math.toDegrees(robotPose.getRotation().getX());

    imuEntry.set(new double[] {
        robotYawDeg,
        robotRollDeg,
        robotPitchDeg,
        robotYawDeg,
        0,
        0,
        0,
        0,
        0,
        0
    });
  }

  /**
   * Publishes the legacy JSON representation.
   */
  private void publishJson(
      Pose3d robotPose,
      Pose3d cameraPose,
      List<TagObservation> visible,
      int tagCount,
      double tagSpan,
      double avgTagDist,
      double avgTagArea,
      double tlMs,
      double clMs,
      TagObservation primary,
      Pose3d mt1Pose,
      Pose3d mt1PoseRed,
      Pose3d mt2Pose) {

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

    results.cameraPoseRobotSpace = pose3dToArray(
        new Pose3d(robotToCamera.getTranslation(), robotToCamera.getRotation()));

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
      fiducials[i].tyDegrees = obs.ty;
      fiducials[i].txPixels = obs.tx * obs.pixelsPerDegree;
      fiducials[i].tyPixels = obs.ty * obs.pixelsPerDegree;
      fiducials[i].txDegreesNoCrosshair = obs.tx;
      fiducials[i].tyDegreesNoCrosshair = obs.ty;

      fiducials[i].cameraPoseTargetSpace =
          pose3dToArray(cameraPose.relativeTo(obs.pose));

      fiducials[i].robotPoseFieldSpace =
          pose3dToArray(robotPose);

      fiducials[i].robotPoseTargetSpace =
          pose3dToArray(robotPose.relativeTo(obs.pose));

      fiducials[i].targetPoseCameraSpace =
          pose3dToArray(obs.pose.relativeTo(cameraPose));

      fiducials[i].targetPoseRobotSpace =
          pose3dToArray(obs.pose.relativeTo(robotPose));
    }

    results.fiducialTargets = fiducials;

    try {
      jsonEntry.setString(jsonb.toJson(results));
    } catch (Exception e) {
      jsonEntry.setString("");
    }
  }

  /**
   * Publishes one LimelightLib 2 MessagePack results frame.
   *
   * The MessagePack payload is a map containing the frame's results.
   * The Fiducial array contains one map per visible AprilTag.
   */
  private void publishMessagePack(
      Pose3d robotPose,
      Pose3d cameraPose,
      List<TagObservation> visible,
      int tagCount,
      double tagSpan,
      double avgTagDist,
      double avgTagArea,
      double tlMs,
      double clMs,
      TagObservation primary,
      Pose3d mt1Pose,
      Pose3d mt1PoseRed,
      Pose3d mt2Pose,
      Pose3d mt2PoseRed) {

    double nowSeconds = Timer.getMonotonicTimestamp();

    Map<String, Object> frame = new LinkedHashMap<>();

    // Frame metadata.
    frame.put("v", 1);
    frame.put("fidx", frameIndex);
    frame.put("valid", tagCount > 0);

    // Pipeline information.
    frame.put("pID", (int) pipelineIndexEntry.getDouble(0));
    frame.put("pTYPE", settings.pipelineType);

    // Primary target information.
    frame.put("tx", primary != null ? primary.tx : 0.0);
    frame.put("ty", primary != null ? primary.ty : 0.0);
    frame.put("txnc", primary != null ? primary.tx : 0.0);
    frame.put("tync", primary != null ? primary.ty : 0.0);
    frame.put("ta", primary != null ? primary.ta : 0.0);
    frame.put("tdist", primary != null ? primary.distToCamera : 0.0);

    frame.put("tl", tlMs);
    frame.put("cl", clMs);

    /*
     * Pose arrays use:
     * [x, y, z, roll, pitch, yaw]
     *
     * Translation is in meters and rotation is in degrees.
     *
     * An empty frame publishes zero pose arrays so that an invalid
     * frame is not mistaken for a valid localization result.
     */
    double[] zeroPose = new double[6];

    double[] mt1Blue = tagCount > 0
        ? pose3dToArray(mt1Pose)
        : zeroPose;

    double[] mt1Red = tagCount > 0
        ? pose3dToArray(mt1PoseRed)
        : zeroPose;

    double[] mt2Blue = tagCount > 0
        ? pose3dToArray(mt2Pose)
        : zeroPose;

    double[] mt2Red = tagCount > 0
        ? pose3dToArray(mt2PoseRed)
        : zeroPose;

    frame.put("botpose", mt1Blue);
    frame.put("botpose_wpiblue", mt1Blue);
    frame.put("botpose_wpired", mt1Red);

    frame.put("botpose_orb", mt2Blue);
    frame.put("botpose_orb_wpiblue", mt2Blue);
    frame.put("botpose_orb_wpired", mt2Red);

    frame.put("botpose_tagcount", tagCount);
    frame.put("botpose_span", tagSpan);
    frame.put("botpose_avgdist", avgTagDist);
    frame.put("botpose_avgarea", avgTagArea);

    // Camera pose relative to the robot.
    frame.put(
        "camerapose_robotspace",
        pose3dToArray(
            new Pose3d(robotToCamera.getTranslation(), robotToCamera.getRotation())));

    /*
     * MegaTag2's camera-to-robot transform is included in the frame
     * for consumers that use the corresponding camera-space data.
     */
    frame.put("t6c_rs", pose3dToArray(
        new Pose3d(robotToCamera.getTranslation(), robotToCamera.getRotation())));

    // Fiducial observations.
    List<Map<String, Object>> fiducials = new ArrayList<>(visible.size());

    for (TagObservation obs : visible) {
      Map<String, Object> fiducial = new LinkedHashMap<>();

      fiducial.put("fID", obs.id);
      fiducial.put("fam", "36h11");
      fiducial.put("ta", obs.ta);

      fiducial.put("tx", obs.tx);
      fiducial.put("ty", obs.ty);

      fiducial.put("txp", obs.tx * obs.pixelsPerDegree);
      fiducial.put("typ", obs.ty * obs.pixelsPerDegree);

      fiducial.put("tx_nocross", obs.tx);
      fiducial.put("ty_nocross", obs.ty);

      fiducial.put("ambiguity", obs.ambiguity);

      // The tag is present in the simulated field layout.
      fiducial.put("fielded", 1);

      // Camera pose relative to the target.
      fiducial.put(
          "t6c_ts",
          pose3dToArray(cameraPose.relativeTo(obs.pose)));

      // Robot pose in field space.
      fiducial.put(
          "t6r_fs",
          pose3dToArray(robotPose));

      // Robot pose relative to the target.
      fiducial.put(
          "t6r_ts",
          pose3dToArray(robotPose.relativeTo(obs.pose)));

      // Target pose relative to the camera.
      fiducial.put(
          "t6t_cs",
          pose3dToArray(obs.pose.relativeTo(cameraPose)));

      // Target pose relative to the robot.
      fiducial.put(
          "t6t_rs",
          pose3dToArray(obs.pose.relativeTo(robotPose)));

      fiducials.add(fiducial);
    }

    frame.put("Fiducial", fiducials);

    // Timestamps.
    frame.put("ts_us", (long) (nowSeconds * 1_000_000.0));
    frame.put("ts_sys", (long) (nowSeconds * 1_000_000.0));

    /*
     * Publish the encoded frame. The frame index changes every update,
     * ensuring that consecutive frames have different payloads.
     */
    resultsPublisher.set(MessagePack.encode(frame));
  }

  private double[] botPoseArray(
      Pose3d pose,
      double latencyMs,
      int tagCount,
      double tagSpan,
      double avgTagDist,
      double avgTagArea,
      double[] rawFiducials) {

    if (tagCount == 0) {
      return new double[0];
    }

    double[] pose6 = pose3dToArray(pose);
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

    return new Pose3d(
        fieldLayout.getFieldLength() - bluePose.getX(),
        fieldLayout.getFieldWidth() - bluePose.getY(),
        bluePose.getZ(),
        new Rotation3d(
            rotation.getX(),
            rotation.getY(),
            rotation.getZ() + Math.PI));
  }

  private Pose3d addNoise(
      Pose3d pose,
      double translationStdDev,
      double rotationStdDevDegrees) {

    if (translationStdDev <= 0 && rotationStdDevDegrees <= 0) {
      return pose;
    }

    double x = pose.getX() + noise.nextGaussian() * translationStdDev;
    double y = pose.getY() + noise.nextGaussian() * translationStdDev;
    double z = pose.getZ() + noise.nextGaussian() * translationStdDev * 0.5;

    Rotation3d rotation = pose.getRotation();

    double yaw = rotation.getZ()
        + Math.toRadians(noise.nextGaussian() * rotationStdDevDegrees);

    return new Pose3d(
        x,
        y,
        z,
        new Rotation3d(rotation.getX(), rotation.getY(), yaw));
  }

  private Optional<TagObservation> project(
      Pose3d cameraPose,
      Pose3d robotPose,
      FieldTag tag) {

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

    if (Math.abs(yawDeg) > halfHFovDeg
        || Math.abs(pitchDeg) > halfVFovDeg) {
      return Optional.empty();
    }

    Translation3d tagNormal =
        new Translation3d(1, 0, 0).rotateBy(tagPose.getRotation());

    Translation3d tagToCam =
        cameraPose.getTranslation().minus(tagPose.getTranslation());

    double dot =
        tagNormal.getX() * tagToCam.getX()
            + tagNormal.getY() * tagToCam.getY()
            + tagNormal.getZ() * tagToCam.getZ();

    double incidenceDeg = Math.toDegrees(
        Math.acos(
            Math.clamp(
                dot / tagToCam.getNorm(),
                -1,
                1)));

    if (incidenceDeg > settings.maxTagIncidenceAngleDegrees) {
      return Optional.empty();
    }

    double fx = (settings.resolutionWidth / 2.0)
        / Math.tan(Math.toRadians(halfHFovDeg));

    double apparentWidthPx =
        fx * settings.tagSizeMeters
            * Math.cos(Math.toRadians(incidenceDeg)) / x;

    double areaPx = apparentWidthPx * apparentWidthPx;

    double ta = Math.clamp(
        areaPx
            / (settings.resolutionWidth * settings.resolutionHeight)
            * 100.0,
        0,
        100);

    double noisyYaw =
        yawDeg + noise.nextGaussian() * settings.angleNoiseStdDevDegrees;

    double noisyPitch =
        pitchDeg + noise.nextGaussian() * settings.angleNoiseStdDevDegrees;

    double distToRobot =
        robotPose.getTranslation().getDistance(tagPose.getTranslation());

    double ambiguity = Math.clamp(
        noise.nextDouble() * 0.05 + (incidenceDeg / 90.0) * 0.15,
        0,
        1);

    double pixelsPerDegree =
        (settings.resolutionWidth / 2.0) / halfHFovDeg;

    return Optional.of(
        new TagObservation(
            tag.getID(),
            tagPose,
            ta,
            noisyYaw,
            noisyPitch,
            distance,
            distToRobot,
            ambiguity,
            apparentWidthPx,
            pixelsPerDegree));
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

  /**
   * Small MessagePack encoder for the data types used by this simulator.
   *
   * Supports:
   * - Maps
   * - Arrays/lists
   * - Strings
   * - Integers and longs
   * - Doubles
   * - Booleans
   * - Null
   */
  private static class MessagePack {

    private static byte[] encode(Object value) {
      ByteArrayOutputStream output = new ByteArrayOutputStream();

      try {
        writeValue(output, value);
      } catch (IOException e) {
        throw new IllegalStateException("Failed to encode MessagePack frame", e);
      }

      return output.toByteArray();
    }

    private static void writeValue(
        ByteArrayOutputStream output,
        Object value) throws IOException {

      if (value == null) {
        output.write(0xc0);
        return;
      }

      if (value instanceof Boolean bool) {
        output.write(bool ? 0xc3 : 0xc2);
        return;
      }

      if (value instanceof Byte
          || value instanceof Short
          || value instanceof Integer
          || value instanceof Long) {
        writeInteger(output, ((Number) value).longValue());
        return;
      }

      if (value instanceof Number number) {
        writeDouble(output, number.doubleValue());
        return;
      }

      if (value instanceof String string) {
        writeString(output, string);
        return;
      }

      if (value instanceof Map<?, ?> map) {
        writeMapHeader(output, map.size());

        for (Map.Entry<?, ?> entry : map.entrySet()) {
          writeValue(output, entry.getKey());
          writeValue(output, entry.getValue());
        }

        return;
      }

      if (value instanceof List<?> list) {
        writeArrayHeader(output, list.size());

        for (Object item : list) {
          writeValue(output, item);
        }

        return;
      }

      if (value instanceof double[] array) {
        writeArrayHeader(output, array.length);

        for (double item : array) {
          writeDouble(output, item);
        }

        return;
      }

      if (value instanceof Object[] array) {
        writeArrayHeader(output, array.length);

        for (Object item : array) {
          writeValue(output, item);
        }

        return;
      }

      throw new IllegalArgumentException(
          "Unsupported MessagePack value type: " + value.getClass());
    }

    private static void writeMapHeader(
        ByteArrayOutputStream output,
        int size) {

      if (size <= 15) {
        output.write(0x80 | size);
      } else if (size <= 0xffff) {
        output.write(0xde);
        writeShort(output, size);
      } else {
        output.write(0xdf);
        writeInt(output, size);
      }
    }

    private static void writeArrayHeader(
        ByteArrayOutputStream output,
        int size) {

      if (size <= 15) {
        output.write(0x90 | size);
      } else if (size <= 0xffff) {
        output.write(0xdc);
        writeShort(output, size);
      } else {
        output.write(0xdd);
        writeInt(output, size);
      }
    }

    private static void writeString(
        ByteArrayOutputStream output,
        String value) {

      byte[] bytes = value.getBytes(StandardCharsets.UTF_8);
      int length = bytes.length;

      if (length <= 31) {
        output.write(0xa0 | length);
      } else if (length <= 0xff) {
        output.write(0xd9);
        output.write(length);
      } else if (length <= 0xffff) {
        output.write(0xda);
        writeShort(output, length);
      } else {
        output.write(0xdb);
        writeInt(output, length);
      }

      output.writeBytes(bytes);
    }

    private static void writeInteger(
        ByteArrayOutputStream output,
        long value) {

      if (value >= 0 && value <= 127) {
        output.write((int) value);
      } else if (value >= -32 && value < 0) {
        output.write((int) value & 0xff);
      } else if (value >= 0 && value <= 0xff) {
        output.write(0xcc);
        output.write((int) value);
      } else if (value >= 0 && value <= 0xffff) {
        output.write(0xcd);
        writeShort(output, (int) value);
      } else if (value >= Integer.MIN_VALUE
          && value <= Integer.MAX_VALUE) {
        output.write(0xd2);
        writeInt(output, (int) value);
      } else {
        output.write(0xd3);
        writeLong(output, value);
      }
    }

    private static void writeDouble(
        ByteArrayOutputStream output,
        double value) {

      output.write(0xcb);
      writeLong(output, Double.doubleToLongBits(value));
    }

    private static void writeShort(
        ByteArrayOutputStream output,
        int value) {

      output.write((value >>> 8) & 0xff);
      output.write(value & 0xff);
    }

    private static void writeInt(
        ByteArrayOutputStream output,
        int value) {

      output.write((value >>> 24) & 0xff);
      output.write((value >>> 16) & 0xff);
      output.write((value >>> 8) & 0xff);
      output.write(value & 0xff);
    }

    private static void writeLong(
        ByteArrayOutputStream output,
        long value) {

      output.write((int) (value >>> 56) & 0xff);
      output.write((int) (value >>> 48) & 0xff);
      output.write((int) (value >>> 40) & 0xff);
      output.write((int) (value >>> 32) & 0xff);
      output.write((int) (value >>> 24) & 0xff);
      output.write((int) (value >>> 16) & 0xff);
      output.write((int) (value >>> 8) & 0xff);
      output.write((int) value & 0xff);
    }
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

    TagObservation(
        int id,
        Pose3d pose,
        double ta,
        double tx,
        double ty,
        double distToCamera,
        double distToRobot,
        double ambiguity,
        double apparentWidthPixels,
        double pixelsPerDegree) {

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