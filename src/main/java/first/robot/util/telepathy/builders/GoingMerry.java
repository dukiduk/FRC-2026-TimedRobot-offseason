package first.robot.util.telepathy.builders;

import static org.wpilib.units.Units.MetersPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.AutoBuilderException;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.units.measure.LinearVelocity;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.system.Filesystem;
import org.wpilib.tunable.Selectable;
import org.wpilib.command2.Command;
import org.wpilib.command2.Subsystem;
import org.wpilib.command2.Command.InterruptionBehavior;
import org.wpilib.command2.Commands;
import first.robot.util.telepathy.commands.ShinFollowPathCommand;
import first.robot.util.telepathy.commands.ShinPathfindingCommand;
import first.robot.util.telepathy.util.RecoveryType;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.*;
import java.util.stream.Collectors;
import java.util.stream.Stream;

// *GoingMerry is what builds the pathfinding commands, so its AutoBuilder Minus. Made to interact with ShinPathfindingCommand

public class GoingMerry {
    private static Globals globals = new Globals();

    /**
     * Configures the GoingMerry for using PathPlanner's built-in commands.
     *
     * @param poseSupplier a supplier for the robot's current pose
     * @param robotRelativeSpeedsSupplier a supplier for the robot's current robot relative chassis
     *     speeds
     * @param output Output function that accepts robot-relative ChassisVelocities and feedforwards for
     *     each drive motor. If using swerve, these feedforwards will be in FL, FR, BL, BR order. If
     *     using a differential drive, they will be in L, R order.
     *     <p>NOTE: These feedforwards are assuming unoptimized module states. When you optimize your
     *     module states, you will need to reverse the feedforwards for modules that have been flipped
     * @param controller Path following controller that will be used to follow paths
     * @param robotConfig The robot configuration
     * @param shouldFlipPath Supplier that determines if paths should be flipped to the other side of
     *     the field. This will maintain a global blue alliance origin.
     * @param driveRequirements the subsystem requirements for the robot's drive train
     */
    public static void configure(
        Supplier<Pose2d> poseSupplier,
        Supplier<ChassisVelocities> robotRelativeSpeedsSupplier,
        BiConsumer<ChassisVelocities, DriveFeedforwards> output,
        PathFollowingController controller,
        RobotConfig robotConfig,
        BooleanSupplier shouldFlipPath,
        Subsystem... driveRequirements) {
        if (globals.configured) {
        DriverStationErrors.reportError(
            "Going Merry has already been configured. This is likely in error.", true);
        }

        globals.pathFollowingCommandBuilder =
            (path) ->
                new ShinFollowPathCommand(
                    path,
                    poseSupplier,
                    robotRelativeSpeedsSupplier,
                    output,
                    controller,
                    robotConfig,
                    shouldFlipPath,
                    driveRequirements);
        globals.poseSupplier = poseSupplier;
        globals.robotRelativeSpeedsSupplier = robotRelativeSpeedsSupplier;
        globals.output = output;
        globals.controller = controller;
        globals.robotConfig = robotConfig;
        globals.shouldFlipPath = shouldFlipPath;
        globals.driveRequirements = driveRequirements.clone();
        globals.isHolonomic = robotConfig.isHolonomic;

        globals.pathfindToPoseCommandBuilder =
            (pose, constraints, goalEndVel) ->
                new ShinPathfindingCommand(
                    pose,
                    constraints,
                    goalEndVel,
                    poseSupplier,
                    robotRelativeSpeedsSupplier,
                    output,
                    controller,
                    robotConfig,
                    driveRequirements)
                .withInterruptBehavior(InterruptionBehavior.CANCEL_INCOMING);

        globals.pathfindToPosesCommandBuilder =
            (pose, stops, constraints, goalEndVel) ->
                new ShinPathfindingCommand(
                    pose,
                    stops,
                    constraints,
                    goalEndVel,
                    poseSupplier,
                    robotRelativeSpeedsSupplier,
                    output,
                    controller,
                    robotConfig,
                    driveRequirements)
                .withInterruptBehavior(InterruptionBehavior.CANCEL_INCOMING);


        globals.pathfindThenFollowPathCommandBuilder =
            (path, constraints) ->
                new ShinPathfindingCommand(
                    path,
                    constraints,
                    poseSupplier,
                    robotRelativeSpeedsSupplier,
                    output,
                    controller,
                    robotConfig,
                    shouldFlipPath,
                    driveRequirements)
                .withInterruptBehavior(InterruptionBehavior.CANCEL_INCOMING);


        // Holy that is a long name
        globals.pathfindThroughStopsThenFollowPathCommandBuilder =
            (path, stops, constraints) ->
                new ShinPathfindingCommand(
                    path,
                    stops,
                    constraints,
                    poseSupplier,
                    robotRelativeSpeedsSupplier,
                    output,
                    controller,
                    robotConfig,
                    shouldFlipPath,
                    driveRequirements)
                .withInterruptBehavior(InterruptionBehavior.CANCEL_INCOMING);
        globals.configured = true;
        globals.pathfindingConfigured = true;
    }

    /**
     * Configures the GoingMerry for using PathPlanner's built-in commands.
     *
     * @param poseSupplier a supplier for the robot's current pose
     * @param robotRelativeSpeedsSupplier a supplier for the robot's current robot relative chassis
     *     speeds
     * @param output Output function that accepts robot-relative ChassisVelocities.
     * @param controller Path following controller that will be used to follow paths
     * @param robotConfig The robot configuration
     * @param shouldFlipPath Supplier that determines if paths should be flipped to the other side of
     *     the field. This will maintain a global blue alliance origin.
     * @param driveRequirements the subsystem requirements for the robot's drive train
     */
    public static void configure(
        Supplier<Pose2d> poseSupplier,
        Supplier<ChassisVelocities> robotRelativeSpeedsSupplier,
        Consumer<ChassisVelocities> output,
        PathFollowingController controller,
        RobotConfig robotConfig,
        BooleanSupplier shouldFlipPath,
        Subsystem... driveRequirements) {
        configure(
            poseSupplier,
            robotRelativeSpeedsSupplier,
            (speeds, feedforwards) -> output.accept(speeds),
            controller,
            robotConfig,
            shouldFlipPath,
            driveRequirements);
    }

    //This constructor can configure both GoingMerry and AutoBuilder
    public static void configure(
        Supplier<Pose2d> poseSupplier,
        Consumer<Pose2d> resetPoseSupplier,
        Supplier<ChassisVelocities> robotRelativeSpeedsSupplier,
        BiConsumer<ChassisVelocities, DriveFeedforwards> output,
        PathFollowingController controller,
        RobotConfig robotConfig,
        BooleanSupplier shouldFlipPath,
        Subsystem... driveRequirements) {
        
        configure(
            poseSupplier,
            robotRelativeSpeedsSupplier,
            output,
            controller,
            robotConfig,
            shouldFlipPath,
            driveRequirements);

        if (!AutoBuilder.isConfigured()) {
        AutoBuilder.configure(
            poseSupplier,
            resetPoseSupplier,
            robotRelativeSpeedsSupplier,
            output,
            controller,
            robotConfig,
            shouldFlipPath,
            driveRequirements);
        }
        globals.resetPoseSupplier = resetPoseSupplier;
    }

    /**
     * Holder for all global variables directly referenced by GoingMerry.
     *
     * <p>This class exists to ensure that {@link #resetForTesting()} resets all static state in
     * GoingMerry.
     */
    private static class Globals {
        boolean configured = false;
        boolean pathfindingConfigured = false;

        // Configuration parameters
        Supplier<Pose2d> poseSupplier;
        Supplier<ChassisVelocities> robotRelativeSpeedsSupplier;
        BiConsumer<ChassisVelocities, DriveFeedforwards> output;
        PathFollowingController controller;
        RobotConfig robotConfig;
        BooleanSupplier shouldFlipPath;
        Subsystem[] driveRequirements;
        Consumer<Pose2d> resetPoseSupplier;

        boolean isHolonomic;

        // Existing command builders
        Function<PathPlannerPath, Command> pathFollowingCommandBuilder;

        TriFunction<Pose2d, PathConstraints, Double, Command>
            pathfindToPoseCommandBuilder;

        QuadFunction<Pose2d, List<Pose2d>, PathConstraints, Double, Command>
            pathfindToPosesCommandBuilder;

        BiFunction<PathPlannerPath, PathConstraints, Command>
            pathfindThenFollowPathCommandBuilder;

        TriFunction<PathPlannerPath, List<Pose2d>, PathConstraints, Command>
            pathfindThroughStopsThenFollowPathCommandBuilder;
    }

    /**
     * Returns whether the GoingMerry has been configured.
     *
     * @return true if the GoingMerry has been configured, false otherwise
     */
    public static boolean isConfigured() {
        return globals.configured;
    }

    /**
     * Returns whether the GoingMerry has been configured for pathfinding.
     *
     * @return true if the GoingMerry has been configured for pathfinding, false otherwise
     */
    public static boolean isPathfindingConfigured() {
        return globals.pathfindingConfigured;
    }

    /**
     * Resets {@code GoingMerry} static state to the values set at class initialization time.
     *
     * <p>This method should not be called during a competition.
     */
    public static void resetForTesting() {
        globals = new Globals();
    }

    /**
     * Get the current robot pose
     *
     * @return Current robot pose
     */
    public static Pose2d getCurrentPose() {
        return globals.poseSupplier.get();
    }

    public static ChassisVelocities getCurrentSpeeds() {
        return globals.robotRelativeSpeedsSupplier.get();
    }

    public static BiConsumer<ChassisVelocities, DriveFeedforwards> getOutput() {
        return globals.output;
    }

    public static PathFollowingController getController() {
        return globals.controller;
    }

    public static RobotConfig getRobotConfig() {
        return globals.robotConfig;
    }

    public static BooleanSupplier getShouldFlipPath() {
        return globals.shouldFlipPath;
    }

    public static Subsystem[] getDriveRequirements() {
        return globals.driveRequirements == null
            ? new Subsystem[0]
            : globals.driveRequirements.clone();
    }

    public static Consumer<Pose2d> getResetPoseSupplier() {
        return globals.resetPoseSupplier;
    }


    /**
     * Get if a path or field position should currently be flipped
     *
     * @return True if path/positions should be flipped
     */
    public static boolean shouldFlip() {
        return globals.shouldFlipPath.getAsBoolean();
    }

    /**
     * Builds a command to follow a path. PathPlannerLib commands will also trigger event markers
     * along the way.
     *
     * @param path the path to follow
     * @return a path following command with for the given path
     * @throws AutoBuilderException if the GoingMerry has not been configured
     */
    public static Command followPath(PathPlannerPath path) {
        if (!isConfigured()) {
        throw new AutoBuilderException(
            "Going Merry was used to build a path following command before being configured");
        }

        return globals.pathFollowingCommandBuilder.apply(path);
    }

    /**
     * Build a command to pathfind to a given pose. If not using a holonomic drivetrain, the pose
     * rotation and rotation delay distance will have no effect.
     *
     * @param pose The pose to pathfind to
     * @param constraints The constraints to use while pathfinding
     * @param goalEndVelocity The goal end velocity of the robot when reaching the target pose
     * @return A command to pathfind to a given pose
     */
    public static Command pathfindToPose(
        Pose2d pose, PathConstraints constraints, double goalEndVelocity) {
        if (!isPathfindingConfigured()) {
        throw new AutoBuilderException(
            "Going Merry was used to build a pathfinding command before being configured");
        }

        return globals.pathfindToPoseCommandBuilder.apply(pose, constraints, goalEndVelocity);
    }

    /**
     * Build a command to pathfind to a given pose. If not using a holonomic drivetrain, the pose
     * rotation and rotation delay distance will have no effect.
     *
     * @param pose The pose to pathfind to
     * @param stops The stops it must pass through before getting to the pose
     * @param constraints The constraints to use while pathfinding
     * @param goalEndVelocity The goal end velocity of the robot when reaching the target pose
     * @return A command to pathfind to a given pose
     */
    public static Command pathfindToPose(
        Pose2d pose, List<Pose2d> stops, PathConstraints constraints, double goalEndVelocity) {
        if (!isPathfindingConfigured()) {
        throw new AutoBuilderException(
            "Going Merry was used to build a pathfinding command before being configured");
        }

        return globals.pathfindToPosesCommandBuilder.apply(pose, stops, constraints, goalEndVelocity);
    }

    /**
     * Build a command to pathfind to a given pose. If not using a holonomic drivetrain, the pose
     * rotation and rotation delay distance will have no effect.
     *
     * @param pose The pose to pathfind to
     * @param constraints The constraints to use while pathfinding
     * @param goalEndVelocity The goal end velocity of the robot when reaching the target pose
     * @return A command to pathfind to a given pose
     */
    public static Command pathfindToPose(
        Pose2d pose, PathConstraints constraints, LinearVelocity goalEndVelocity) {
        return pathfindToPose(pose, constraints, goalEndVelocity.in(MetersPerSecond));
    }

    /**
     * Build a command to pathfind to a given pose. If not using a holonomic drivetrain, the pose
     * rotation and rotation delay distance will have no effect.
     *
     * @param pose The pose to pathfind to
     * @param stops The stops it must pass through before getting to the pose
     * @param constraints The constraints to use while pathfinding
     * @param goalEndVelocity The goal end velocity of the robot when reaching the target pose
     * @return A command to pathfind to a given pose
     */
    public static Command pathfindToPose(
        Pose2d pose, List<Pose2d> stops, PathConstraints constraints, LinearVelocity goalEndVelocity) {
        return pathfindToPose(pose, stops, constraints, goalEndVelocity.in(MetersPerSecond));
    }

    /**
     * Build a command to pathfind to a given pose. If not using a holonomic drivetrain, the pose
     * rotation will have no effect.
     *
     * @param pose The pose to pathfind to
     * @param constraints The constraints to use while pathfinding
     * @return A command to pathfind to a given pose
     */
    public static Command pathfindToPose(Pose2d pose, PathConstraints constraints) {
        return pathfindToPose(pose, constraints, 0);
    }

    /**
     * Build a command to pathfind to a given pose. If not using a holonomic drivetrain, the pose
     * rotation will have no effect.
     *
     * @param pose The pose to pathfind to
     * @param stops The stops it must pass through before getting to the pose
     * @param constraints The constraints to use while pathfinding
     * @return A command to pathfind to a given pose
     */
    public static Command pathfindToPose(Pose2d pose, List<Pose2d> stops, PathConstraints constraints) {
        return pathfindToPose(pose, stops, constraints, 0);
    }


    /**
     * Build a command to pathfind to a given path, then follow that path. If not using a holonomic
     * drivetrain, the pose rotation delay distance will have no effect.
     *
     * @param goalPath The path to pathfind to, then follow
     * @param pathfindingConstraints The constraints to use while pathfinding
     * @return A command to pathfind to a given path, then follow the path
     */
    public static Command pathfindThenFollowPath(
        PathPlannerPath goalPath, PathConstraints pathfindingConstraints) {
        if (!isPathfindingConfigured()) {
        throw new AutoBuilderException(
            "Going Merry was used to build a pathfinding command before being configured");
        }

        return globals.pathfindThenFollowPathCommandBuilder.apply(goalPath, pathfindingConstraints);
    }

    /**
     * Build a command to pathfind to a given path, then follow that path. If not using a holonomic
     * drivetrain, the pose rotation delay distance will have no effect.
     *
     * @param goalPath The path to pathfind to, then follow
     * @param stops The stops it must pass through before getting to the path
     * @param pathfindingConstraints The constraints to use while pathfinding
     * @return A command to pathfind to a given path, then follow the path
     */
    public static Command pathfindThenFollowPath(
        PathPlannerPath goalPath, List<Pose2d> stops, PathConstraints pathfindingConstraints) {
        if (!isPathfindingConfigured()) {
        throw new AutoBuilderException(
            "Going Merry was used to build a pathfinding command before being configured");
        }

        return globals.pathfindThroughStopsThenFollowPathCommandBuilder.apply(goalPath, stops, pathfindingConstraints);
    }

    /**
     * Submits a {@link PathRequest} to build the corresponding pathfinding or path-following command.
     *
     * @param request The path request details containing target pose/path, stops, constraints, etc.
     * @param defaultConstraints The fallback path constraints to use if none were specified in the request.
     * @return A command configured to execute the requested path operation.
     */
    public static Command buildRequest(PathRequest request, PathConstraints defaultConstraints) {
        if (!isPathfindingConfigured()) {
        throw new AutoBuilderException(
            "Going Merry was used to build a pathfinding command before being configured");
        }

        // Use requested constraints if present; otherwise fall back to default
        PathConstraints constraints = Optional.of(request.getConstraints()).orElse(defaultConstraints);

        if (request.isPathFindThenFollowPath()) {
        return pathfindThenFollowPath(
            request.getTargetPath(),
            request.getStops(),
            constraints);
        } else {
        return pathfindToPose(
            request.getTargetPose(),
            request.getStops(),
            constraints,
            request.getGoalEndVel());
        }
    }

    /**
     * Create and populate a sendable chooser with all PathPlannerAutos in the project. The default
     * option will be Commands.none()
     *
     * @return Selectable populated with all autos
     */
    public static Selectable<Command> buildAutoChooser() {
        return buildAutoChooser("");
    }

    /**
     * Create and populate a sendable chooser with all PathPlannerAutos in the project
     *
     * @param defaultAutoName The name of the auto that should be the default option. If this is an
     *     empty string, or if an auto with the given name does not exist, the default option will be
     *     Commands.none()
     * @return Selectable populated with all autos
     */
    public static Selectable<Command> buildAutoChooser(String defaultAutoName) {
        return buildAutoChooserWithOptionsModifier(defaultAutoName, (stream) -> stream);
    }

    /**
     * Create and populate a sendable chooser with all PathPlannerAutos in the project. The default
     * option will be Commands.none()
     *
     * @param optionsModifier A lambda function that can be used to modify the options before they go
     *     into the AutoChooser
     * @return Selectable populated with all autos
     */
    public static Selectable<Command> buildAutoChooserWithOptionsModifier(
        Function<Stream<PathPlannerAuto>, Stream<PathPlannerAuto>> optionsModifier) {
        return buildAutoChooserWithOptionsModifier("", optionsModifier);
    }

    /**
     * Create and populate a sendable chooser with all PathPlannerAutos in the project
     *
     * @param defaultAutoName The name of the auto that should be the default option. If this is an
     *     empty string, or if an auto with the given name does not exist, the default option will be
     *     Commands.none()
     * @param optionsModifier A lambda function that can be used to modify the options before they go
     *     into the AutoChooser
     * @return Selectable populated with all autos
     */
    public static Selectable<Command> buildAutoChooserWithOptionsModifier(
        String defaultAutoName,
        Function<Stream<PathPlannerAuto>, Stream<PathPlannerAuto>> optionsModifier) {
        if (!AutoBuilder.isConfigured()) {
            throw new RuntimeException(
                "AutoBuilder was not configured before attempting to build an auto chooser");
        }

        Selectable<Command> chooser = new Selectable<>();
        List<String> autoNames = getAllAutoNames();

        PathPlannerAuto defaultOption = null;
        List<PathPlannerAuto> options = new ArrayList<>();

        for (String autoName : autoNames) {
            PathPlannerAuto auto = new PathPlannerAuto(autoName);

            if (!defaultAutoName.isEmpty() && defaultAutoName.equals(autoName)) {
                defaultOption = auto;
            } else {
                options.add(auto);
            }
        }

        chooser.add("None", Commands.none());
        if (defaultOption == null) {
            chooser.setDefault("None");
        } else {
            chooser.setDefault(defaultAutoName);
        }

        optionsModifier
            .apply(options.stream())
            .forEach(auto -> chooser.add(auto.getName(), auto));

        return chooser;
    }

    /**
     * Get a list of all auto names in the project
     *
     * @return List of all auto names
     */
    public static List<String> getAllAutoNames() {
        File[] autoFiles = new File(Filesystem.getDeployDirectory(), "pathplanner/autos").listFiles();

        if (autoFiles == null) {
        return new ArrayList<>();
        }

        return Stream.of(autoFiles)
            .filter(file -> !file.isDirectory())
            .map(File::getName)
            .filter(name -> name.endsWith(".auto"))
            .map(name -> name.substring(0, name.lastIndexOf(".")))
            .collect(Collectors.toList());
    }

    /**
     * Builds an auto command for the given auto name.
     *
     * @param autoName the name of the auto to build
     * @return an auto command for the given auto name
     */
    public static Command buildAuto(String autoName) {
        return new PathPlannerAuto(autoName);
    }
    
    /**
     * Get if GoingMerry was configured for a holonomic drive train
     *
     * @return True if holonomic
     */
    public static boolean isHolonomic() {
        if (!GoingMerry.isConfigured()) {
        throw new RuntimeException("GoingMerry was not configured before use");
        }

        return globals.isHolonomic;
    }

    public static Selectable<RecoveryType> buildAutoRecoveryChooser() {
        return buildAutoRecoveryChooser("NO_RECOVERY");
    }

    public static Selectable<RecoveryType> buildAutoRecoveryChooser(RecoveryType defaultType) {
        return buildAutoRecoveryChooser(defaultType.toString());
    }

    public static Selectable<RecoveryType> buildAutoRecoveryChooser(String defaultType) {
        Selectable<RecoveryType> chooser = new Selectable<>();
        for (RecoveryType type : RecoveryType.values()) {
            chooser.add(type.toString(), type);
        }
        chooser.setDefault(defaultType);
        chooser.onChange((type) -> RecoveryType.setCurrentMode(type));

        return chooser;
    }

    /** Functional interface for a function that takes 3 inputs */
    @FunctionalInterface
    public interface TriFunction<In1, In2, In3, Out> {
        /**
         * Apply the inputs to this function
         *
         * @param in1 Input 1
         * @param in2 Input 2
         * @param in3 Input 3
         * @return Output
         */
        Out apply(In1 in1, In2 in2, In3 in3);
    }

    /** Functional interface for a function that takes 4 inputs */
    @FunctionalInterface
    public interface QuadFunction<In1, In2, In3, In4, Out> {
        /**
         * Apply the inputs to this function
         *
         * @param in1 Input 1
         * @param in2 Input 2
         * @param in3 Input 3
         * @param in4 Input 4
         * @return Output
         */
        Out apply(In1 in1, In2 in2, In3 in3, In4 in4);
    }
}


/*

 .  ..   ....... . ..... .......  .   .  ...     ...      ........   .....  .   .....      ......     .....    .. ..      .....      ......     ......   .
                        .. .                           .                 .                                    .                    .  .                 . 
   . ..................... ........   ........    .......   .......  ........  ..          .        ..      .  .     ..             .. .      ..     ..  .
 .  ...  .    .    ...  .. ..       .           .....    ..        .             .                           .         .                               .  
    . ...  .        ... .           .         . .       . .       ...                         .         .    .                          .         .    .  
    .....       . . . ......                             ..              ................ . ..                              .. .      ..                  
     ....   ..   .. ....   ..  .. ..                ..        ..        ......@+.:.....%.                                                                 
  .          .       ..                                     .         . ...%:...........=.. .    . .. .. ........ ..           .        .. .  ..  .     . 
.... .........   .................  .......... ........ .................@:..............+.......  . @.:....::..+@.....    .  ...   .. ....   ........... 
        .  ...     ..                    ...       .                  ..%:................:%...........%:-%.....::-..          .      .             .    .
       ..         ...   ..           ..     ..     .  ..          .   .@@:--..+........:.....%....=:......@.*......:.. .    ..         ...       ...      
... ................................................... ......... ....@................%......%..@..........:......:*. ............................... ...
.. ..   .. .     .....      .  .        ..      .    .        .    ..:....:.@..................+:....................@...      ..  ..        . .... .   . 
    ...  .    .......    ...   .         ...       ...  ..   ...  ..-@-..............:.......:@.:....@...............=.  ..... ..  ......... ...    . .  .
 . ..  .....     ....      .... ........   ............  .   .. ...%................:.......-.........@......@.....:@. .  .......  ....      .  ...       
            ..     ..   .   .     ..            .       .          *:..............@.......@:........@.......+.....:.    ..                   ...       ..
     .         .  .     .     .      .   .      .....            .. #..........:.+......:.......%%=@.......:@.%....:.                  .         .      . 
 ............................     .............. .:...  ...        . .-=.:....%#..@......=................:=......=..   ..  .....     .....      ....     
.                                           .....@=:.                  .  ....    .-@@@::.@..............-........@ ...                                   
                                 .          ....*==+..                     .      .@.......::...........@.........:@ ... .                 .              
..  .. .......... ..............................=+==@.................. ...........-..........-@::.:*@:...........:.@ ......  ........ ......... .........
       .    ..     ..  .                  ..  .=====+=..                          .+..............=.@................-@:..   . .       .            .    .
  ..    .        .... ....   .....    .   ...========+.. .      ..                .*...............:=....@::.:#::.....::#.....         ...        ..      
............................. .@............*=========+.............................=...........#....*..*:......:=.......*@@:.............................
.. ..   ......................:=+      ....@======@+====...  ..         .       .. .@.........::.....+#:.........%........#..:@.   .. ...... . .......   .
    .......       .          .@++...  ...-==+==@==%+==+=+.........  .. . .  .   .....:......=@.......:............:...........--.  .....    .         .   
   .   .. . ..   ..   ..   ...=+=@......+==+=++++=%==*====...@.@.. ...   ...   ..  ..%............:=..................%..........   ....        ..  .   ..
 .     ..  ...             ..@+=++.....====+=+#+++===+#=+=+..=#%=....       ..     ...............%...................::.......*..             .          
                   .       ..=*===+. -======+@===@=====%===+%=+%== ..              .*............#...................@.........%.     ..         .        
....... .  ...... . ........@=@====*@========++=%==========%==+#==+..       ..... ...%:............................:............@..... .. .      ....     
                          .==+==+=++========%==@===+======#====+===*..              ..:.........-...............:..:..............:%...                  .
                       ....=======#=++@====++=@====+===++@====@+=@==+.  .. ...  ......#.........@@-@-:...........:...................:..   . ..        .. 
..  .. .......... ........++=+==+#+=++#+===+===============@=@==+@===.. ..@.....=: .. :@.............:......:%.:........%.:..........@.......... .........
       .   ...     .. ...#===@=+=*=@======+@+%=====+======%%+++==@===..%....-.....@...@.::@......:..........::::*.....@:.............:=..        .. .    .
        .          .  ..@===+@====++==+===#+@======*======*=%=%=====@.:.....:......@#-....+@:.::.%...........@..:.@.@.:..................         .       
. ......................==+==*=========%=@=++=*====%=======#====*=*....=...........:*::...:::.%-.:==........@.....-+..................:.... ..............
          . ....... ...@====+#==%+======@==@+%=+===@+=====@====**%.....@....%.....=....%.........@.:.=@..:+.....%@...............@.......#. .... .  ..   .
       ..  . ..=... . .+=====#+@+++=%===+%++=#==*==#=====@====+++:.....::.........:.....@:.........-*......::......@............=:@-.....-...         .   
       ..   ...@=@....#=+====%=+%===%===+=%=@=+==*=====+@===+===+........*:@.....:...................-.:.:.=.@:....:%:.......:@..........@#@......        
.  .   ..     .+=+=...-+==+==@%=@===%======@+=====@+===@==+=@===@-...........:.#..........@...........#-..@..........@@:.....-..........:@####..          
    .. ..    ...#+==*..+=+@==+======*=====+=====+=+==+*=+=======@=@.:.......:@@...........=...........:.%.@...........%:...:.:..........*.*###- .         
 .  .....  .....@=++=++%==@@=*++-==+*+==+++========@%======%====%==+%....@-:..:...........:..............@-@..............%::.:...........####@ . ...     
  ...   ....  . =++#===+++@=*@==%+==@===+++=+=====+====#+=======@===#..@%=::...:..........................:-:%...:.:....-:.:..............#####...        
               ..@+=%=====@===@++++=*===*===%======+@=====%=====%===::%::.::::#.:@..........................@...........:...:.............######%.. ..    
. ............. .@++=%+===%==+++**++===@+===%=+====+#=%==+=====+====@:.:::::::::.+-:@.%@.:................:.::@.........................%%########@.......
 ..        ...  ..===+*==+%======++=+%#==+==*=%+======*=+===========..:+::::::::::.@...::.@................:::.-.....................###############@.. . 
      . .  .   ...@+======*=====+=+=@==+===++++=====+===++=====@===@:::::::::::.@::*.........@:....:@::####:.#..#@................%##################@....
     ..          ..#=+=+=#==+========*+====+*@+======@=%==+===*+==++%:::::::::::::%........:...........-*####:=...#*........:..@######################%.. 
       ..      . ...%+======++=======%=====+#========+%===+==+====+@:::::::::::::::%:....................#@###*=.:..#%###%@+@##########################.. 
       ..  ...  .....%==+=++=+=+==+=================+@+====++*==+#.::::::::::::::::::.....:...............@###########################################@.  
       ..   ..   ......@++++===#==*=+=%+===#===+===@===+========+..%::::::::::::::::%..::@................@###########################################....
.  ..  ..    ..  ........@+*=+=++%=#+++#=+%======+=+@=.:...*=@%:@:::::::::::::::::::#:::@.................:#########################################%.. ..
    .  ..        ...%+=++++%=@=+====+@==%===========@:.....%+@.@#:::@:::::::::::::::::.%:..................@########################################..    
 . ...................@=====+#=@===+=+=%===+======+=@......-@.%.:#:..::::::::::::::::%.@...................:@####################################*.@..    
.   ....................@==*+==#=%++======*==+====+=@.........:.::*::@:::::::::::::::::#...................:.:@################################%..:-......
        .     .     ... ...*+=@+=++=+*===*+@#.%=====:.%...#=*...-@:::::::::::::::::::::@:.....................::############################@:....@ .     
. . ...............#@@@%==+..@==#==@:.:.::.....*====....:==*=:.+:::::.*.:::::::::::::::*#:......................:::.%################%@=........:@........
 .   ...@====================@..#=++@..........:%=+=..:*==.....*::::::.@::::::::::::::::%..*.....................:*.....=#+=...................%. .       
   .  ..+======================#*...@@:==@...@*.:++@.@=*+-@....+:::::::#%:::::::::::::::::+:.:=@:.........:.::....#:*@=..:..............:..:@....      ...
..    ..@=-===============%@+%%====#@.:%::@=+=....=@#:.@.#:....:::::.:@####-:.:::.:::::::::=..::#@-:.:-#%%*...:......:.:@:#:.....::@.%=...... . .. .. .   
       .... %@========================@::-*:@.@*#=@.:..@......-.@::::.@######:..:=%::::::::::.::::.@######@.............@@:..@.:.*+.....         .        
       .  %%%%%%%%%@===+====@========@-:.:::@.#.....:.....:..%..::::::@######@.:::::::::::::.::::.@############%%@@##%.... ...... .   ..   .    .         
    ....@%%%%%%%%@@@#=-+@#========-=====@::::::#.:@.@.@:-..#::.@...@::+#######@::.::::::..:@:::::#####################@...   . .       .  .    ...        
.     .@===================================%=:*:::#@%:..:.@:*@*:...:.+@@###########@%=@@###############################..         ..      . ..            
    ....@=============================+=%:===@##::@#@:..::#....*........@##############################################=.    .         .                  
 .......+=============#@...... ..%#.....@...%####%@###.::.:@:...@...+......@##########################################%. .  ...       ....        ...     
. ......@=========@-.......   ..@..#....%.@@.@##########:@.@....@@=::......:..:@######################################-...............  ...  ...  ...  ...
     ...@=====#%.......   .... ..%..-....+*@.=+####@####=::.......=................:=@@@@@@@@@%%%###@################@........... ........................
    . .%==%%....  ...     .     .@...:..%:::.:@####@####@:%-......:.:..#@:......:+%................+:..:.......-@@###=.. .    ...       .           .     
....  @@:............ .. ........=........=*..%###@@####@#@===:.....:.......:............:......@::..:# . .. ... ...@ ............. .... .................
     ....      ..         ...   . ..@....@..-.:@...@####@...@===##%@=--%@...:-#+:-:......@:-@%@.....*=..     ..    ..        .                         .. 
..  .                  %........ #...:@:..%... ..  .@###@.....@===@=.%@@@:@=::........-..:=:#@:.:#@==@..  ..        ..     .   ..    ...        .  ..     
       ..        .. . ...=@-. ..@===...=.....   .....=@@..........@@===*:.:.-...:..::.....@.....@==%=@.           .         .....     ..   ..             
       ..  ..    ..   ...:======*====@..... ....@....**@@@@@@....@@@==@====@...........:....:%+=#===@@..    .    .  .      .  ...     ..   .    .     .   
    .. ..   ..   .... .....%=======*====@*=.....#@@@@@+.....+%@@@+...%.@@========@+#@@@@@@@=====@=====..          ...       .. .      ..  .   ....        
.       .                 ..@==-@====%=#.......#@........@............=...#*====*****#*#%%%@=======@-.@..    ..                                           
 .  .. ..  .  ......    ......#===*==@=%@@=..............@.-@#................@*=@@===========%@+=#===@:                    ..         .                  
 .. .....  ...   ..  ...   .....%@.........................-@=-...@........=......-%@@@@%@=======++...%....       ..     .  ...       ..          ...     
...  ...    .......   .. ......:................................-%..........@.............-............@. .  .... ...  ........  .....  ...  ...  ...  ...
  .  . .......................-....................................%+......:.-%....=.......*...%.......=......... . ........ .... .... ........    .......
           ....    .....   ..:.......................................@@.......=:...@.................... .                     .                          
      ..... ..     ..  .   .#....................................:......=...@..:.@.=........:...............    ... .          ....              .....    
      .    .  ...      . @.........................................@=.....:..-..#:.@..................@..    ..                                        .  
     ..  ...       ..  @-..............................................@....:...@.+@..................+.  ..        ..   . .    .                   .     
       ..    ..  .. .@:...................................................*-..@.@.-..................%.                        ..     ..         .        
.      ..  ...    ..#.......................................................:-..*.*...@............@:...   ..       .          ..      .   .         ..   
   .. ..     .   .+.........................................................#@.@.+...............+......          .   .     .. .      ..  .    ...   .. . 
        .       .+.................................................#@.:....#@...=....:.........@......                             ..                     
    ... .   ... .@......................................................=:  .##....@:...............=.. .                   ...   .    .          .       
.    .  .  .   ..+.........................................................@..#..@.................@=.   ..       .                               .       
...   ..... .. ..=...................................................#. ...*...#.....:..............+.  ...                 ...   .... .... ....   .. ....
.  ..............+.................................................#%. .. .@....................#..@..   ....                .... .... .........   .......
     ............@......................................=+%@@*:..#..@.  ..@....@...............@..*.. ....                  ...   .......  ...   ..  .....
               ..@..................................................=.  .@.*....:.............@..#@..  ...                  ..  ...    ..  ...   ..     ..
    .....  .  ...@............................................@.....*  .*..%.....%..............@.@..  ..                  ...  ..         .     ... ..   
   .  .  .. ..   .@..........................................@.@:...@ ..@..@..................@...@.... ...                 ..... .     .... ..  .....   .
. .  ......   . ....@......................................=..@.........@..@........:.......@.....-.....                    .. ..     ..  .     ..  .     
.. ..  .....  .  .. ..@...................................@..#.....%....@.............=...%.......*                   .       .. .                 ..     
.. .    .  ........  ..@@...............................#...@..:@+*.+...@...%....................@.......            ..   ...... . ......  .  .......   ..
  .... ...  .......  ...@.+..........................%@..#.@+@%%%@*.....%....:............@.....@.    ..              .     ..  ..   ...  ...   ..   .    
........ .... .......... ...@................@@@:...@..-...@%%%%%=-.%*..=..@...@.............=-*.........              .... ......... ....................
.  ...  ..      . .  . .  @..@...................@..*:....@%%%%%@=.%.*..........-..........%...@ ..  .                          .              .        ..
... ..  .      ..    ... ..*..-...................+......%%%%%%%=%.@..@.....@............@....@..........        .  ............. .........  .............
......  ............... .....@...................@.....+%%%%%%%@@.....%......@........+....@.:. ........        .. .... .........  .... .... . ....... ...
.   .   .. .. ..     .  .   .... .@.....#@=.....@.....=..%%%%%@+.. ...@.......-.....@...@:..@                  ..   .    .              .  .              
                       ..     . .  .*@@%.......@....@.   .@%%%:...  .@=*....+%*@@@....@...=@..                 .   .. ...                       ..        
..  .. .  ... .....    ..   ....  .. .... .......%--..  ...=@@..  ..=======@........@:...=.....  .    ..          .   ..  .  . ...         .  .  ..  .... 
      ..     . ..  . ...    ..  ..        ..  ......    ......     .+=@====@.....:@.....@..  ..  .              ..           .      .   . ..             .
.  .  ...      .. ... .   . ....      .    .  .         .... .    .@==++===@. -@.. ......    .  ..   ...      ..  ...       .....  .  ...    .  ...       
 . ................  . .  . .   .       .   .  .. ....      ..... @%%%@@%@...  ....   ..          ..          .       .       ..               .   ..     
..      ...   ..    . .   . .  ...   ..     .. .. .       .  . ..@%%%%%@.  .         ..  ..      ... .        ..     ..      .     .. .. ...    .         
  ..... ..... ....      ..... ....        ..     .. ..    . ....@%%%%%#.... . ....                           .     .    . .  .        ..   ..            .
 .. ..... ................ ..        ...  ..   ..   ..         *%%%%@%..       ..   .    ..  ..    .... ... .  ..    ..  ..       ....  ..   .   ..      .
.   .                . .  ... ...                           . .%%%@:..           .      ..           ..          ....       .        .. .   . ..   ..     
.       .  ..   ....          ..     ..                      .@@+..   .                       ....        .    .          ..  . .  . .  .         ..      
.         ...      .                        ...  .    . ....  ..     ..            ....  . .                  ..    ..       .     ..     .  .   ....   ..


*/
