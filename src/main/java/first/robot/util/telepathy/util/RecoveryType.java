package first.robot.util.telepathy.util;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import first.robot.util.telepathy.builders.GoingMerry;
import first.robot.util.telepathy.commands.PPRecoveryCommand;

import java.util.function.BiFunction;

//* Enum determining the type of path recovery followed in ShinFollowPathCommand
public enum RecoveryType {

    //No extra recovery mode, just follows its path like normal
    NO_RECOVERY((path, state) -> Commands.none()),

    //Uses a modified drive-to-point command for recovery. Reliable but can get caught in narrow areas or sharp turns
    DRIVE_TO_POINT_RECOVERY((path, state) -> PPRecoveryCommand.generateCommand(path, state)),

    //PATHFINDING_RECOVERY: Pathfinds to the desired state. Most resource heavy, but guarantees robot safety (hopefully)
    PATHFINDING_RECOVERY((path, state) -> GoingMerry.pathfindToPose(state.pose, path.getGlobalConstraints(), state.linearVelocity));

    public final BiFunction<PathPlannerPath, PathPlannerTrajectoryState, Command> recoveryCommandBuilder;

    private RecoveryType(BiFunction<PathPlannerPath, PathPlannerTrajectoryState, Command> recoveryCommandBuilder) {
        this.recoveryCommandBuilder = recoveryCommandBuilder;
    }

    private static RecoveryType current = NO_RECOVERY;

    public static RecoveryType getCurrentMode() {
        return current;
    }

    public static void setCurrentMode(RecoveryType type) {
        current = type;
    }
}