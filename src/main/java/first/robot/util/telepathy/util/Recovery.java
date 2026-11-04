package first.robot.util.telepathy.util;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.BiFunction;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;

public class Recovery {
    private static HashMap<String, BiFunction<PathPlannerPath, PathPlannerTrajectoryState, Command>> recoveryCommands = new HashMap<>();
    private static String currentRecoveryMode = "No Recovery";
    
    public static final String NO_RECOVERY = "No Recovery";

    private Recovery() {}

    static {
        recoveryCommands.put(NO_RECOVERY, (path, state) -> Commands.none());
    }

    public static void registerRecoveryMode(String name, BiFunction<PathPlannerPath, PathPlannerTrajectoryState, Command> commandGenerator) {
        recoveryCommands.put(name, commandGenerator);
    }

    public static void setCurrentMode(String name) {
        if (recoveryCommands.containsKey(name)) {
            currentRecoveryMode = name;
        } else {
            throw new IllegalArgumentException("Recovery mode \"" + name + "\" is not registered.");
        }
    }

    public static String getCurrentMode() {
        return currentRecoveryMode;
    }

    public static BiFunction<PathPlannerPath, PathPlannerTrajectoryState, Command> getCommandBuilder(String recoveryType) {
        return recoveryCommands.getOrDefault(recoveryType, (path, state) -> Commands.none());
    }

    public static ArrayList<String> getAllRecoveryModes() {
        return new ArrayList<>(recoveryCommands.keySet());
    }


}
