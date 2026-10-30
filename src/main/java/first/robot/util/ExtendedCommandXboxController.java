package first.robot.util;

import org.wpilib.driverstation.GenericHID.RumbleType;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.button.CommandXboxController;

public class ExtendedCommandXboxController extends CommandXboxController {
  public ExtendedCommandXboxController(int port) {
    super(port);
  }

  public Command rumbleFor(RumbleType type, double value, double seconds) {
    return Commands.sequence(
            Commands.runOnce(() -> getHID().setRumble(type, value)),
            Commands.waitSeconds(seconds),
            Commands.runOnce(() -> getHID().setRumble(type, 0)))
        .ignoringDisable(false);
  }
}