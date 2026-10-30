// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.button.CommandXboxController;

import first.robot.generated.TunerConstants;
import first.robot.subsystems.swerve.CommandSwerveDrivetrain;
import first.robot.subsystems.swerve.DriveCommands;

public class RobotContainer {

  private double speedLimiter = 0.7;

  private final CommandSwerveDrivetrain drivetrain;

  private final CommandXboxController driverController = new CommandXboxController(0);
  //private final CommandXboxController operatorController = new CommandXboxController(1);


  public RobotContainer() {
    drivetrain = TunerConstants.createDrivetrain();
    configureBindings();
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                DriveCommands.joystickDriveRequest(
                    () -> -driverController.getLeftY() * speedLimiter,
                    () -> -driverController.getLeftX() * speedLimiter,
                    () -> -driverController.getRightX() * speedLimiter
                )
            )
        );
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
