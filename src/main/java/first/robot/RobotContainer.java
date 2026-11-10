// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.button.CommandXboxController;

import first.robot.Constants.*;
import first.robot.generated.TunerConstants;

import first.robot.subsystems.intake.Intake;
import first.robot.subsystems.intake.IntakeIOReal;
import first.robot.subsystems.intake.IntakeIOSim;

import first.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class RobotContainer {
    private double speedLimiter = 0.7;

    private final CommandSwerveDrivetrain drivetrain;
    private final Intake intake;

    private final CommandXboxController driverController = new CommandXboxController(0);
    @SuppressWarnings("unused")
    private final CommandXboxController operatorController = new CommandXboxController(1);


    public RobotContainer() {
        drivetrain = TunerConstants.createDrivetrain();
        intake = new Intake(Constants.kMode == Mode.REAL ? new IntakeIOReal() : new IntakeIOSim());
        configureBindings();
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.joystickDrive(
                () -> -driverController.getLeftY() * speedLimiter,
                () -> -driverController.getLeftX() * speedLimiter,
                () -> -driverController.getRightX() * speedLimiter
            )
        );

        driverController.leftTrigger().onTrue(
            new InstantCommand(
                () -> intake.setSetpoint(0),
                intake
            )
        );

        driverController.leftBumper().onTrue(
            new InstantCommand(
                () -> intake.setSetpoint(1),
                intake
            )
        );
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

    public CommandSwerveDrivetrain getDrivetrain() {
        return drivetrain;
    }
}
