// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.driverstation.MatchState;
import org.wpilib.framework.RobotBase;

import first.robot.Constants.ShooterConstants;
import first.robot.util.Elastic;
import first.robot.util.FieldUtil;

public class Robot extends LoggedRobot {
    private Command autonomousCommand;
    
    private final RobotContainer robotContainer;

    public Robot() {
        Logger.recordMetadata("Project Name", "Nixon"); 

        robotContainer = new RobotContainer();
        if (RobotBase.isReal()) {
            Logger.addDataReceiver(new WPILOGWriter());
            Logger.addDataReceiver(new NT4Publisher());
        } else {
            Logger.addDataReceiver(new NT4Publisher());
        }

        Logger.start();
    }
    
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override 
    public void driverStationConnected() {
        applyAllianceConfig();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        Elastic.selectTab("Auto");
        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null) {
        CommandScheduler.getInstance().schedule(autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {
        if (autonomousCommand != null) {
        autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopInit() {
        Elastic.selectTab("Teleop");
        applyAllianceConfig();
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void utilityInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void utilityPeriodic() {}

    @Override
    public void utilityExit() {}

    private void applyAllianceConfig() {
        var allianceOpt = MatchState.getAlliance();
        if (allianceOpt.isEmpty()) return;

        robotContainer.getDrivetrain().seedFieldCentric(
            FieldUtil.getZeroRotation()
        );
        
        robotContainer.getDrivetrain().getPigeon2().setYaw(FieldUtil.getZeroRotation().getDegrees());
        
        // *Clear caches first so getShootingSetpoint() recomputes with new alliance
        ShooterConstants.flipShootingSetpoints();
        
        Logger.recordOutput("Alliance", FieldUtil.alliance);
    }
}
