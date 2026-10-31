// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import static org.wpilib.units.Units.*;

import java.util.Arrays;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.ctre.phoenix6.swerve.SwerveRequest;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.util.Units;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearVelocity;
import org.wpilib.units.measure.Time;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.framework.RobotBase;

import first.robot.generated.TunerConstants;
import first.robot.util.FieldUtil;

// The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants.
// This class should not be used for any other purpose. All constants should be declared globally (i.e. public static).
// It is advised to statically import this class (or one of its inner classes) wherever the constants are needed, to reduce verbosity.

public final class Constants {

    // *Enum to switch b/w real, sim, and replay
    public static final Mode kCurrentMode = RobotBase.isReal() ? Mode.REAL : Mode.SIM;
    public static enum Mode {
        REAL,
        SIM,
        REPLAY
    }

    // *Constants for the drivetrain
    public static class DriveConstants {

        public static final double kMaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
        public static final double kMaxAngularRate = RotationsPerSecond.of(.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

        public static final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
            .withDeadband(kMaxSpeed * 0.1).withRotationalDeadband(kMaxAngularRate * 0.1) // Add a 2% deadband
            .withDriveRequestType(DriveRequestType.Velocity) // Use open-loop control for drive motors
            .withDesaturateWheelVelocities(true); // Desaturate wheel speeds to avoid exceeding max speed

        public static final SwerveRequest.SwerveDriveBrake xBrake = new SwerveRequest.SwerveDriveBrake();
        public static final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
        public static final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.Velocity);

        public static final PPHolonomicDriveController pathController =
            new PPHolonomicDriveController(
                new PIDConstants(7.5, 0, 0), // Lightened from 8.0 for sharp turns
                new PIDConstants(4, 0, 0)
            );

        // !High ceiling, calculated from claude given robot config. May need to be tuned on real robot.
        public static final double kPathMaxLinearAcceleration = 5;
        public static final double kPathMaxAngularAcceleration = 5;

        // Copied from 4915 Spartronics. Their Post Path Precise Alignment is far better than ours
        public static final Rotation2d kRotationTolerance = Rotation2d.fromDegrees(1.0);
        public static final Distance kPositionTolerance = Centimeter.of(1.0);
        public static final LinearVelocity kSpeedTolerance = MetersPerSecond.of(0.3);

        public static final Time kEndTriggerDebounce = Seconds.of(0.04);

        public static final String choreoKey = "choreo/";

    }

    // *Constants for vision processing and AprilTag field layout
    public static class VisionConstants{
        public static final double kHubAlignToleranceDeg = 1.0;

        public static double getDirectionFlipper() {
            if (MatchState.getAlliance().isEmpty()) {
                return 0;
            }

            return MatchState.getAlliance().get() == Alliance.RED ? 1 : -1;
        }

        public static Transform3d kRobotToCamera1 = new Transform3d(
            new Translation3d(0.25, -.072, 0.09),
            new Rotation3d(0, Math.toRadians(10), 0)
        );

        // *Hub Positions
        static final Pose2d kRedHub = new Pose2d(11.901424, 4.024, Rotation2d.fromDegrees(0));
        static final Pose2d kBlueHub = new Pose2d(4.611, 4.024, Rotation2d.fromDegrees(0));

        public static Pose2d getHubPose() {
            return FieldUtil.allianceRelativePose(kBlueHub);
        }

        public static Pose2d getHubPose(Alliance alliance) {
            return alliance == Alliance.RED ? kRedHub : kBlueHub;
        }

        public static Pose2d getAimPose() {
            return ShooterConstants.getShootingSetpoint(1);
        }
        
        public static Pose2d getCenter() {
            return new Pose2d(FieldUtil.FIELD_MAX_X / 2, FieldUtil.FIELD_MAX_Y / 2, Rotation2d.fromDegrees(0));
        }
    }

    // *pivot motorID is 21 (honestly we need 2 motors, idk why we only have one)
    // All distances are in rotations
    public static final class PivotConstants {
        public static final int kPivotMotorCanId = 21;
        public static final double kPivotMotorPower = 0.1;

        public static final double kP = 1;
        public static final double kI = 0;
        public static final double kD = 0;

        public static final double kMaxVelocity = 1.0; // tune as needed
        public static final double kMaxAcceleration = 1.0; // tune as needed

        public static final double kSetpointA = 0.0;
        public static final double kSetpointB = 17;
        
        public static final double kUpPosition   = 0;
        public static final double kDownPosition = 0.5;
        public static final double kTolerance    = 0.05;

        public static final double kFreeSpeedRpm = 5676;
        public static final double kStallCurrent = 30;
    }
    
    // *intake motorID is 22
    public static final class IntakeConstants {
        public static final int kIntakeMotorCanId = 22;
        public static final double kIntakeMotorPower = 0.4;
    }

    // *shooter motorIDs are values of 30
    public static final class ShooterConstants {
        public static final int kShooterMotorCanId1 = 31;
        public static final int kShooterMotorCanId2 = 32;

        public static final int kKickerMotorCanId1 = 33;
        public static final int kKickerMotorCanId2 = 34;


        public static final double kKickerMotorPower = 0.59;
        

        // |Aimpose for testing setpoint pathing to shootinng
        // ?Facing toward blue hub at (4.611624, 4.024) from (2, 2)
        // ?Angle = atan2(2.024, 2.611624) ≈ 37.592°    
        public static final Pose2d[] kShootingSetpoints = {
            FieldUtil.facePose(new Pose2d(3.03, 0.75, Rotation2d.ZERO), VisionConstants.getHubPose(Alliance.BLUE)),
            FieldUtil.facePose(new Pose2d(2,    2,    Rotation2d.ZERO), VisionConstants.getHubPose(Alliance.BLUE)),
            FieldUtil.facePose(new Pose2d(1.6,  4,    Rotation2d.ZERO), VisionConstants.getHubPose(Alliance.BLUE)),
            FieldUtil.facePose(new Pose2d(2,    6,    Rotation2d.ZERO), VisionConstants.getHubPose(Alliance.BLUE)),
            FieldUtil.facePose(new Pose2d(3.03, 7.25, Rotation2d.ZERO), VisionConstants.getHubPose(Alliance.BLUE)),
        };

        public static void flipShootingSetpoints() {
            for (int i = 0; i < kShootingSetpoints.length; i++) {
                kShootingSetpoints[i] = FieldUtil.allianceRelativePose(kShootingSetpoints[i]);
            }
        }

        public static Pose2d getShootingSetpoint(int num) {
            return kShootingSetpoints[num - 1];
        }
    }

    // *climb motorIDs are values of 40
    public static final class ClimbConstants{
        public static final int kClimbMotorCanId = 41;
        public static final double kClimbSpeed = .1;
        public static final double kClimbMax = 15.0;
        private static final Pose2d kRedClimb = new Pose2d(14.94, 3.71, Rotation2d.fromDegrees(0));
        private static final Pose2d kBlueClimb = new Pose2d(1.6, 3.71, Rotation2d.fromDegrees(0));

        public static Pose2d getClimbPose(Alliance alliance) {
            return alliance == Alliance.RED ? kRedClimb : kBlueClimb;
        }
        
    }

    // *Constants for tag observation and pose validation
    public static final double kTagInvisibilityWaitTime = 0.5;
    public static final double kPoseValidationTime = 1.0;
}