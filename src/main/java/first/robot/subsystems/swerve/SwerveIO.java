package first.robot.subsystems.swerve;

import org.littletonrobotics.junction.AutoLog;

import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.math.geometry.Pose2d;

/**
 * Swerve IO inputs struct for AdvantageKit.
 * This matches CommandSwerveDrivetrain.updateInputs exactly.
 */

public interface SwerveIO {
    @AutoLog
    public static class SwerveIOInputs {
        /** Full module states from CTRE swerve state. */
        public SwerveModuleVelocity[] moduleStates;

        /** Timestamp in seconds (Utils.getCurrentTimeSeconds()). */
        public double timestamp;

        /** Robot chassis speeds (vx, vy, omega). */
        public ChassisVelocities robotChassisVelocities;

        /** Robot heading in radians (from Pose rotation). */
        public double robotHeading;

        /** Total current draw from all drive + steer motors. */
        public double totalCurrent;

        /** Battery voltage. */
        public double totalVoltage;

        /** Whether the drive is field oriented (currently always false). */
        public boolean isFieldOriented;

        /** Full robot pose from CTRE swerve state. */
        public Pose2d robotPose;
    }

    public void updateInputs(SwerveIOInputs inputs);
}
