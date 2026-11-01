package first.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.command2.sysid.SysIdRoutine;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Voltage;

import first.robot.Constants.IntakeConstants;

import static org.wpilib.units.Units.*;


public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    private final SysIdRoutine sysIdRoutine;

    private PIDController pid = new PIDController(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD);
    private SimpleMotorFeedforward ff = new SimpleMotorFeedforward(IntakeConstants.kS, IntakeConstants.kV, IntakeConstants.kA);

    private enum Mode {
        OPEN_LOOP,
        CLOSED_LOOP,
        SYSID, // SysId seems to have some kind of conversion error (when I used it the result was always 2pi times the expected value) so we're not gonna use it for now
        TESTING
    }

    private boolean zeroing = false;
    private Mode mode = Mode.OPEN_LOOP;
    private Voltage openLoopVoltage = Volts.of(0);

    public Intake(IntakeIO io) {
        this.io = io;
        io.powerRegisterMotors();
        io.setVoltage(Volts.of(0));

        this.sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                (voltage) -> {
                    io.setVoltage(voltage);
                    mode = Mode.SYSID;
                },
                log -> {
                    log.motor("intake")
                        .voltage(inputs.voltage)
                        .angularPosition(inputs.position)
                        .angularVelocity(inputs.velocity);
                },
                this
            )
        );
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        switch (mode) {
            case CLOSED_LOOP -> {
                double pidOut = pid.calculate(inputs.velocity.in(RadiansPerSecond), inputs.setpoint.in(RadiansPerSecond));
                double ffOut = ff.calculate(inputs.setpoint.in(RadiansPerSecond));
                double totalOut = Math.clamp(pidOut + ffOut, -12.0, 12.0);

                if (Math.abs(inputs.setpoint.in(RadiansPerSecond)) < 25) {
                    pid.reset();
                    io.setVoltage(Volts.of(0));
                    zeroing = true;
                } else {
                    zeroing = false;
                    io.setVoltage(Volts.of(totalOut));
                }

                Logger.recordOutput("Intake/PID", pidOut);
                Logger.recordOutput("Intake/FF", ffOut);
                Logger.recordOutput("Intake/Total", Volts.of(totalOut));
            }

            case OPEN_LOOP -> {
                pid.reset();
                io.setVoltage(openLoopVoltage);
            }

            case SYSID -> {
                pid.reset();
            }

            case TESTING -> {
                pid.reset();
                io.setVoltage(Volts.of(12));
            }
        }

        Logger.recordOutput("Intake/Zeroing", zeroing);
        Logger.recordOutput("Intake/Mode", mode);
        Logger.recordOutput("Intake/Max Velocity", IntakeConstants.kMaxVelocity);
        Logger.processInputs("LoggedIntake", inputs);
    }

    public void setSetpoint(double multiplier) {
        mode = Mode.CLOSED_LOOP;
        inputs.setpoint = RadiansPerSecond.of(Math.clamp(multiplier * IntakeConstants.kMaxVelocity.in(RadiansPerSecond), -IntakeConstants.kMaxVelocity.in(RadiansPerSecond), IntakeConstants.kMaxVelocity.in(RadiansPerSecond)));
    }

    public void setSetpoint(AngularVelocity omega) {
        mode = Mode.CLOSED_LOOP;
        inputs.setpoint = omega;
    }

    public void stop() {
        mode = Mode.OPEN_LOOP;
        openLoopVoltage = Volts.of(0);
    }

    public void setPWM(double multiplier) {
        mode = Mode.OPEN_LOOP;
        openLoopVoltage = Volts.of(Math.clamp(multiplier * 12, -12, 12));
    }

    public AngularVelocity getAngularVelocity() {
        return inputs.velocity;
    }

    // For system identification
    public Command sysIdQuasistaticForward() {
        return sysIdRoutine
            .quasistatic(SysIdRoutine.Direction.FORWARD)
            .finallyDo(() -> mode = Mode.OPEN_LOOP);
    }

    public Command sysIdQuasistaticReverse() {
        return sysIdRoutine
            .quasistatic(SysIdRoutine.Direction.REVERSE)
            .finallyDo(() -> mode = Mode.OPEN_LOOP);
    }

    public Command sysIdDynamicForward() {
        return sysIdRoutine
            .dynamic(SysIdRoutine.Direction.FORWARD)
            .finallyDo(() -> mode = Mode.OPEN_LOOP);
    }

    public Command sysIdDynamicReverse() {
        return sysIdRoutine
            .dynamic(SysIdRoutine.Direction.REVERSE)
            .finallyDo(() -> mode = Mode.OPEN_LOOP);
    }

}
