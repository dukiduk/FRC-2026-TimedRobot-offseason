package first.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.util.MathUtil;
import org.wpilib.units.measure.AngularVelocity;

import first.robot.Constants.IntakeConstants;

import static org.wpilib.units.Units.*;


public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    private PIDController pid;
    private  SimpleMotorFeedforward ff;
    private boolean controlled;

    public Intake(IntakeIO io) {
        this.io = io;
        io.powerRegisterMotors();

        pid.setPID(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD);
        ff.setKv(IntakeConstants.kV);
        ff.setKa(IntakeConstants.kA);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        if (controlled) {
            io.setVoltage(Volts.of(Math.clamp(pid.calculate(inputs.velocity.in(RPM), inputs.setpoint.in(RPM)) + ff.calculate(inputs.setpoint.in(RPM)), -12.0, 12.0)));
        }
        Logger.processInputs("LoggedIntake", inputs);
    }

    public void setSetpoint(double percent) {
        controlled = true;
        inputs.setpoint = RPM.of(percent * 5676);
    }

    public void setSetpoint(AngularVelocity omega) {
        controlled = true;
        inputs.setpoint = omega;
    }

    public void stop() {
        controlled = false;
        io.setVoltage(Volts.of(0));
    }

    public void setPWM(double percent) {
        controlled = false;
        io.setVoltage(Volts.of(percent * 12));
    }

    public AngularVelocity getRPM() {
        return inputs.velocity;
    }

}
