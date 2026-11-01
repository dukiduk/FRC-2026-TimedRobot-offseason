package first.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Voltage;

public interface IntakeIO {
    @AutoLog 
    public static class IntakeIOInputs {
        public AngularVelocity velocity;
        public AngularVelocity setpoint;
        public Voltage voltage;
    }

    public void powerRegisterMotors();
    public void setVoltage(Voltage volts);
    public void updateInputs(IntakeIOInputs inputs);
}
