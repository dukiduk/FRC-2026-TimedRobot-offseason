package first.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;
import org.wpilib.units.measure.*;

public interface IntakeIO {
    @AutoLog 
    public static class IntakeIOInputs {
        public Angle position;
        public AngularVelocity velocity;
        public Voltage voltage;
        public AngularVelocity setpoint;
    }

    public void powerRegisterMotors();
    public void setVoltage(Voltage volts);
    public void updateInputs(IntakeIOInputs inputs);
}
