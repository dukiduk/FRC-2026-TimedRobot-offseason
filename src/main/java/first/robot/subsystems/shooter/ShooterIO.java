package first.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;
import org.wpilib.units.measure.*;

public interface ShooterIO {
    @AutoLog 
    public static class ShooterIOInputs {
        public AngularVelocity velocity;
        public Voltage voltage;
        public Angle position;
    }

    public void setVoltage(Voltage voltage);
    public void stop();
    public void registerMotors();

    public void updateInputs(ShooterIOInputs inputs);
}
