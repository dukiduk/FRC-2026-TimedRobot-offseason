package first.robot.subsystems.shooter;

import static org.wpilib.units.Units.*;

import org.wpilib.math.numbers.N1;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.LinearSystem;
import org.wpilib.math.system.Models;
import org.wpilib.simulation.FlywheelSim;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Voltage;

import first.robot.Constants.IntakeConstants;
import first.robot.subsystems.intake.IntakeIO.IntakeIOInputs;
import first.robot.util.misc.VirtualPD;

public class ShooterIOSim implements ShooterIO {
    LinearSystem<N1,N1,N1> system = Models.flywheelFromPhysicalConstants(DCMotor.getNEO(2), 1, 1)
    private final FlywheelSim sim = new FlywheelSim(system, DCMotor.getNEO(2), 1.0);
    private Angle angle = Radians.of(0);
    private Voltage appliedVolts = Volts.of(0);


     @Override 
    public void setVoltage(Voltage volts) {
        appliedVolts = volts;
    }

    @Override 
    public void registerMotors() {
        VirtualPD.registerMotor(() -> Amps.of(sim.getCurrentDraw()), "Shooter");
    }

    @Override 
    public void updateInputs(ShooterIOInputs inputs) {
        sim.setInputVoltage(appliedVolts.in(Volts));
        sim.update(0.02);

        inputs.voltage = appliedVolts;
        inputs.velocity = RadiansPerSecond.of(sim.getAngularVelocity());
        angle = angle.plus(Radians.of(sim.getAngularVelocity() * 0.02));
        inputs.position = angle;
    }
}
