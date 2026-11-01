package first.robot.subsystems.intake;

import static org.wpilib.units.Units.*;

import org.wpilib.math.numbers.N1;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.LinearSystem;
import org.wpilib.math.system.Models;
import org.wpilib.simulation.FlywheelSim;
import org.wpilib.units.measure.*;

import first.robot.Constants.IntakeConstants;
import first.robot.util.misc.VirtualPD;

public class IntakeIOSim implements IntakeIO{
    private final LinearSystem<N1, N1, N1> system = Models.flywheelFromPhysicalConstants(DCMotor.getNEO(1), IntakeConstants.kMOI.in(KilogramSquareMeters), IntakeConstants.gearing);
    private final FlywheelSim sim = new FlywheelSim(system, DCMotor.getNEO(1), 1.0);
    private Angle angle = Radians.of(0);
    private Voltage appliedVolts = Volts.of(0);

    @Override 
    public void setVoltage(Voltage volts) {
        appliedVolts = volts;
    }

    @Override 
    public void powerRegisterMotors() {
        VirtualPD.registerMotor(() -> Amps.of(sim.getCurrentDraw()), "Intake");
    }

    @Override 
    public void updateInputs(IntakeIOInputs inputs) {
        sim.setInputVoltage(appliedVolts.in(Volts));
        sim.update(IntakeConstants.kDt);

        inputs.voltage = appliedVolts;
        inputs.velocity = RadiansPerSecond.of(sim.getAngularVelocity());
        angle = angle.plus(Radians.of(sim.getAngularVelocity() * IntakeConstants.kDt));
        inputs.position = angle;
    }
}
