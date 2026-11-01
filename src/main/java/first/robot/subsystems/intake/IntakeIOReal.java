package first.robot.subsystems.intake;

import static org.wpilib.units.Units.*;

import org.wpilib.hardware.bus.CANPort;
import org.wpilib.units.measure.Voltage;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;

import first.robot.Constants.IntakeConstants;
import first.robot.util.misc.VirtualPD;

public class IntakeIOReal implements IntakeIO {
    private final SparkMax motor = new SparkMax(CANPort.CAN_S0, IntakeConstants.kIntakeMotorCanId, MotorType.kBrushless);
    private final RelativeEncoder encoder = motor.getEncoder();
    private final SparkMaxConfig motorConfig = new SparkMaxConfig();

    public IntakeIOReal() {
        motorConfig
            .inverted(false)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(40)
            .secondaryCurrentLimit(60);

        // I will note that persist mode writes parameters to SPARK's non-volatile memory, i.e. it lets them persist across power cycles.
        // Not sure how useful it will be if we want to rerun the robot code every power cycle to test new parameters.
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void powerRegisterMotors() {
        VirtualPD.registerMotor(() -> Amps.of(motor.getOutputCurrent().get()), "Intake");
    }

    @Override
    public void setVoltage(Voltage volts) {
        motor.setVoltage(volts);
    } 

    @Override 
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.voltage = Volts.of(motor.getBusVoltage().get() * motor.getAppliedOutput().get());
        inputs.velocity = RPM.of(encoder.getVelocity().get());
        inputs.position = Rotations.of(encoder.getPosition().get());
    }
}
