package first.robot.subsystems.shooter;
import static org.wpilib.units.Units.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.util.Signal;

import first.robot.util.misc.VirtualPD;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import org.wpilib.hardware.bus.CANPort;
import org.wpilib.units.measure.Voltage;

public class ShooterIOReal implements ShooterIO {
    private final SparkMax shooterMotor1;
    private final SparkMax shooterMotor2;
    
    private final SparkMaxConfig shooterConfig1 = new SparkMaxConfig();
    private final SparkMaxConfig shooterConfig2 = new SparkMaxConfig();

    //private final Signal<Double> velocitySignal;
    private final RelativeEncoder shooterEncoder1;
    private final RelativeEncoder shooterEncoder2;

    public ShooterIOReal() {
        shooterMotor1 = new SparkMax(CANPort.CAN_S1, 67, MotorType.kBrushless);
        shooterMotor2 = new SparkMax(CANPort.CAN_S1, 69, MotorType.kBrushless);

        shooterConfig1
            .inverted(false)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(40)
            .secondaryCurrentLimit(60);
        shooterConfig2
            //.inverted(true)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(40)
            .secondaryCurrentLimit(60)
            .follow(shooterMotor1, true);

        shooterMotor1.configure(shooterConfig1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        shooterMotor2.configure(shooterConfig2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        shooterEncoder1 = shooterMotor1.getEncoder();
        shooterEncoder2 = shooterMotor2.getEncoder();
    }

    @Override
    public void registerMotors() {
        VirtualPD.registerMotor(() -> Amps.of(shooterMotor1.getOutputCurrent().get()), "Shooter");
        VirtualPD.registerMotor(() -> Amps.of(shooterMotor2.getOutputCurrent().get()), "Shooter");
    }

    @Override
    public void setVoltage(Voltage voltage) {
        shooterMotor1.setVoltage(voltage);
    }

    @Override
    public void stop() {
        shooterMotor1.setVoltage(0.0);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.voltage = Volts.of((shooterMotor1.getBusVoltage().get() * shooterMotor1.getAppliedOutput().get() + shooterMotor2.getBusVoltage().get() * shooterMotor2.getAppliedOutput().get()));
        inputs.velocity = RadiansPerSecond.of((shooterEncoder1.getVelocity().get() + shooterEncoder2.getVelocity().get()) * 0.5);
        inputs.position = Radians.of((shooterEncoder1.getPosition().get() + shooterEncoder2.getPosition().get()) * 0.5);
    }
}
