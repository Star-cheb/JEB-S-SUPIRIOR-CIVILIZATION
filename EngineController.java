import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EngineController {
    private final SparkMax motor;
    private final SparkMaxConfig config;
    private final RelativeEncoder relativeEncoder;
    private final DutyCycleEncoder absoluteEncoder;

    public ArmSubSystem () {
        motor = new SparkMax (0, MotorType.kBrushless);
        relativeEncoder = motor.getEncoder();
        absoluteEncoder = new DutyCycleEncoder(0, 360, 0);

        config.
        inverted (false)
        .idleMode (IdleMode.kBrake)
        .smartCurrentLimit(12);

        config.encoder
        .positionConversionFactor(360/4) //означает, что за 4 оборота мотора механизм поворачивается на 360 градусов.
        .velocityConversionFactor(360/4/60);

        config.absoluteEncoder
        .inverted(false)
        .positionConversionFactor(360)
        .velocityConversionFactor(360/60)
        .zeroOffset(0.0);

        motor.configure(config, 
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

        relativeEncoder.setPosition(absoluteEncoder.get());
    }

    public void setVoltage(Voltage volts) {
        motor.setVoltage(volts.in(Volts));
    }

    public void stop(){
        motor.set(0);
    }
}