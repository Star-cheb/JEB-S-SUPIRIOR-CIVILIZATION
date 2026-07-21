import static edu.wpi.first.units.Units.Degrees;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem {
    private final SparkMax motor;
    private final SparkMaxConfig config;
    private final RelativeEncoder relativeEncoder;
    private final DutyCycleEncoder absoluteEncoder;
    private final SparkClosedLoopController daController;
    public ArmSubsystem () {
        motor = new SparkMax (0, MotorType.kBrushless);
        relativeEncoder = motor.getEncoder();
        absoluteEncoder = new DutyCycleEncoder(0, 360, 0);

        config.
        inverted (false)
        .idleMode (IdleMode.kBrake)
        .smartCurrentLimit(12);

    config.closedLoop.pid(
    ArmConstants.kP,
    ArmConstants.kI, 
    ArmConstants.kD)
    .feedForward 
    .kS(ArmConstants.kS)
    .kG(ArmConstants.kG)
    .kA(ArmConstants.kA)
    .kV(ArmConstants.kV);

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

    public void moveDaArm (Angle angle ){
    daController.setSetpoint(angle.in(Degrees), ControlType.kMAXMotionPositionControl);
    }

}