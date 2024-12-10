package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

public class Arm extends SubsystemBase {
    private int CAN_ID;
    private CANSparkMax motor = new CANSparkMax(CAN_ID, MotorType.kBrushless);
    private int CURRENT_LIMIT;
    private double ENCODER_TO_RADIANS;

    private final MutableMeasure<Voltage> sysIdVoltage = MutableMeasure.mutable(Volts.of(0));
    private final MutableMeasure<Angle> sysIdPosition = MutableMeasure.mutable(Radians.of(0));
    private final MutableMeasure<Velocity<Angle>> sysIdVelocity = MutableMeasure.mutable(RadiansPerSecond.of(0));
    private SysIdRoutine sysIdRoutine = new SysIdRoutine(new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(voltage -> setVoltage(voltage.magnitude()), log -> {
                log.motor("Arm").voltage(sysIdVoltage.mut_replace(getVoltage(), Volts))
                        .angularPosition(sysIdPosition.mut_replace(getPosition(), Radians))
                        .angularVelocity(sysIdVelocity.mut_replace(getVelocity(), RadiansPerSecond));
            }, this));

    public Arm() {
        motor.setSmartCurrentLimit(CURRENT_LIMIT);
        motor.getEncoder().setPositionConversionFactor(ENCODER_TO_RADIANS);
    }

    private void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    private double getVoltage() {
        return motor.getAppliedOutput();
    }

    private double getPosition() {
        return motor.getEncoder().getPosition();
    }

    private double getVelocity() {
        return motor.getEncoder().getVelocity();
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }
}
