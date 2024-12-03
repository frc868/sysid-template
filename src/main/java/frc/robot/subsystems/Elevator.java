package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

public class Elevator extends SubsystemBase {
    private int CAN_ID;
    private CANSparkMax motor = new CANSparkMax(CAN_ID, MotorType.kBrushless);

    private final MutableMeasure<Voltage> sysIdVoltage = MutableMeasure.mutable(Volts.of(0));
    private final MutableMeasure<Distance> sysIdPosition = MutableMeasure.mutable(Meters.of(0));
    private final MutableMeasure<Velocity<Distance>> sysIdVelocity = MutableMeasure.mutable(MetersPerSecond.of(0));
    private SysIdRoutine sysIdRoutine = new SysIdRoutine(new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(voltage -> setVoltage(voltage.magnitude()), log -> {
                log.motor("Elevator").voltage(sysIdVoltage.mut_replace(getVoltage(), Volts))
                        .linearPosition(sysIdPosition.mut_replace(getPosition(), Meters))
                        .linearVelocity(sysIdVelocity.mut_replace(getVelocity(), MetersPerSecond));
            }, this));

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
