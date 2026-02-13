package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public interface ShooterIO {

    class ShooterIOStateAutoLogged extends ShooterIOState implements LoggableInputs {
    public void toLog(LogTable table) {
        table.put("shooterVelocityRPM", shooterVelocityRPM);
        table.put("shooterGoalVoltageVolts", shooterGoalVoltageVolts);
        table.put("shooterAppliedVoltageVolts", shooterAppliedVoltageVolts);
    }

    public void fromLog(LogTable table) {
        shooterVelocityRPM = table.get("shooterVelocityRPM", shooterVelocityRPM);
        shooterGoalVoltageVolts = table.get("shooterGoalVoltageVolts", shooterGoalVoltageVolts);
        shooterAppliedVoltageVolts = table.get("shooterAppliedVoltageVolts", shooterAppliedVoltageVolts);
    }
}

    @AutoLog
    class ShooterIOState {
        public double shooterVelocityRPM = 0.0;
        public double shooterGoalVoltageVolts = 0.0;
        public double shooterAppliedVoltageVolts = 0.0;
    }

    public void setVoltage(Voltage voltage);
    public void stop();
    public AngularVelocity getAngularVelocity();
    public Voltage getVoltage();

    //called every 20ms
    default void periodic() {};


}
