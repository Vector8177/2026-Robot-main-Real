package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.TurretConstants.*;

public class TurretIOSim implements TurretIO{
    //TODO: make all state use Mut

    final DCMotorSim zMtr;
    final DCMotorSim yMtr;

    public TurretIOSim() {
        yMtr = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(TURRET_Y_MOTOR, .001, TURRET_Y_GEARING), //TODO: fix MOI
            TURRET_Y_MOTOR);
        zMtr = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(TURRET_Z_MOTOR, .001, TURRET_Z_GEARING),  //TODO: fix MOI
            TURRET_Z_MOTOR);
    }

    @Override
    public void periodic() {
        yMtr.update(0.02);
        zMtr.update(0.02);

        TurretVisualizer.getInstance().update(getRotation());
    }

    @Override
    public void applyVoltsY(Voltage v) {
        double clampedVoltage = MathUtil.clamp(v.in(Volts), -12, 12);
        yMtr.setInputVoltage(clampedVoltage);
    }

    @Override
    public void applyVoltsZ(Voltage v) {
        double clampedVoltage = MathUtil.clamp(v.in(Volts), -12, 12);
        zMtr.setInputVoltage(clampedVoltage);
    }

    @Override
    public Rotation3d getRotation() {
        return new Rotation3d(0, yMtr.getAngularPositionRad(), zMtr.getAngularPositionRad());
    }

    
}
