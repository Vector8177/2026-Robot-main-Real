package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;

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
            TURRET_Y_MOTOR);
        
        CurrentLimitsConfigs currentConfig = new CurrentLimitsConfigs();
        currentConfig.StatorCurrentLimit = 80;
        currentConfig.StatorCurrentLimitEnable = true;

    }

    @Override
    public void applyVoltsY(Voltage v) {
        yMtr.setInputVoltage(v.in(Volts));
    }

    @Override
    public void applyVoltsZ(Voltage v) {
        zMtr.setInputVoltage(v.in(Volts));
    }

    @Override
    public Rotation3d getRotation() {
        return new Rotation3d(0, yMtr.getAngularPositionRad(), zMtr.getAngularPositionRad());
    }

    
}
