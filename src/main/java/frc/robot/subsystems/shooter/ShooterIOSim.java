package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.ShooterConstants.*;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterIOSim implements ShooterIO {
    private final FlywheelSim shooterSim;
    
    private double commandedVoltage = 0.0;
    
    public ShooterIOSim() {
        shooterSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                SHOOTER_MOTOR,
                .01,  // Add to constants: 0.01 - 0.1 typical
                SHOOTER_GEARING),
            SHOOTER_MOTOR);
    }
    
    @Override
    public void setVoltage(Voltage voltage) {
        commandedVoltage = voltage.in(Volts);
        shooterSim.setInputVoltage(commandedVoltage);
    }
    
    @Override
    public Voltage getVoltage() {
        return Volts.of(commandedVoltage);
    }
    
    @Override
    public void periodic() {
        shooterSim.update(0.02);
    }

    @Override
    public void stop() {
        shooterSim.setInputVoltage(0);
    }

    @Override
    public AngularVelocity getAngularVelocity() {
        return shooterSim.getAngularVelocity();
    }
}