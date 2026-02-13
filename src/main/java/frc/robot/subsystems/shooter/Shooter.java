package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOState;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOStateAutoLogged;

public class Shooter extends SubsystemBase {
    MutVoltage goalVoltage = Volts.mutable(0);
    ShooterIO io;
    ShooterIOStateAutoLogged state = new ShooterIOStateAutoLogged();

    public Shooter(ShooterIO io){
        this.io = io;
    }

    @Override
    public void periodic() {
        super.periodic();
        io.periodic();
        

        // logging
        updateState(state);
        Logger.processInputs("shooter",  state);
    }

    public void setVoltage(Voltage voltage){
        this.goalVoltage.mut_replace(voltage);
        this.io.setVoltage(voltage);
    }

    public void updateState(ShooterIOStateAutoLogged state){
        state.shooterAppliedVoltageVolts = io.getVoltage().in(Volts);
        state.shooterGoalVoltageVolts = goalVoltage.in(Volts);
        state.shooterVelocityRPM = io.getAngularVelocity().in(RotationsPerSecond);
    }

    public AngularVelocity getAngularVelocity(){
        return io.getAngularVelocity();
    }
}
