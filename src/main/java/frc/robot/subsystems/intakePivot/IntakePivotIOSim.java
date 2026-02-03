package frc.robot.subsystems.intakePivot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.IntakePivotConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class IntakePivotIOSim implements IntakePivotIO {
    private final SingleJointedArmSim armSim = new SingleJointedArmSim(
            SIM_INTAKE_ARM_MOTOR,
            GEARING,
            SingleJointedArmSim.estimateMOI(INTAKE_ARM_LENGTH.in(Meters), INTAKE_ARM_MASS.in(Kilograms)),
            INTAKE_ARM_LENGTH.in(Meters),
            Units.degreesToRadians(-180), // TODO: placeholder
            Units.degreesToRadians(180), // TODO: placeholder
            false,
            0);// TODO: placeholder

    private final TalonFX arm = new TalonFX(MOTOR_ID);

    public IntakePivotIOSim() {
        TalonFXConfiguration config = new TalonFXConfiguration();
    
    // config.Slot0.kP =  kP;  
    // config.Slot0.kI = kI;   
    // config.Slot0.kD = kD;   
    // config.Slot0.kV = kV;   
    // config.Slot0.kS = kS;
    // config.Slot0.kG = kG;


        arm.getConfigurator().apply(config, .005);
        arm.setPosition(0);
        arm.setNeutralMode(NeutralModeValue.Brake);

    }

    // note: this function needs to be called every 20ms
    public void updateInputs(IntakePivotIOInputs inputs) {
        armSim.setInputVoltage(arm.getSimState().getMotorVoltage());
        armSim.update(0.02);

        arm.getSimState().setRawRotorPosition(armSim.getAngleRads() * GEARING / (2 * Math.PI));
        arm.getSimState().setRotorVelocity(
                armSim.getVelocityRadPerSec() * GEARING / (2 * Math.PI));


        inputs.intakePivotAppliedVolts = arm.getSimState().getMotorVoltage();
        inputs.intakePivotVelocityRadPerSec = armSim.getVelocityRadPerSec();

        IntakePivotVisualizer.getInstance()
                .update(Radians.of(armSim.getAngleRads()));
    }

    public double getPosition() {
        return arm.getPosition().getValueAsDouble();
    }

    public void setPosition(double position) {
        arm.setControl(new PositionVoltage(Radians.of(position)));
    }

    public void setVoltage(double speed) {
        arm.setVoltage(speed);
    }

}
