package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.sim.TalonFXSimState;

import frc.robot.Constants.IntakeConstants;

public class IntakeIOSim implements IntakeIO {
    private final TalonFX intakeMotor;
    private final TalonFXSimState intakeMotorSim;
    private final NeutralOut neutralOut = new NeutralOut();

    public IntakeIOSim(){
        intakeMotor = new TalonFX(IntakeConstants.MOTOR_ID);
        intakeMotorSim = intakeMotor.getSimState();
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {

    }

    @Override
    public void setIntakeVoltage(double volts) {
        intakeMotorSim.setSupplyVoltage(volts);
    }

    @Override
    public void stop() {
        // intakeMotorSim.set(neutralOut);
    }
}
