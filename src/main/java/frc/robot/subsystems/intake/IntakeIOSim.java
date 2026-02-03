package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOSim implements IntakeIO {
    private final DCMotorSim motorSim;

    public IntakeIOSim(){
        motorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                IntakeConstants.SIM_INTAKE_MOTOR, 
                4 * 0.5 * 4e-5 * Math.pow(.0038, 2), //supposedly estimates MOI for 4 rollers
                .5), //placeholder gearing
            IntakeConstants.SIM_INTAKE_MOTOR);
    }

    //note: needs to be called every 20ms
    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakeAppliedVolts = motorSim.getInputVoltage();
        inputs.intakeVelocityRadPerSec = motorSim.getAngularVelocityRadPerSec();

        motorSim.update(.02);
    }

    @Override
    public void setIntakeVoltage(double volts) {
        motorSim.setInputVoltage(volts);
    }

    @Override
    public void stop() {
        motorSim.setInputVoltage(0);
    }
}
