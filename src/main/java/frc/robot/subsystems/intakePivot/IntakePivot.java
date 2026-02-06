package frc.robot.subsystems.intakePivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakePivotConstants;
import frc.robot.subsystems.intakePivot.IntakePivotIO.IntakePivotIOInputs;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static frc.robot.Constants.IntakePivotConstants.INTAKE_POSITION;
import static frc.robot.Constants.Dimensions.MAX_FUEL;

import org.littletonrobotics.junction.Logger;

public class IntakePivot extends SubsystemBase {
  private final PIDController pidController;
  private final IntakePivotIO io;
  private final IntakePivotIOInputs inputs = new IntakePivotIOInputs();
  private double targetPosition;
  private ArmFeedforward feedForward;
  private int fuels = 0;

  public IntakePivot(IntakePivotIO io) {

    this.io = io;
    pidController = new PIDController(IntakePivotConstants.kP, IntakePivotConstants.kI, IntakePivotConstants.kD);
    pidController.setTolerance(.2);

    feedForward =
        new ArmFeedforward(
            IntakePivotConstants.kS, IntakePivotConstants.kG, IntakePivotConstants.kV, IntakePivotConstants.kA);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    // Logger.processInputs("IntakePivot", inputs);
    Logger.recordOutput("IntakePivot Current Position", io.getPosition());
    double pidMotorSpeed =
        pidController.calculate(io.getPosition(), targetPosition)
            + feedForward.calculate(targetPosition, 0);
    Logger.recordOutput("IntakePivot Speed", pidMotorSpeed);
    setMotor(
        MathUtil.clamp((pidMotorSpeed), -IntakePivotConstants.MAX_VOLTAGE, IntakePivotConstants.MAX_VOLTAGE));
  }

  public void setMotor(double voltage) {
    io.setVoltage(voltage);
  }

  public void setPosition(double position) {
    Logger.recordOutput("IntakePivot Target Position", position);
    System.out.println(io.getPosition() + " IntakePivot");
    targetPosition = position;
    io.setPosition(position);
  }

  public void setIntakePivotSetpoint(double offset) {
    targetPosition = (io.getPosition() + offset);
  }

  public boolean atSetpoint() {
    return pidController.atSetpoint();
  }

  public boolean canIntake(){
    boolean isAtCorrectPosition =  Math.abs(io.getPosition() - INTAKE_POSITION) < Degrees.of(5).in(Radians);
    boolean enoughCapacity = fuels <= MAX_FUEL;

    return isAtCorrectPosition && enoughCapacity;
  }

  public void intake(){
    fuels++;
  }
}
