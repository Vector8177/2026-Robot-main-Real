package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;

// import org.littletonrobotics.junction.Logger;
// import org.littletonrobotics.junction.inputs.LoggableInputs;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputs inputs = new IntakeIOInputs();
  private double targetSpeed = 0;

  public Intake(IntakeIO io) {
    this.io = io;
    new SlewRateLimiter(.4);
  }

  private void setSpeedRaw(double speed) {
    speed = MathUtil.clamp(speed, -3, 1);
    // Logger.recordOutput("Intake speed", speed);
    io.setIntakeVoltage(speed * IntakeConstants.MAX_VOLTAGE);
  }

  public void setSpeed(double speed) {
    System.out.println("setting the speed");
    targetSpeed = speed;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    // Logger.processInputs("Intake", inputs);

    setSpeedRaw(targetSpeed);
  }

  public void stop() {
    io.stop();
  }
}
