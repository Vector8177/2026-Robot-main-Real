package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    default void updateInputs(ShooterIOInputs inputs) {}

    default void setShooterVoltage(double volts) {}

    default void stop() {}

    @AutoLog
    class ShooterIOInputs {
    double leftShooterVelocityRadPerSec = 0d;
    double rightShooterVelocityRadPerSec = 0d;
    double leftShooterAppliedVolts = 0d;
    double rightShooterAppliedVolts = 0d;
  }
}
