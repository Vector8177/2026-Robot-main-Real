package frc.robot.subsystems.turret;


import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Voltage;
public interface TurretIO {
  @AutoLog
  class TurretIOInputs {
    public double turretVelocityRadPerSec = 0.0;
    public double turretAbsoluteEncoderPosition = 0.0;
    public double turretAppliedVolts = 0.0;
  }

  default void updateInputs(TurretIOInputs inputs) {}
  default void periodic(){}

  void applyVoltsY(Voltage a);
  void applyVoltsZ(Voltage a);

  Rotation3d getRotation();
}
