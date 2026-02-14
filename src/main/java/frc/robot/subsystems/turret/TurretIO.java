package frc.robot.subsystems.turret;


import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.Voltage;
public interface TurretIO {
  @AutoLog
  class TurretIOInputs {
    public MutAngularVelocity yAngularVelocity = DegreesPerSecond.mutable(0);
    public MutAngularVelocity zAngularVelocity = DegreesPerSecond.mutable(0);
    public MutAngle yAngle = Degrees.mutable(0);
    public MutAngle zAngle = Degrees.mutable(0);
    public MutAngle yGoalAngle = Degrees.mutable(0);
    public MutAngle zGoalAngle = Degrees.mutable(0);
  }

  default void updateInputs(TurretIOInputsAutoLogged inputs) {}
  default void periodic(){}

  void applyVoltsY(Voltage a);
  void applyVoltsZ(Voltage a);

  Rotation3d getRotation();
}
