package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Angle;
// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.units.measure.Voltage;

public class TurretIOTalonFX implements TurretIO {

  @Override
  public void applyVoltsY(Voltage a) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'applyVoltsY'");
  }

  @Override
  public void applyVoltsZ(Voltage a) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'applyVoltsZ'");
  }

  @Override
  public Rotation3d getRotation() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getRotation'");
  }
  // private final TalonFX turretMotor;
  // private final TalonFXConfiguration configuration;
  // // private final DigitalInput input;
  // // private final DutyCycleEncoder encoder;

  // public TurretIOTalonFX() {
  //   turretMotor = new TalonFX(TurretConstants.MOTOR_ID);
  //   configuration = new TalonFXConfiguration();

  //   turretMotor.getConfigurator().apply(configuration, .05);
  //   turretMotor.setPosition(0);
  //   turretMotor.setNeutralMode(NeutralModeValue.Brake);

  //   // input = new DigitalInput(0);
  //   // encoder = new DutyCycleEncoder(input);
  //   // // encoder.setDutyCycleRange(0.01, .99);
  // }

  // @Override
  // public void updateInputs(TurretIOInputs inputs) {
  //   inputs.turretAppliedVolts =
  //       turretMotor.getDutyCycle().getValueAsDouble()
  //           * turretMotor.getSupplyVoltage().getValueAsDouble();
  //   inputs.turretVelocityRadPerSec =
  //       Units.rotationsPerMinuteToRadiansPerSecond(turretMotor.getVelocity().getValueAsDouble());
  //   // inputs.wristAbsoluteEncoderPosition = encoder.get();
  // }

  // @Override
  // public void setVoltage(double voltage) {
  //   turretMotor.setVoltage(voltage);
  // }

  // @Override
  // public double getPosition() {
  //   return turretMotor.getPosition().getValueAsDouble();
  //   // if (encoder.get() > .75) {
  //   //   return .6;
  //   // }
  //   // return encoder.get();

  // }

  // @Override
  // public void setPosition(double position) {
  //   turretMotor.setPosition(position);
  // }
  
}
