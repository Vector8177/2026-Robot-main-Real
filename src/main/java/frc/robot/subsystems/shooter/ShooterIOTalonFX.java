package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;


public class ShooterIOTalonFX implements ShooterIO{
    private final TalonFX leftShooterMotor;
    private final TalonFX rightShooterMotor;
    private final TalonFXConfiguration configuration;

    public ShooterIOTalonFX() {
        leftShooterMotor = new TalonFX(Constants.ShooterConstants.LEFT_MOTOR_ID);
        rightShooterMotor = new TalonFX(Constants.ShooterConstants.RIGHT_MOTOR_ID);

        configuration = new TalonFXConfiguration();
        leftShooterMotor.getConfigurator().apply(configuration, .05);
        rightShooterMotor.getConfigurator().apply(configuration, .05);

        rightShooterMotor.setControl(new Follower(Constants.ShooterConstants.LEFT_MOTOR_ID, MotorAlignmentValue.Aligned));

        leftShooterMotor.setNeutralMode(NeutralModeValue.Brake);
        rightShooterMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.leftShooterAppliedVolts =
        leftShooterMotor.getDutyCycle().getValueAsDouble()
            * leftShooterMotor.getSupplyVoltage().getValueAsDouble();
    inputs.leftShooterVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(
            leftShooterMotor.getVelocity().getValueAsDouble());

    inputs.rightShooterAppliedVolts =
        rightShooterMotor.getDutyCycle().getValueAsDouble()
            * rightShooterMotor.getSupplyVoltage().getValueAsDouble();
    inputs.rightShooterVelocityRadPerSec=
        Units.rotationsPerMinuteToRadiansPerSecond(
            rightShooterMotor.getVelocity().getValueAsDouble());
  }

  @Override
  public void setShooterVoltage(double volts) {
    leftShooterMotor.setVoltage(volts);
    rightShooterMotor.setVoltage(volts);
  }

  @Override
  public void stop() {
    leftShooterMotor.setVoltage(0);
    rightShooterMotor.setVoltage(0);
  }
}
