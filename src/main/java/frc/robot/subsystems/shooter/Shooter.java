package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs;


public class Shooter extends SubsystemBase {
    private final ShooterIO io;
    private final ShooterIOInputs inputs = new ShooterIOInputs();
    private double targetSpeed = 0;

    public Shooter(ShooterIO io) {
        this.io = io;
        new SlewRateLimiter(.4);
    }

    private void setSpeedRaw(double speed) {
        speed = MathUtil.clamp(speed, -3, 1);
        // Logger.recordOutput("Shooter speed", speed);
        io.setShooterVoltage(speed * ShooterConstants.MAX_VOLTAGE);
    }

    public void setSpeed(double speed) {
        targetSpeed = speed;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        // Logger.processInputs("Shooter", inputs);
        setSpeedRaw(targetSpeed);
    }

    public void stop() {
        io.stop();
    }
}
