package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.ShooterConstants.*;
import static frc.robot.Constants.TurretConstants.TURRET_PIVOT;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.util.FuelSim;

public class ShooterVisualizer {
    private Supplier<Translation3d> fuelLaunchPointSupplier;
    private Supplier<Rotation3d> turretAngleSupplier;
    private Supplier<Pose2d> robotPoseSupplier;
    private Supplier<LinearVelocity> fuelVelocitySupplier;
    private static ShooterVisualizer vis = new ShooterVisualizer();

    public static ShooterVisualizer getInstance(){
        return vis;
    }

    private ShooterVisualizer() {};

    public void setRobotPoseSupplier(Supplier<Pose2d> robotPoseSupplier){
        this.robotPoseSupplier = robotPoseSupplier;
        fuelLaunchPointSupplier = () -> {
            return new Translation3d(
                robotPoseSupplier.get().getX() + TURRET_PIVOT.getX(), 
                robotPoseSupplier.get().getY() + TURRET_PIVOT.getY(), 
                TURRET_PIVOT.getZ());
        };
    }

    public void setTurretAngleSupplier(Supplier<Rotation3d> turretAngleFromRobotSupplier){
        this.turretAngleSupplier = () -> {
            Rotation3d robotRotation = new Rotation3d(robotPoseSupplier.get().getRotation());
            Rotation3d turretAngleFromRobot = turretAngleFromRobotSupplier.get();

            return turretAngleFromRobot.plus(robotRotation);
        };
    }

    public void setFlywheelAngularVelocitySupplier(Supplier<AngularVelocity> shooterAngularVelocitySupplier){
        this.fuelVelocitySupplier = () -> {
            AngularVelocity v =  shooterAngularVelocitySupplier.get();
            return MetersPerSecond.of(v.in(RadiansPerSecond) * SHOOTER_WHEEL_RADIUS.in(Meters));
        };
    }
    
    public void launchFuel(){
        System.out.println(fuelVelocitySupplier.get().in(MetersPerSecond));
        FuelSim.getInstance().spawnFuel(
            fuelLaunchPointSupplier.get(), 
            new Translation3d(-fuelVelocitySupplier.get().in(MetersPerSecond), 0, 0).rotateBy(turretAngleSupplier.get()));
    }
}
