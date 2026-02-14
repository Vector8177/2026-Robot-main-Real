package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Radians;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Angle;
import frc.robot.util.FuelSim;
import static frc.robot.Constants.TurretConstants.*;

public class TurretVisualizer {
    private static TurretVisualizer vis = new TurretVisualizer();

    private TurretVisualizer() {};

    public static TurretVisualizer getInstance(){
        return vis;
    }

    public void update(Rotation3d rot){
        Logger.recordOutput("TurretPose", new Pose3d(TURRET_PIVOT, rot));
    }

    public void launchFuel(){
        // FuelSim.getInstance().launchFuel();
    }
}
