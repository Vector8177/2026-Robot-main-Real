package frc.robot.subsystems.turret;

import frc.robot.util.FuelSim;

public class TurretVisualizer {
    private static TurretVisualizer vis = new TurretVisualizer();

    private TurretVisualizer() {};

    public static TurretVisualizer getInstance(){
        return vis;
    }

    public void launchFuel(){
        // FuelSim.getInstance().launchFuel();
    }
}
