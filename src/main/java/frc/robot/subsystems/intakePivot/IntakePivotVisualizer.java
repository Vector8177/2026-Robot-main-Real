package frc.robot.subsystems.intakePivot;

import static edu.wpi.first.units.Units.Radians;
import static frc.robot.Constants.IntakePivotConstants.INTAKE_PIVOT;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Angle;

public class IntakePivotVisualizer {
    private static final IntakePivotVisualizer VISUALIZER = new IntakePivotVisualizer();
    private IntakePivotVisualizer() {}

    public static IntakePivotVisualizer getInstance(){
        return VISUALIZER;
    }

    public void update(Angle theta){
        Logger.recordOutput("ArmPose", new Pose3d(INTAKE_PIVOT, new Rotation3d(0, theta.in(Radians), 0)));
    }
}
