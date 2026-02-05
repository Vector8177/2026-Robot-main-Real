package frc.robot.subsystems.intakePivot;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static frc.robot.Constants.IntakePivotConstants.INTAKE_PIVOT;
import static frc.robot.Constants.IntakePivotConstants.INTAKE_POSITION;
import static frc.robot.Constants.IntakePivotConstants.INTAKE_ARM_STARTING_ANGLE;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;

public class IntakePivotVisualizer {
    private static final IntakePivotVisualizer VISUALIZER = new IntakePivotVisualizer();
    private MutAngle angle = Radians.mutable(0);

    private IntakePivotVisualizer() {}

    public static IntakePivotVisualizer getInstance(){
        return VISUALIZER;
    }

    public void update(Angle angle){
        Logger.recordOutput("ArmPose", new Pose3d(INTAKE_PIVOT, new Rotation3d(0, angle.plus(INTAKE_ARM_STARTING_ANGLE).in(Radians), 0)));
        this.angle.mut_replace(angle);
    }

    public boolean isDeployed(){
        return angle.minus(Radians.of(INTAKE_POSITION)).abs(Degrees) < 5; //within 5 degrees of deployed angle
    }
}
