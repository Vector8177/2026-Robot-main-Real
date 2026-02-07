package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakePivotConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intakePivot.IntakePivot;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.wrist.Wrist;

public class MainCommands {

  private MainCommands() {}

  // public static Command track(Turret turret, Vision vision){

  // }

  // public static Command turnTurret(Turret turret){
  //   return runOnce(()-> turret.setSpeed(.3), turret);
  // }

  // public static Command turnTurretBackward(Turret turret){
  //   return runOnce(()-> turret.setSpeed(-.3), turret);
  // }

  // public static Command stopTurret(Turret turret){
  //   return runOnce(()-> turret.setSpeed(.0), turret);}
  
  
  public static Command stopIntake(Intake intake) {
    return runOnce(() -> intake.setSpeed(0), intake);
  }

  public static Command runOutakeFast(Intake intake) {
    return runOnce(() -> intake.setSpeed(-3), intake);
  }

  public static Command runIntake(Intake intake) {
    return runOnce(() -> intake.setSpeed(.3), intake);
  }

  public static Command runOuttake(Intake intake) {
    return runOnce(() -> intake.setSpeed(-.3), intake);
  }

  public static Command runOuttakeSlow(Intake intake) {
    return runOnce(() -> intake.setSpeed(-.15), intake);
  }

  public static Command runIntakeSlow(Intake intake) {
    return runOnce(() -> intake.setSpeed(.15), intake);
  }


  public static Command turretFollow(Turret turret){
    return runOnce(()-> turret.followCamera());
      
    //return runOnce(() -> turret.followCamera(true), turret);
  }
  // public static Command stopTurretFollow(Turret turret){
  //   return runOnce(() -> turret.followCamera(false), turret);
  // }


 

  // Shuts hood down and unspools turret
  public static Command stow(Wrist wrist, Turret turret) {
    return sequence(
        runOnce(() -> wrist.setPosition(0), wrist),
        runOnce(() -> turret.setPosition(0), turret));
  }

  public static Command intakePosition(IntakePivot intakePivot){
    return runOnce(() -> intakePivot.setPosition(IntakePivotConstants.INTAKE_POSITION));
  }
  
  public static Command stowIntake(IntakePivot intakePivot){
    return runOnce(() -> intakePivot.setPosition(0));
  }

   // public static Command moveTurret0(Turret turret){
  //   return runOnce(() -> turret.setPosition(TurretConstants.posO), turret);
  // }
  // public static Command moveTurret1(Turret turret){
  //   return runOnce(() -> turret.setPosition(TurretConstants.pos1), turret);
  // }
  // public static Command moveTurret2(Turret turret){
  //   return runOnce(() -> turret.setPosition(TurretConstants.pos2), turret);
  // }
  // public static Command moveTurret3(Turret turret){
  //   return runOnce(() -> turret.setPosition(TurretConstants.pos3), turret);
  // }
}