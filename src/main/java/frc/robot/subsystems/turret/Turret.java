package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.RobotContainer;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.turret.TurretIO.TurretIOInputs;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TorqueCurrentConfigs;

public class Turret extends SubsystemBase {
  private final PIDController pidController;
  private final TurretIO io;
  private final TurretIOInputs inputs = new TurretIOInputs();
  private double targetPosition;
  private ArmFeedforward feedForward;
  private boolean follow;
  private DoubleSupplier gyro;
  private double x;
  private double L1;
  private double L2;

  public Turret(TurretIO io) {

    this.io = io;
    pidController = new PIDController(TurretConstants.kP, TurretConstants.kI, TurretConstants.kD);
    pidController.setTolerance(.2);
    follow = false; 
    gyro = null;
    feedForward =
        new ArmFeedforward(
            TurretConstants.kS, TurretConstants.kG, TurretConstants.kV, TurretConstants.kA);
    this.x = -1;
    this.L1 = -1;
    this.L2 = -1;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    // Logger.processInputs("Wrist", inputs);
    //Logger.recordOutput("Wrist Current Position", io.getPosition());
    //System.out.println("TX --> "+LimelightHelpers.getTX("limelight-right"));
    // if(follow && LimelightHelpers.getTV("limelight-right") && gyro != null){
    //   // double distance = (TurretConstants.TAG_HEIGHT_METERS-TurretConstants.LIMELIGHT_HEIGHT_METERS)/
    //   //   (Math.tan(Math.toRadians(TurretConstants.LIMELIGHT_ANGLE-LimelightHelpers.getTY("limelight-right"))));

    //   // double angle = io.getPosition()-gyro.getAsDouble()-LimelightHelpers.getTX("limelight-right");
    //   // double x = Math.sqrt(Math.pow(TurretConstants.DISTANCE_TAG_TO_TARGET, 2)+Math.pow(distance, 2)
    //   //           -2*TurretConstants.DISTANCE_TAG_TO_TARGET*distance*Math.cos(Math.toRadians(180-angle)));
      


    //   // double theta = Math.asin((TurretConstants.DISTANCE_TAG_TO_TARGET*Math.sin(Math.toRadians(180-angle)))/x)
    //   //   +LimelightHelpers.getTX("limelight-right");
    //   // setTurretSetpoint(-theta/360.0);

    //   setTurretSetpoint(LimelightHelpers.getTX("limelight-right"));
    // }
    
    // double pidMotorSpeed =
    //     pidController.calculate(io.getPosition(), targetPosition)
    //         + feedForward.calculate(targetPosition, 0);
    // ;
    // // Logger.recordOutput("Wrist Speed", pidMotorSpeed);
     double pidMotorSpeed =
        pidController.calculate(io.getPosition(), targetPosition)
            + feedForward.calculate(targetPosition, 0);
      setMotor(
        MathUtil.clamp((pidMotorSpeed), -TurretConstants.MAX_VOLTAGE, TurretConstants.MAX_VOLTAGE));
    Logger.recordOutput("Turret Target Position", targetPosition);
    Logger.recordOutput("Turret Current Position", io.getPosition());
    //System.out.println("L1: "+x);
    Logger.recordOutput("L1", L1);
    Logger.recordOutput("L2", L2);
    RawFiducial[] limelight = LimelightHelpers.getRawFiducials("limelight-turret");
    if(limelight.length > 0){
          Logger.recordOutput("TX", limelight[0].txnc); //wont work in sim
    }
  }

  public void setMotor(double voltage) {
    io.setVoltage(voltage);
  }

  public void setPosition(double position) {
    Logger.recordOutput("Turret Target Position", position);
    System.out.println(io.getPosition() + " Turret");
    targetPosition = position;
  }

  public void setTurretSetpoint(double offset) {
    targetPosition = (io.getPosition() + offset);
  }

  public boolean atSetpoint() {
    return pidController.atSetpoint();
  }

  public void followGyro(boolean follow, DoubleSupplier gyro){
    this.follow = follow;
    this.gyro = gyro;
    if(follow && LimelightHelpers.getTV("limelight-turret") && gyro != null){
      double distance = (TurretConstants.TAG_HEIGHT_METERS-TurretConstants.LIMELIGHT_HEIGHT_METERS)/
        (Math.tan(Math.toRadians(TurretConstants.LIMELIGHT_ANGLE-LimelightHelpers.getTY("limelight-turret"))));
      System.out.println(distance);
      double angle = -io.getPosition()-gyro.getAsDouble()-LimelightHelpers.getTX("limelight-turret");
      double x = Math.sqrt(Math.pow(TurretConstants.DISTANCE_TAG_TO_TARGET, 2)+Math.pow(distance, 2)
                -2*TurretConstants.DISTANCE_TAG_TO_TARGET*distance*Math.cos(Math.toRadians(180-angle)));
      


      double theta = Math.asin((TurretConstants.DISTANCE_TAG_TO_TARGET*Math.sin(Math.toRadians(180-angle)))/x)
        +LimelightHelpers.getTX("limelight-turret");
      //setTurretSetpoint(theta/360.0);
      
      //setTurretSetpoint(LimelightHelpers.getTX("limelight-right"));
    }
  
    // Logger.recordOutput("Wrist Speed", pidMotorSpeed);
    // setMotor(
    //     MathUtil.clamp((pidMotorSpeed), -TurretConstants.MAX_VOLTAGE, TurretConstants.MAX_VOLTAGE));
  }

  public void followCamera(boolean follow){
    this.follow = follow;
    LimelightHelpers.SetFiducialIDFiltersOverride("limelight-turret", TurretConstants.cIds[0]);
    if(LimelightHelpers.getTargetCount("limelight-turret")==2){
        double delH = TurretConstants.TAG_HEIGHT_METERS-TurretConstants.LIMELIGHT_HEIGHT_METERS;
        double theta1 = 0d;
        double theta2 = 0d;
        double L1 = 0d;
        double L2 = 0d;
        double beta = 0d;
        double thetap1 = 0d;
        double thetap2 = 0d;
        double Sx = 0d;
        double Sy = 0d;
        double theta = 0d;
        double S = 0d;
        RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("limelight-turret");
        for(RawFiducial f : fiducials){
          int id = f.id;
          if(id==TurretConstants.cIds[0][0]){
              theta1 = f.txnc;
              L1 = (delH)/(Math.tan(Math.toRadians(TurretConstants.LIMELIGHT_ANGLE+f.tync)));
          }
          else{
            theta2 = f.txnc;
            L2 = (delH)/(Math.tan(Math.toRadians(TurretConstants.LIMELIGHT_ANGLE+f.tync)));
          }
        }
        beta = Math.abs(Math.toDegrees(Math.asin((L2*Math.sin(Math.toRadians(Math.abs(theta1-theta2))) / TurretConstants.DISTANCE_BETWEEN_TAGS))));
        thetap1 = 90 - beta;
        thetap2 = -Math.abs(theta2 - theta1) + thetap1; //Check
        Sx = L2 * Math.sin(Math.toRadians(thetap2)) + TurretConstants.DISTANCE_TAG_SIDE_TO_TARGET * Math.sin(Math.toRadians(TurretConstants.alpha));
        Sy = L2 * Math.cos(Math.toRadians(thetap2)) + TurretConstants.DISTANCE_TAG_SIDE_TO_TARGET * Math.cos(Math.toRadians(TurretConstants.alpha));
        theta = Math.abs(Math.toDegrees(Math.abs(Math.atan(Sx / Sy)) + theta2));
        S = Math.sqrt(Math.pow(Sx, 2) + Math.pow(Sy, 2));
        this.x = L1;
        if(theta1-theta2<0){
          theta = -theta;
        }
        System.out.println("Theta: "+(theta/10));
        this.L1 = L1;
        this.L2 = L2;
        
        //setTurretSetpoint(-theta/3600);
        
        //double L1 = (delH)/(Math.tan(TurretConstants.LIMELIGHT_ANGLE+LimelightHelpers.get))
    }

    LimelightHelpers.SetFiducialIDFiltersOverride("limelight-turret", TurretConstants.allId);
    
  } 
}