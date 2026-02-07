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
  double delH = -1;
    double theta1 = -1;
        double theta2 = -1;
        double L1 = -1;
        double L2 = -1;
        double beta = -1;
        double phi = -1;
        double thetap1 = -1;
        double thetap2 = -1;
        double Sx = -1;
        double Sy = -1;
        double S = -1;
        double theta = -1;
        double y1 = -1;
        double y2 = -1;

  public Turret(TurretIO io) {

    this.io = io;
    pidController = new PIDController(TurretConstants.kP, TurretConstants.kI, TurretConstants.kD);
    pidController.setTolerance(.2);
    follow = false; 
    gyro = null;
    feedForward =
        new ArmFeedforward(
            TurretConstants.kS, TurretConstants.kG, TurretConstants.kV, TurretConstants.kA);
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
    // Logger.recordOutput("Turret Target Position", targetPosition);
    // Logger.recordOutput("Turret Current Position", io.getPosition());
    // //System.out.println("L1: "+x);
    // Logger.recordOutput("L1", L1);
    // Logger.recordOutput("L2", L2);
    // Logger.recordOutput("TX", LimelightHelpers.getRawFiducials("limelight-turret")[0].txnc);
      Logger.recordOutput("Theta", theta);
      Logger.recordOutput("Theta1", theta1);
      Logger.recordOutput("Theta2", theta2);
      Logger.recordOutput("DelH", delH);
      Logger.recordOutput("beta", beta);
      Logger.recordOutput("S", S);
      Logger.recordOutput("Sx", Sx);
      Logger.recordOutput("Sy", Sy);
      Logger.recordOutput("L1", L1);
      Logger.recordOutput("L2", L2);
      Logger.recordOutput("thetap1", thetap1);
      Logger.recordOutput("thetap2", thetap2);
      Logger.recordOutput("Turret Current Position", io.getPosition());
      Logger.recordOutput("Turret Target Position", targetPosition);
      Logger.recordOutput("Phi", phi);
      Logger.recordOutput("TY - CM", y1);
      Logger.recordOutput("TY - Left", y2);
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

  public void followCamera(){
    LimelightHelpers.SetFiducialIDFiltersOverride("limelight-turret", TurretConstants.cIds[0]);
    double theta = 0d;
    double delH = TurretConstants.TAG_HEIGHT_METERS-TurretConstants.LIMELIGHT_HEIGHT_METERS;
        double theta1 = 0d;
        double theta2 = 0d;
        double L1 = 0d;
        double L2 = 0d;
        double beta = 0d;
        double phi = 0d;
        double thetap1 = 0d;
        double thetap2 = 0d;
        double Sx = 0d;
        double Sy = 0d;
        double S = 0d;
    if(LimelightHelpers.getTargetCount("limelight-turret")==2){
        
        RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("limelight-turret");
        for(RawFiducial f : fiducials){
          int id = f.id;
          if(id==TurretConstants.cIds[0][0]){
            theta1 = f.txnc;
            this.y1 = f.tync;
            L1 = (delH)/(Math.tan(Math.toRadians(TurretConstants.LIMELIGHT_ANGLE+y1)));
          }
          else{
            this.y2 = f.tync;
            theta2 = f.txnc;
            L2 = (delH)/(Math.tan(Math.toRadians(TurretConstants.LIMELIGHT_ANGLE+y2)));
          }
        }
        beta = Math.toDegrees(Math.asin((L2*Math.sin(Math.toRadians((theta1-theta2))) / TurretConstants.DISTANCE_BETWEEN_TAGS)));
        phi = Math.toDegrees(Math.asin((L1*Math.sin(Math.toRadians((theta1-theta2))) / TurretConstants.DISTANCE_BETWEEN_TAGS)));


        if(phi>90 && beta<90){
          thetap1 = 90 - beta;
          thetap2 = -Math.abs(theta2 - theta1) + thetap1; //Check
        }
        else if(phi<90 && beta>90){
          thetap1 = beta-90;
          thetap2 = -Math.abs(theta1-theta2)+thetap1;
          thetap1 = -thetap1;
          thetap2 = -thetap2;
        }
        else{
          thetap1 = 90-beta;
          thetap2 = Math.abs(theta1)+theta2-thetap1;
        }



        Sx = L2 * Math.sin(Math.toRadians(thetap2)) + TurretConstants.DISTANCE_TAG_SIDE_TO_TARGET * Math.sin(Math.toRadians(90-TurretConstants.alpha));
        Sy = L2 * Math.cos(Math.toRadians(thetap2)) + TurretConstants.DISTANCE_TAG_SIDE_TO_TARGET * Math.cos(Math.toRadians(90-TurretConstants.alpha));
        theta = Math.abs((Math.toDegrees((Math.atan(Sx / Sy))))); // maybe abs
        double oldTheta = theta;
        //theta += -thetap1 + Math.abs(theta1);


        if(theta1>0 && theta2>0){
          if(beta<90 && phi>90){
            theta += Math.abs(-Math.abs(thetap1) + Math.abs(theta1));
          }
          else{
            theta = Math.abs(Math.abs(thetap1)+theta1-Math.abs(theta));
          }
        }
        else if(theta1<0 && theta2<0){
          if(beta>90 && phi<90){
            theta += Math.abs(-Math.abs(thetap1) + Math.abs(theta1));
          }
          else{
            theta = Math.abs(Math.abs(theta1)+Math.abs(thetap1)-Math.abs(theta));
          }
        }
        
        // if((theta1<0 && theta2<0 || theta1>0 && theta2>0)){
        //   theta += Math.abs(-thetap1 + Math.abs(theta1));
        // }
        // else{
        //   theta += theta2-thetap2;
        // }
        S = Math.sqrt(Math.pow(Sx, 2) + Math.pow(Sy, 2));
        // this.x = L1;
        if(theta1<0 && theta2<0){
          theta = -theta;
          System.out.println("somethingcrazythatwouldn'totherwiseoccur");
        }

        // if(oldTheta<theta2-thetap2){
        //   theta = -theta;
        // }
        targetPosition = io.getPosition()+(theta/36);





        //if(2*theta1-theta!=thetap1){
        //   theta = -theta;
        // }
        // theat = theta;
        // theat2 = theta2;
        
        //System.out.println("Theta: "+(theta/10));
        // this.L1 = L1;
        // this.L2 = L2;
        //final double t = theta;
        
        
        //double L1 = (delH)/(Math.tan(TurretConstants.LIMELIGHT_ANGLE+LimelightHelpers.get))
        LimelightHelpers.SetFiducialIDFiltersOverride("limelight-turret", TurretConstants.allId);
        // if((int)theta / Integer.MAX_VALUE!=0){
        //     throw new RuntimeException();
        // }
      }
    this.theta = theta;
    this.delH = delH;
    this.L1 = L1;
    this.L2 = L2;
    this.S = S;
    this.Sx = Sx;
    this.Sy = Sy;
    this.beta = beta;
    this.phi = phi;
    this.theta1 = theta1;
    this.theta2 = theta2;
    this.thetap1 = thetap1;
    this.thetap2 = thetap2;
    
  } 
}