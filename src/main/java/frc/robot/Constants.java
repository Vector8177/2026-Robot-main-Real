// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.RobotBase;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Set;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class Dimensions{
    public static final Distance ROBOT_WIDTH = Inches.of(34);
    public static final Distance ROBOT_LENGTH = Inches.of(34);
    public static final Distance BUMPER_HEIGHT = Inches.of(4.5);

    public static final Distance INTAKE_OFFSET_X = Inches.of(17);
    public static final Distance INTAKE_BOUNDING_Y = Inches.of(26.5);
    public static final Distance INTAKE_BOUNDING_X = Inches.of(8.097393);

    public static final int MAX_FUEL = 35;
  }

  public static final class ElevatorConstants {
    public static final int LEFT_MOTOR_ID = 31; // lead motor
    public static final int RIGHT_MOTOR_ID = 33;

    public static final double INTAKE = -5; // -7.9
    public static final double L1 = -10;
    public static final double L2 = -6.5; // 31
    public static final double L3 = 27; // 29
    public static final double L4 = 77; // 103
    public static final double NET = 105; // 103.5
    // above is coral, below is algea

    public static final double L3_ALGAE = 83;
    public static final double L1_ALGAE = 90;
    public static final double L2_ALGAE = 53;
    public static final double BARGE = 102;
    public static final int MAX_VOLTAGE = 9; // 12 og

    public static double kP = 2.5;
    public static double kI = 0.0;
    public static double kD = 0.1;

    public static double kS = 0.11237;
    public static double kV = 0.56387;
    public static double kA = 0.041488;
    public static double kG = 0.76416;
  }

  public static final class IntakeConstants {
    public static final int MOTOR_ID = 31;
    public static final int MAX_VOLTAGE = 12;
    public static final DCMotor SIM_INTAKE_MOTOR = DCMotor.getKrakenX60(1);
  }

  public static final class IntakePivotConstants {
    public static final int MOTOR_ID = 32;
    public static final int MAX_VOLTAGE = 12;


    //DEFINITELY CHANGE, THIS IS A GUESS!!
    public static final double INTAKE_POSITION = Degrees.of(-90).in(Radians);


    //tune pid and svag later
    public static final double kP = 10; // 1.5
    public static final double kI = 0;
    public static final double kD = 1;

    public static final double kS = 0.11237;
    public static final double kV = 0.56387;
    public static final double kA = 0.041488;
    public static final double kG = 0.76416;

    // sim constants
    public static final DCMotor SIM_INTAKE_ARM_MOTOR = DCMotor.getKrakenX60(1); //placeholder
    public static final double GEARING = 2; //placeholder
    public static final Distance INTAKE_ARM_LENGTH = Inches.of(12.935416);
    public static final Mass INTAKE_ARM_MASS = Pounds.of(11.638);
    public static final Translation3d INTAKE_PIVOT = new Translation3d(0.3048, 0, 0.1482902);
    public static final Angle INTAKE_ARM_STARTING_ANGLE = Degrees.of(90); //yaw of intake pivot when the robot first starts relative to the world's coordinate system
    public static final Angle INTAKE_ARM_EXPORTED_OFFSET_FROM_HORIZONTAL = Degrees.of(-11.838585);
  }
  public static final class ShooterConstants {
    public static final AngularVelocity SHOOTER_SPEED = RotationsPerSecond.of(10);
    public static final DCMotor SHOOTER_MOTOR = DCMotor.getKrakenX60(2);
    public static final double SHOOTER_GEARING = 26.0/15.0; //output to input
    public static final double FLYWHEEL_TO_LAUNCHER_GEARING = 1.0/1.0; //TODO: placeholder
    public static final Distance SHOOTER_WHEEL_RADIUS = Inches.of(3); //TODO: placeholder
    public static final Translation3d SHOOTER_PIVOT = new Translation3d(-0.168275, 0, 0.4300016);
  }

  public static final class TurretConstants {

    public static final int MOTOR_ID = 21;
    public static final int MAX_VOLTAGE = 12;

    public static final double LIMELIGHT_ANGLE = 33; //WILL NEED CHANGE
    public static final double LIMELIGHT_HEIGHT_METERS = 15.25; //WILL NEED CHANGE
    public static final double TAG_HEIGHT_METERS = 44; //WILL NEED CHANGE
    public static final double DISTANCE_TAG_TO_TARGET = 23.75;
    public static final double DISTANCE_TAG_SIDE_TO_TARGET = 27.56923104d;
    public static final double DISTANCE_BETWEEN_TAGS = 14d;
    public static final double alpha = Math.abs(Math.toDegrees(Math.acos(DISTANCE_BETWEEN_TAGS / DISTANCE_TAG_SIDE_TO_TARGET)));
    public static final int[][] cIds = new int[][]{{10, 9}};//change later
    public static final int[] allId = new int[] {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20,
        21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32};//change later



    //tune pid and svag later
    public static final double y_kP = 1.5; // 1.5
    public static final double y_kI = 0.0;
    public static final double y_kD = 0.1;

    public static final double z_kP = 1.5; // 1.5
    public static final double z_kI = 0.0;
    public static final double z_kD = 0.1;


    public static final double kS = 0.11237;
    public static final double kV = 0.56387;
    public static final double kA = 0.041488;
    public static final double kG = 0.76416;


    //Most likely not needed later
    public static final double posO = 0d;
    public static final double pos1 = 5d;
    public static final double pos2 = 10d;
    public static final double pos3 = 20d;

    public static final DCMotor TURRET_Y_MOTOR = DCMotor.getKrakenX44(1);
    public static final double TURRET_Y_GEARING = (12.0 / 44.0) * (18.0 / 48.0) * (18.0 / 18.0) * (15.0 / 15.0) * (10.0 / 185.0);
    public static final DCMotor TURRET_Z_MOTOR = DCMotor.getKrakenX44(1);
    public static final double TURRET_Z_GEARING = (12.0 / 40.0) * (26.0 / 40.0) * (10.0 / 77.0);
  }

  public static final class WristConstants {
    public static final int MOTOR_ID = 41;
    public static final int MAX_VOLTAGE = 9;

    // These are relative encoder positions - comments are for absolute encoder -

    // relative encoders were negative, shantanu made them positive on 9/4/25

    public static final double INTAKE_POSITION = 10.25; // 10.25
    public static final double SCORING_POSITION_L1 = 5; // .35
    public static final double SCORING_POSITION_L2 = .1; // .25
    public static final double SCORING_POSITION_L4 = .1; // .34
    public static final double PERPENDICULAR_POSITION = 6.5; // .6
    public static final double SCORING_POSITION_NET = -7.8; // .65
    public static final double FLICK_WRIST_POSITION = -7.5; // .525

    // THIS VARIABLE IS NOT ACCURATE, PLZ CHANGE AFTER TESTING
    public static final double L2ALGAE = 12; //
    public static final double L3ALGEA = 12;
    public static final double SCORING_POSITION_BARGE = 3;
    public static double kP = 1.75; // 1.5
    public static double kI = 0.0;
    public static double kD = 0;

    public static double kS = 0.11237;
    public static double kV = 0.56387;
    public static double kA = 0.041488;
    public static double kG = 0.76416;
  }

  public static final class VisionConstants {
    // public static final double alignSpeed = -.5;
    // public static final double alignRange = 5;
    // public static final double kP = .04;

    // AprilTag layout
    public static AprilTagFieldLayout aprilTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

    // Camera names, must match names configured on coprocessor
    public static String camera0Name = "limelight-left"; // this is the right LL
    public static String camera1Name = "limelight-right"; // this is the left LL

    public static final double LIMELIGHT_HEIGHT_METERS = 0.65;
    public static final double LIMELIGHT_PITCH_DEG = 25.0;
    // Example target tags
    public static final Set<Integer> TARGET_TAG_IDS = Set.of(21, 22, 7);

    // Basic filtering thresholds
    public static double maxAmbiguity = 0.3;
    public static double maxZError = .75;

    // Standard deviation baselines, for 1-meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double linearStdDevBaseline = 0.02; // Meters
    public static double angularStdDevBaseline = 0.06; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] cameraStdDevFactors =
        new double[] {
          1.0, // Camera 0
          1.0 // Camera 1
        };

    // Multipliers to apply for MegaTag 2 observations
    public static double linearStdDevMegatag2Factor = .5; // More stable than full 3D solve .5
    public static double angularStdDevMegatag2Factor =
        Double.POSITIVE_INFINITY; // No rotation data available

    public static double maxAvgTagDistance = 3; // was 2.0// change this later

    // Boolean for Left/Right Reef
    public static boolean k_isRightReef = true;

    // Boolean for Committing to Shoot
    public static boolean k_positioned = true;

    // PID for Tag Relative Control for Scoring
    public static final double kP_aim = 0.15; // .1
    public static final double kI_aim = 0.0;
    public static final double kD_aim = 0.0;

    public static final double kP_range = 0.75; // .6
    public static final double kI_range = 0.0;
    public static final double kD_range = 0.0;

    public static final double kP_strafe = 0.75; // .6
    public static final double kI_strafe = 0.0;
    public static final double kD_strafe = 0.0;

    // AimNRange Reef Right
    public static final double k_aimReefRightTarget = 2.3; // 2.5
    public static final double k_rangeReefRightTarget = -0.45; // -.4
    public static final double k_strafeReefRightTarget = 0.19; // .18

    // AimNRange Reef Left
    public static final double k_aimReefLeftTarget = 4.4; // 4.6
    public static final double k_rangeReefLeftTarget = -.45; // -.45
    public static final double k_strafeReefLeftTarget = -0.32; // -.25

    // Prerequisites

    public static final double k_tzValidRange = -1.5;
    public static final double k_yawValidRange = 35;

    // Thresholds
    public static final double k_rangeThreshold = 0.02;
    public static final double k_strafeThreshold = 0.02;
    public static final double k_aimThreshold = 0.5;

    // For testing
    public static boolean k_positioning = false;
  }

  public static final class ClimberConstants {
    public static final int RIGHT_MOTOR_ID = 52;
    public static final int LEFT_MOTOR_ID = 51;

    public static final int MAX_VOLTAGE = 12;
  }
}
