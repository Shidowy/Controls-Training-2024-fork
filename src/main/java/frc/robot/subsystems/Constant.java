// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.DriveConstants.kTeleopMaxAngularSpeedRadiansPerSecond;
import static frc.robot.Constants.DriveConstants.kTeleopMaxSpeedMetersPerSecond;

import java.util.List;
import java.util.Map;
import java.util.TreeMap;
import java.util.Map.*;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import org.opencv.core.Mat;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import frc.util.COTSFalconSwerveConstants;
import frc.util.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 *
 * <p>Include units in Constant names whenever possible/convenient. If not, include a comment
 * designating units.
 */
public final class Constants {
  public static final String kRobotIdFile = "/home/lvuser/id.txt";
  public static final String kCurrentLogFolder = "/home/lvuser/currentLogs";
  public static final double kNominalVoltage = 12.0;
  public static final int kCANTimeoutMs = 250;
  public static final double kTimestepSeconds = 0.02;

  public static final double kRIOLoopTime = 0.02;

  // motor constants
  public static final double kFalconSensorUnitsPerRotation = 2048; // pulses per rotation
  public static final double kFalconWindingsResistanceOhms = 12.0 / 257;
  public static final double kFalconTorquePerAmp = 4.69 / 257;
  public static final double kFalconOutputUnitsFull = 1023;
  public static final double kFalconOutputUnitsPerVolt = kFalconOutputUnitsFull / kNominalVoltage;
  public static final double kFalconFreeSpeedRotationsPerSecond = 6380.0 / 60.0;
  public static final double kSecondsPer100Ms = 0.1;

  public static final double kNeoFreeSpeedRotationsPerSecond = 5676.0 / 60.0;
  public static final double kNeo550FreeSpeedRotationsPerSecond = 11000.0 / 60.0;

  public static final double k775FreeSpeedRotationsPerSecond = 18730.0 / 60.0;

  // 5 ft front bumper: 60 0.8

  public static final double kShootingFenderSetpointShooter = 24;
  public static final double kShootingFenderSetpointTangentialRatio = 0.55;

  public static final class OIConstants {
    public static final int kDriverPort = 0;
    public static final int kOperatorPort = 1;

    public static final double kDriverRightXDeadband = 0.05;
    public static final double kDriverLeftXDeadband = 0.05;
    public static final double kDriverLeftYDeadband = 0.05;

    public static final double kTriggerThreshold = 0.1;
  }

  public static final class AutoConstants {
    public static final double kAutoMaxSpeedMetersPerSecond = 1.5;
    public static final double kAutoMaxAccelerationMetersPerSecondSquared = 2;

    public static final double kAutoMaxAngularSpeedRadiansPerSecond =
        1.5 / DriveConstants.kTurningRadiusMeters;
    public static final double kAutoMaxAngularAccelerationRadiansPerSecondSquared = 1 * Math.PI;

    public static final double kPAutoXController = 5;
    public static final double kIAutoXController = 0;
    public static final double kDAutoXController = 0;
    public static final double kPAutoYController = 5;
    public static final double kIAutoYController = 0;
    public static final double kDAutoYController = 0;

    public static final double kAutoXYIntegratorMaxMetersPerSecond = 0.5;
    public static final double kPAutoThetaController = 1;
    public static final double kIAutoThetaController = 0;
    public static final double kAutoThetaIntegratorMaxRadiansPerSecond = 0.2;
    public static final double kDAutoThetaController = 0;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kAutoThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kAutoMaxAngularSpeedRadiansPerSecond,
            kAutoMaxAngularAccelerationRadiansPerSecondSquared);
  }

  public static final class ModuleConstants {
    // Drive control constants
    public static final double kDriveSupplyCurrentLimitAmps = 35;
    public static final double kDriveStatorCurrentLimitAmps = 60;
    public static final double kDriveStatorCurrentThresholdTimeSecs = 0.1;

    //// Drive mechanism/encoder constants
    public static final double kWheelDiameterMeters = 0.1016;
    public static final double kWheelCircumferenceMeters = 0.1016 * Math.PI;
    public static final double kDriveGearRatio = 6.86; // motor turns per wheel turns
    public static final double kDriveDistMetersPerMotorRev =
        kWheelCircumferenceMeters / kDriveGearRatio;
    public static final double kDriveDistMetersPerPulse =
        kDriveDistMetersPerMotorRev / kFalconSensorUnitsPerRotation;
    //// NOTE: CTRE Encoders return velocity in units/100 ms. CTRE velocity readings should be
    // multiplied by 10 to be per second.
    //// Drive feedforward constants
    // Field Carpet characterization constants
    // public static final double ksDriveVolts = 0.66707;
    // public static final double kvDriveVoltSecondsPerMeter = 2.7887;
    // public static final double kaDriveVoltSecondsSquaredPerMeter = 0.29537;

    // Practice carpet characterization constants
    // public static final double ksDriveVolts = 0.667;
    // public static final double kvDriveVoltSecondsPerMeter = 2.7695;
    // public static final double kaDriveVoltSecondsSquaredPerMeter = 0.23776;

    // practice carpet
    public static final double ksDriveVolts = 0.51019;
    public static final double kvDriveVoltSecondsPerMeter = 2.2644;
    public static final double kaDriveVoltSecondsSquaredPerMeter = 0.1;

    public static final double ksDriveVoltsBackLeft = 0.5;

    public static final double kvMaxVelocity = 12 / kvDriveVoltSecondsPerMeter;
    public static final double kaMaxAcceleration = 12 / kaDriveVoltSecondsSquaredPerMeter;

    //// Drive PID constants
    public static final double kPDrive = 0.1;
    // Turning control constants
    public static final double kTurningSupplyCurrentLimitAmps = 20;
    public static final double kTurningStatorCurrentLimitAmps = 60;
    public static final double kTurningStatorCurrentThresholdTimeSecs = 0.1;

    //// Turning mechanism/encoder constants
    public static final double kTurningGearRatio = 12.8; // motor turns per shaft turns
    public static final double kTurningRadiansPerMotorRev = 2 * Math.PI / kTurningGearRatio;
    public static final double kTurningRadiansPerPulse =
        kTurningRadiansPerMotorRev / kFalconSensorUnitsPerRotation;

    //// Turning feedforward constants
    public static final double ksTurningVolts = 0.60572;
    public static final double kvTurningVoltSecondsPerRadian = 0.20175;
    public static final double kaTurningVoltSecondsSquaredPerRadian = 0.0053;

    //// Turning PID constants
    public static final double kPTurningOutputUnit100MsPerSensorUnit =
        1.5 * kTurningRadiansPerPulse * kFalconOutputUnitsPerVolt / kSecondsPer100Ms;
    public static final double kDTurningOutputUnit100MsSquaredPerSensorUnit =
        0.2 * kTurningRadiansPerPulse * kFalconOutputUnitsPerVolt / kSecondsPer100Ms;
    public static final double kFTurningOutputUnit100MsPerSensorUnit = 0.4 * 1023 / 8360;

    public static final double kTurningPositionToleranceSensorUnits =
        Units.degreesToRadians(2) * kFalconSensorUnitsPerRotation;
    //// Turning trapezoidal motion profile/motion magic constants
    public static final double kModuleMaxSpeedTurningRadiansPerSecond = 8 * Math.PI;
    public static final double kModuleMaxAccelerationTurningRadiansPerSecondSquared = 64 * Math.PI;
    public static final double kModuleMaxSpeedTurningPulsesPer100Ms =
        kModuleMaxSpeedTurningRadiansPerSecond / kTurningRadiansPerPulse * 0.1;
    public static final double kModuleMaxAccelerationTurningPulsesPer100MsSquared =
        kModuleMaxAccelerationTurningRadiansPerSecondSquared / kTurningRadiansPerPulse * 0.01;
  }

  public static final class DriveConstants {

    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(Swerve.widthBetweenModules/ 2, Swerve.lengthBetweenModules / 2),
            new Translation2d(Swerve.lengthBetweenModules / 2, -Swerve.widthBetweenModules / 2),
            new Translation2d(-Swerve.lengthBetweenModules / 2, Swerve.widthBetweenModules / 2),
            new Translation2d(-Swerve.lengthBetweenModules / 2, -Swerve.widthBetweenModules / 2));

    public static final double kTurningRadiusMeters =
        Math.sqrt(Math.pow(Swerve.lengthBetweenModules / 2, 2) + Math.pow(Swerve.widthBetweenModules / 2, 2));

    // Max speed teleoperated
    public static final double kTeleopMaxSpeedMetersPerSecond = 3; // meters per second
    public static final double kTeleopMaxAngularSpeedRadiansPerSecond =
        4; // radians per second

    public static final double kDriveTolerance = .1;

    public static final double kTeleopMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kTeleopMaxAngularAccelerationRadiansPerSecondSquared = 1.5 * Math.PI;

    public static final int kPoseHistoryCapacity = 500;

    public static final double kPRotation = 2;
    public static final double kRotationTolerance = 3;
  }

  public interface FieldConstants {
    public Pose2d getPickupPos();
    public Pose2d getSpeakerPos();
    public Pose2d getSpeakerAnglePos();
    public Pose2d getAmpPos();
    public Pose2d[] getRingPositions();
    public boolean isOnRed();
  }

  public static final class RedFieldConstants implements FieldConstants {
    public Pose2d getPickupPos() { return new Pose2d(10, 2, new Rotation2d()); }
    public Pose2d getSpeakerPos() { return new Pose2d(16.579342, 5.547867999999999, new Rotation2d()); }
    public Pose2d getSpeakerAnglePos() { return new Pose2d(16.559342, 5.547867999999999, new Rotation2d()); }
    public Pose2d getAmpPos() { return new Pose2d(14.700757999999999, 8.5, new Rotation2d()); }
    public Pose2d[] getRingPositions() {
        return new Pose2d[] {
            new Pose2d(13.5, 7, new Rotation2d()),
            new Pose2d(13.5, 5, new Rotation2d()),
        };
    }
    public boolean isOnRed() {return true;}
  }

  public static final class BlueFieldConstants implements FieldConstants {
    public Pose2d getPickupPos() { return new Pose2d(15, 2, new Rotation2d()); }
    public Pose2d getSpeakerPos() { return new Pose2d(-0.038099999999999995, 5.547867999999999, new Rotation2d()); }
    public Pose2d getSpeakerAnglePos() { return new Pose2d(-0.038099999999999995, 5.547867999999999, new Rotation2d()); }
    public Pose2d getAmpPos() { return new Pose2d(1.8415, 8.5, new Rotation2d()); }
    public Pose2d[] getRingPositions() {
        return new Pose2d[] {
            new Pose2d(),
            new Pose2d(),
        };
    }
    public boolean isOnRed() {return false;}
  }

  public static final class VisionConstants {
    public static final String kCameraName = "photonvision";

    // old constraints (might want to use again)

    public static final TrapezoidProfile.Constraints kXConstraints = new TrapezoidProfile.Constraints(1, 2);
    public static final TrapezoidProfile.Constraints kYConstraints = new TrapezoidProfile.Constraints(.5, 2);
    public static final TrapezoidProfile.Constraints kOmegaConstraints = new TrapezoidProfile.Constraints(3, 8);

    public static final double kTranslationTolerance = 0.02;
    public static final double kThetaTolerance = Units.degreesToRadians(0);

    // TODO: tune!
    public static final TrapezoidProfile.Constraints kDefaultXYContraints = new TrapezoidProfile.Constraints(
        Swerve.maxSpeed * 0.3,
        Swerve.maxAngularVelocity);

    public static final TrapezoidProfile.Constraints kDefaultOmegaConstraints = new TrapezoidProfile.Constraints(
        Swerve.maxAngularVelocity * 0.2,
        Swerve.maxAngularVelocity);

    // TODO: tune!
    public static final double X_kP = 1.25;
    public static final double X_kI = 0.3;
    public static final double X_kD = 0.0;

    public static final double Y_kP = 1.25;
    public static final double Y_kI = 0.3;
    public static final double Y_kD = 0.0;  

    public static final double THETA_kP = 1.5;
    public static final double THETA_kI = 0.5;
    public static final double THETA_kD = 0.15;

    // TODO: ensure validity of measurements
    public static final Transform3d kRobotToCamera = new Transform3d(new Translation3d(.2921, 0, 0.24),
        new Rotation3d(0,-.402,0)); 
    //-0.698

    public static final double kFieldLengthMeters = 16.541;
    public static final double kFieldWidthMeters = 8.211;
    public static final List<AprilTag> kBlueTagList = 
                                        List.of(
                                                // new AprilTag(1, new Pose3d(15.079471999999997, 0.24587199999999998, 1.355852, 
                                                //                     new Rotation3d(new Quaternion(0.5000000000000001, 0.0, 0.0, 0.8660254037844386)))),
                                                // new AprilTag(2, new Pose3d(16.185134, 0.883666, 1.355852, 
                                                //                     new Rotation3d(new Quaternion(0.5000000000000001, 0.0, 0.0, 0.8660254037844386)))),
                                                new AprilTag(3, new Pose3d(16.579342, 4.982717999999999, 1.4511020000000001, 
                                                                    new  Rotation3d(new Quaternion(6.123233995736766e-17, 0.0, 0.0, 1.0)))),
                                                new AprilTag(4, new Pose3d(16.579342, 5.547867999999999, 1.4511020000000001, 
                                                                    new Rotation3d(new Quaternion(6.123233995736766e-17, 0.0, 0.0, 1.0)))),                  
                                                // new AprilTag(5, new Pose3d(14.700757999999999, 8.2042, 1.355852, 
                                                //                     new Rotation3d(new Quaternion(-0.7071067811865475, -0.0, 0.0, 0.7071067811865476)))),
                                                // new AprilTag(6, new Pose3d(1.8415, 8.2042, 1.355852, 
                                                //                     new Rotation3d(new Quaternion(-0.7071067811865475, -0.0, 0.0, 0.7071067811865476)))),
                                                new AprilTag(7, new Pose3d(-0.038099999999999995, 5.547867999999999, 1.4511020000000001, 
                                                                    new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0)))),   
                                                new AprilTag(8, new Pose3d(-0.038099999999999995, 4.982717999999999, 1.4511020000000001, 
                                                                    new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0))))   
                                                // new AprilTag(9, new Pose3d(0.356108, 0.883666, 1.355852, 
                                                //                     new Rotation3d(new Quaternion(0.8660254037844387, 0.0, 0.0, 0.49999999999999994)))),
                                                // new AprilTag(10, new Pose3d(1.4615159999999998, 0.2458719999999999, 1.355852, 
                                                //                     new Rotation3d(new Quaternion(0.8660254037844387, 0.0, 0.0, 0.49999999999999994)))),  
                                                // new AprilTag(11, new Pose3d(11.904726, 3.7132259999999997, 1.3208, 
                                                //                     new Rotation3d(new Quaternion(-0.8660254037844387, -0.0, 0.0, 0.49999999999999994)))),
                                                // new AprilTag(12, new Pose3d(11.904726, 4.49834, 1.3208, 
                                                //                     new Rotation3d(new Quaternion(0.8660254037844387, 0.0, 0.0, 0.49999999999999994)))),       
                                                // new AprilTag(13, new Pose3d(11.220196, 4.105148, 1.3208, 
                                                //                     new Rotation3d(new Quaternion(6.123233995736766e-17, 0.0, 0.0, 1.0)))),  
                                                // new AprilTag(14, new Pose3d(5.320792, 4.105148, 1.3208, 
                                                //                     new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0)))),
                                                // new AprilTag(15, new Pose3d(4.641342, 4.49834, 1.3208, 
                                                //                     new Rotation3d(new Quaternion(0.5000000000000001, 0.0, 0.0, 0.8660254037844386)))), 
                                                // new AprilTag(16, new Pose3d(4.641342, 3.71322599999999974, 1.3208, 
                                                //                     new Rotation3d(new Quaternion(-0.4999999999999998, -0.0, 0.0, 0.8660254037844386))))                                                                                                                                              
                                                                    );                                    

  }

  public static final class IntakeConstants {
    public static final int kIntakePort = 14;
    public static final int kIntakeCurrentLimit = 80;
    public static final boolean kIntakeInverted = false;
    public static final double kIntakeKp = .012;
    public static final double kIntakeKi = 0;
    public static final double kIntakeKd = 0;
  }

  public static final class GeneralRollerConstants {
    public static final int kGeneralRollerCurrentLimit = 80;
    public static final int kIndexerPort = 16;
    public static final int kFeederPort = 15;

  }

  public static final class ShooterConstants {

    public static final int kShooterLeftPort = 17;
    public static final int kShooterRightPort = 18;

    public static final double kSShooter = .25;
    public static final double kVShooter = .12;
    public static final double kAShooter = .56;
    public static final double kPShooter = .04*12;
    public static final double kIShooter = .01;
    public static final double kDShooter = 0.01;
    public static final double kSensorToMechanismGearRatio = 36/27;
    public static final double kShooterCruiseVelocity = 80;
    public static final double kShooterAcceleration = 160;
    public static final double kShooterJerk = 1600; 
    public static final double kShooterErrorTolerance = 6;
    public static final boolean kShooterClockwisePositive = false;
    public static final int kCurrentLimit = 80;
  }

  public static final class PivotConstants {
    public static final int kPivotPort = 19;
    public static final double kSPivot = .91;
    public static final double kVPivot = 1.35;
    public static final double kPPivot = 40;
    public static final double kIPivot = 0;
    public static final double kDPivot = .01;
    public static final double kPivotCruiseVelocity = Math.PI*4;
    public static final double kPivotAcceleration = Math.PI;
    public static final double kPivotJerk = Math.PI*80; 
    public static final boolean kPivotClockwisePositive = true;
    public static final double kPivotErrorTolerance = .03;
    public static final double kSensorToMechanismGearRatio = 75;
    public static final double kPivotToRobot = .2921;
    public static final int kCurrentLimit = 40;
  }

  public static final class ClimberConstants {
    public static final int kClimberLeftPort = 21;
    public static final int kClimberRightPort = 20;
    public static final double kSClimberLeft = .91;
    public static final double kSClimberRight = .91;
    public static final double kVClimber = 4;
    public static final double kPClimberLeft = 12;
    public static final double kPClimberRight = 12;
    public static final double kIClimber = 0;
    public static final double kDClimber = 0;
    public static final double kClimberCruiseVelocity = Math.PI*4;
    public static final double kClimberAcceleration = Math.PI*16;
    public static final double kClimberJerk = Math.PI*160; 
    public static final boolean kClimberClockwisePositive = true;
    public static final double kClimberErrorTolerance = .25;
    public static final double kSensorToMechanismGearRatio = 100;
    public static final int kCurrentLimit = 60;
    public static final int kCurrentLimitThreshold = 6;
  }

  public static final class IndexerConstants {
    
  }

  public static final class Swerve {
    public static final int pigeonID = 9;
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

    public static final COTSFalconSwerveConstants chosenModule = // TODO: This must be tuned to specific robot
        COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

    /* Drivetrain Constants */
    public static final double lengthBetweenModules = 0.5842;
    public static final double widthBetweenModules = 0.5842;
    public static final double driveRadius = 0.413091;
    public static final double wheelCircumference = chosenModule.wheelCircumference;

    public static final ReplanningConfig kReplanningConfig = new ReplanningConfig();
    public static final HolonomicPathFollowerConfig kPathFollowingConfig = new HolonomicPathFollowerConfig(// HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(2, 0.0, 0), // Translation PID constants
                        new PIDConstants(2, 0, 0), // Rotation PID constants
                        6, // Max module speed, in m/s
                        driveRadius, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig());

    /*
     * Swerve Kinematics
     * No need to ever change this unless you are not doing a traditional
     * rectangular/square 4 module swerve
     */
    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(Swerve.lengthBetweenModules / 2.0, -Swerve.widthBetweenModules / 2.0),
        new Translation2d(Swerve.lengthBetweenModules / 2.0, Swerve.widthBetweenModules / 2.0),
        new Translation2d(-Swerve.lengthBetweenModules / 2.0, Swerve.widthBetweenModules / 2.0),
        new Translation2d(-Swerve.lengthBetweenModules / 2.0, -Swerve.widthBetweenModules / 2.0));

    
    /* Module Gear Ratios */
    public static final double driveGearRatio = chosenModule.driveGearRatio;
    public static final double angleGearRatio = chosenModule.angleGearRatio;

    /* Motor Inverts */
    public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
    public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = chosenModule.canCoderInvert;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 40;
    public static final int anglePeakCurrentLimit = 80;
    public static final double anglePeakCurrentDuration = 0.1;
    public static final boolean angleEnableCurrentLimit = true;

    public static final int driveContinuousCurrentLimit = 35;
    public static final int drivePeakCurrentLimit = 60;
    public static final double drivePeakCurrentDuration = 0.1;
    public static final boolean driveEnableCurrentLimit = true;

    /*
     * These values are used by the drive falcon to ramp in open loop and closed
     * loop driving.
     * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
     */
    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    /* Angle Motor PID Values */
    // public static final double angleKP = chosenModule.angleKP;
    // public static final double angleKI = chosenModule.angleKI;
    // public static final double angleKD = chosenModule.angleKD;
    // public static final double angleKF = chosenModule.angleKF;

    public static final double angleKP = 30;
    public static final double angleKI = 0;
    public static final double angleKD = .05;
    public static final double angleKF = 0;

    /* Drive Motor PID Values */
    public static final double driveKP = 2.5;
    public static final double driveKI = 3;
    public static final double driveKD = 0.0;
    public static final double driveKF = 0.0;

    /* Drive Motor Characterization Values */
    public static final double driveKS = (0.32 / 12);
    public static final double driveKV = (1.51 / 12);
    public static final double driveKA = (0.27 / 12);

    /* Swerve Profiling Values */
    /** Meters per Second */
    public static final double maxSpeed = 6; // TODO: This must be tuned to specific robot
    /** Radians per Second */
    public static final double maxAngularVelocity = 3; // TODO: This must be tuned to specific robot

    /* Neutral Modes */
    public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Brake;
    public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 { // TODO: This must be tuned to specific robot
      public static final int driveMotorID = 1;
      public static final int angleMotorID = 2;
      public static final int canCoderID = 10;
      public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.36279296875);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset, false);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 { // TODO: This must be tuned to specific robot
      public static final int driveMotorID = 3;
      public static final int angleMotorID = 4;
      public static final int canCoderID = 11;
      public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.42529296875);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset, true);
    }

    /* Back Right Module - Module 2 */
    public static final class Mod2 { // TODO: This must be tuned to specific robot
      public static final int driveMotorID = 5;
      public static final int angleMotorID = 6;
      public static final int canCoderID = 12;
      public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.1552734375);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset, true);
    }

    /* Back Left Module - Module 3- */
    public static final class Mod3 { // TODO: This must be tuned to specific robot
      public static final int driveMotorID = 7;
      public static final int angleMotorID = 8;
      public static final int canCoderID = 13;
      public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.191162109375);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset, false);
    }
  }
}
