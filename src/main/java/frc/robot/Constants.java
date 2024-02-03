// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {
  public static final class ModuleConstants {
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1 / 0.4444;
        public static final double kTurningMotorGearRatio = 1 / 0.2273;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 0.5;
  }

  public static final class DriveConstants {

    public static final double kTrackWidth = Units.inchesToMeters(18.89147); //23.25189
    // Distance between right and left wheels
    public static final double kWheelBase = Units.inchesToMeters(23.25189); //18.89147
    // Distance between front and back wheels

    //Plano cartesiano con respecto al centro del robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2), //front left (back left)   
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),  //front right (es front left)
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2), //back left (back right)
            new Translation2d(kWheelBase / 2, kTrackWidth / 2)); //back right (front right)


        //Mover a CAN
    public static final int kFrontLeftDriveMotorPort = 2;
    public static final int kBackLeftDriveMotorPort = 7;
    public static final int kFrontRightDriveMotorPort = 3;
    public static final int kBackRightDriveMotorPort = 6;

    public static final int kFrontLeftTurningMotorPort = 1;
    public static final int kBackLeftTurningMotorPort = 8;
    public static final int kFrontRightTurningMotorPort = 4;
    public static final int kBackRightTurningMotorPort = 5;

    //Uno del drive otro del turning
    public static final boolean kFrontLeftTurningEncoderReversed = true;
    public static final boolean kBackLeftTurningEncoderReversed = true;
    public static final boolean kFrontRightTurningEncoderReversed = true;
    public static final boolean kBackRightTurningEncoderReversed = true;

    public static final boolean kFrontLeftDriveEncoderReversed = true;
    public static final boolean kBackLeftDriveEncoderReversed = true;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kBackRightDriveEncoderReversed = false;


//MOVER LOS ENCODERS

    public static final int kFrontLeftDriveAbsoluteEncoderPort = 1;
    public static final int kBackLeftDriveAbsoluteEncoderPort = 2;
    public static final int kFrontRightDriveAbsoluteEncoderPort = 0;
    public static final int kBackRightDriveAbsoluteEncoderPort = 3;

    public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
    public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
    public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
    public static final boolean kBackRightDriveAbsoluteEncoderReversed = false; 
    
//Dependiendo de su posición, va a poner estos valores---- SE CAMBIA
 
    public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = -4.0236;
    public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = -5.8766;
    public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = -1.1520;
    public static final double kBackRightDriveAbsoluteEncoderOffsetRad = -3.0971;
    

    public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
            kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 0.5;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 0.5;
}
  public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kMaxAngularSpeedRadiansPerSecond = //
                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 1.5;
        public static final double kPYController = 1.5;
        public static final double kPThetaController = 3;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }
    public static final class OIConstants {
      public static final int kDriverControllerPort = 0;

      public static final int kDriverYAxis = 1;
      public static final int kDriverXAxis = 0;
      public static final int kDriverRotAxis = 4;
      public static final int kDriverFieldOrientedButtonIdx = 1;

      public static final double kDeadband = 0.05;
  }
}
