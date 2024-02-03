package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class subswerve extends SubsystemBase {

    public final modswerve frontLeft = new modswerve(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed); 

    public final modswerve frontRight = new modswerve(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed); 

    public final modswerve backLeft = new modswerve(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed); 

    public final modswerve backRight = new modswerve(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed);  


    SwerveModulePosition[] positions = {frontLeft.gSwerveModulePosition(),
        frontRight.gSwerveModulePosition(),backLeft.gSwerveModulePosition(),backRight.gSwerveModulePosition()};
    
    /*public SwerveModulePosition[] SwerveModulePosition(){
        return new SwerveModulePosition[]{
            frontLeft.gSwerveModulePosition(), frontRight.gSwerveModulePosition(), backLeft.gSwerveModulePosition(), backRight.gSwerveModulePosition(),
                };
            }    */

    private AHRS gyro = new AHRS(SPI.Port.kMXP);

    SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, getRotation2d(), positions, new Pose2d(0, 0, getRotation2d()));
   /*  
    private final static SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
           gyro.getRotation2d()); */
 /*
    private static final edu.wpi.first.math.kinematics.SwerveModulePosition[] positions = null; */

    public subswerve() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose2d) {
        odometer.resetPosition(getRotation2d(), positions, getPose());
    }

    public Pose2d getPose2d(){
        return odometer.getPoseMeters();
    }

    @Override
    public void periodic() {
        
        positions[0] = frontLeft.gSwerveModulePosition();
        positions[1] = frontRight.gSwerveModulePosition();
        positions[2] = backLeft.gSwerveModulePosition();
        positions[3] = backRight.gSwerveModulePosition(); 

        odometer.update(getRotation2d(), positions); 
        
        SmartDashboard.putNumber("Robot Angle", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
        SmartDashboard.putNumber("pose x", odometer.getPoseMeters().getX());
        SmartDashboard.putNumber("pose y", odometer.getPoseMeters().getY());
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();  
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);  
   }
    
}
