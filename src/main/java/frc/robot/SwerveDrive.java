package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveDrive {
    
    // Distance from center for wheels. Currently assumes rectangulare wheel arrangement
    private double frontWheelPosition = 0.2604;
    private double backWheelPosition = -0.2604;
    private double rightWheelPosition = -0.2786;
    private double leftWheelPosition = 0.2786;

    private WheelDrive backRight;
    private WheelDrive backLeft;
    private WheelDrive frontRight;
    private WheelDrive frontLeft;

    // Kinematics
    private SwerveDriveKinematics swerveKinematics;
    private ChassisSpeeds chassisSpeeds;
    

    public SwerveDrive(WheelDrive backRight, WheelDrive backLeft, WheelDrive frontRight, WheelDrive frontLeft) {
        this.backRight = backRight;
        this.backLeft = backLeft;
        this.frontRight = frontRight;
        this.frontLeft = frontLeft;

        Translation2d frontRightLocation = new Translation2d(frontWheelPosition, rightWheelPosition);
        Translation2d frontLeftLocation = new Translation2d(frontWheelPosition, leftWheelPosition);
        Translation2d backLeftLocation = new Translation2d(backWheelPosition, leftWheelPosition);
        Translation2d backRightLocation = new Translation2d(backWheelPosition, rightWheelPosition);

        swerveKinematics = new SwerveDriveKinematics(frontRightLocation, frontLeftLocation, backLeftLocation, backRightLocation);
    }

    public void driveWithKinematics(double leftStickX, double leftStickY, double rightStickX) {
        // The commented out values are copied from last year's
        // double xAxisSpeed = Math.copySign(Math.pow(leftStickX, 2.0), leftStickX);
        // double yAxisSpeed = -Math.copySign(Math.pow(leftStickY, 2.0), leftStickY);
        // double spinSpeed = Math.copySign(Math.pow(rightStickX, 2.0), rightStickX);

        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(-leftStickY, leftStickX, rightStickX, new Rotation2d(0));

        SwerveModuleState[] moduleStates = swerveKinematics.toSwerveModuleStates(chassisSpeeds);
        frontLeft.driveKinematics(moduleStates[0]);
        frontRight.driveKinematics(moduleStates[1]);
        backRight.driveKinematics(moduleStates[2]);
        backLeft.driveKinematics(moduleStates[3]);
    }
}
