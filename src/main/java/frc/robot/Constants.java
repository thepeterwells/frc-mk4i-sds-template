package frc.robot;

public class Constants {
    // CAN IDs for all swerve motor controllers
    public static int frontRightSpeedCANID = 1;
    public static int frontRightAngleCANID = 5;

    public static int frontLeftSpeedCANID = 2;
    public static int frontLeftAngleCANID = 6;

    public static int backLeftSpeedCANID = 3;
    public static int backLeftAngleCANID = 7;

    public static int backRightSpeedCANID = 4;
    public static int backRightAngleCANID = 8;

    public static double driveFeedForward = 0.000175;
    public static double driveProportional = 0.00001;
    public static double driveIntegral = 0.0000004;
    public static double driveDerivative = 0;

    public static double steerFeedForward = 0;
    public static double steerProportional = 8;
    public static double steerIntegral = 0.01;
    public static double steerDerivative = 0.01;
}
