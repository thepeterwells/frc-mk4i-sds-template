package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CAN;

public class WheelDrive {
    private TalonSRX angleMotor;
    private CANSparkMax speedMotor;
    private SparkMaxPIDController speedPIDController;
    private RelativeEncoder speedEncoder;
    private SwerveModuleState wheelSwerveModuleState;

    public WheelDrive (int angleMotor, int speedMotor) {
        this.angleMotor = new TalonSRX(angleMotor);
        this.speedMotor = new CANSparkMax(speedMotor, MotorType.kBrushless);

        this.angleMotor.setNeutralMode(NeutralMode.Brake);
        this.angleMotor.configClosedloopRamp(0);
        this.angleMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog);

        speedEncoder = this.speedMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
        speedEncoder.setPosition(0);
        
        speedPIDController = this.speedMotor.getPIDController();
        speedPIDController.setFeedbackDevice(speedEncoder);


        this.angleMotor.configClosedLoopPeakOutput(0, 1);
        speedPIDController.setOutputRange(-1, 1);

        updatePIDValues(Constants.driveFeedForward, Constants.driveProportional,
        Constants.driveIntegral, Constants.driveDerivative, Constants.steerFeedForward,
        Constants.steerProportional, Constants.steerIntegral, Constants.steerDerivative);
    }

    public void updatePIDValues(double DFF, double DP, double DI, double DD, double SFF, double SP, double SI, double SD) {
        speedPIDController.setFF(DFF);
        speedPIDController.setP(DP);
        speedPIDController.setI(DI);
        speedPIDController.setD(DD);
        angleMotor.config_kF(0, SFF);
        angleMotor.config_kP(0, SP);
        angleMotor.config_kI(0, SI);
        angleMotor.config_kD(0, SD);
      }

    public void driveKinematics(SwerveModuleState swerveModuleState) {
        wheelSwerveModuleState = swerveModuleState;

        //The following will handle the "wrap" from 360 -> 0 for the rotation of the Talon
        double currentAngle = angleMotor.getSelectedSensorPosition() / 1024;
        double fullRotationCount = Math.floor(currentAngle);
        double angleInRadians = ((2 * Math.PI) - ((2 * Math.PI) * (currentAngle - fullRotationCount)));

        // Optimize rotation positions, so the wheels don't turn 180 degrees rather than just spinning the drive motor backwards
        // Determine if the distance between the desired angle and the current angle is less than or equal to 90
        // This is to determine whether the drive motors should be driven forward or backward.
        // If the difference between the desired and current positions is less than or equal to 90 degrees, then...
        if ((Math.abs(wheelSwerveModuleState.angle.getRadians() - angleInRadians) <= (Math.PI / 2)) || (Math.abs(wheelSwerveModuleState.angle.getRadians() - angleInRadians) >= ((3 * Math.PI) / 2))) {
            // If the wheel would have to cross into a new rotation to travel the shortest distance to the desired angle, then...
            if (Math.abs(wheelSwerveModuleState.angle.getRadians() - angleInRadians) >= ((3 * Math.PI) / 2)) {
                // If the difference is positive, then...
                if (wheelSwerveModuleState.angle.getRadians() > angleInRadians) {
                    // Subtract 2pi from the desired angle to show the PID controller later that the shortest distance is to cross 0
                    wheelSwerveModuleState = new SwerveModuleState(wheelSwerveModuleState.speedMetersPerSecond, new Rotation2d(wheelSwerveModuleState.angle.getRadians() - (2 * Math.PI)));
                }
                // If the difference is negative, then...
                else {
                    // Add 2pi to the desired angle to show the PID controller later that the shortest distance is to cross 2pi
                    wheelSwerveModuleState = new SwerveModuleState(wheelSwerveModuleState.speedMetersPerSecond, new Rotation2d(wheelSwerveModuleState.angle.getRadians() + (2 * Math.PI)));
                }
            }
    
        }
        // If the difference between the desired and current positions is greater than 90 degrees, then...
        else {
            // If the difference is positive, then...
            if (wheelSwerveModuleState.angle.getRadians() > angleInRadians) {
                // Invert the drive motor output, and flip the desired angle by subtracting pi
                wheelSwerveModuleState = new SwerveModuleState(-wheelSwerveModuleState.speedMetersPerSecond, new Rotation2d(wheelSwerveModuleState.angle.getRadians() - Math.PI));
            }  else {
                // Invert the drive motor output, and flip the desired angle by adding pi
                wheelSwerveModuleState = new SwerveModuleState(-wheelSwerveModuleState.speedMetersPerSecond, new Rotation2d(wheelSwerveModuleState.angle.getRadians() + Math.PI));
            }
        }

        wheelSwerveModuleState = new SwerveModuleState(wheelSwerveModuleState.speedMetersPerSecond, new Rotation2d(((2 * Math.PI) - wheelSwerveModuleState.angle.getRadians()) + (fullRotationCount * (2 * Math.PI))));

        if (wheelSwerveModuleState.speedMetersPerSecond != 0) {
            angleMotor.set(ControlMode.Position, (wheelSwerveModuleState.angle.getDegrees() / 360.0) * 1024);
        }

        speedPIDController.setReference((((wheelSwerveModuleState.speedMetersPerSecond / ((4 / 39.37) * Math.PI)) * 60) / .15), ControlType.kVelocity);
   
    }
}
