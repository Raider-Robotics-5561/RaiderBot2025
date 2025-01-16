package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Const;
import frc.robot.Const.ModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
    // TODO: Clean up this file and make it more readable.
    private final TalonFX driveMotor;
    private final TalonFXConfiguration driveMotor_cfg; 

    private final SparkMax turningMotor;
    private final RelativeEncoder turningEncoder;

    private final PIDController turningPidController;

    //private final AnalogInput absoluteEncoder;
    public final CANcoder SteerCANcoder;
    // private final CANcoder DriveCANcoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;
    // public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr; - For Neo if we decide to use revs PIDcontroller

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int CANcoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        
        //Switch over to absolute encoder from analog input to CANCoder 
        //absoluteEncoder = new AnalogInput(absoluteEncoderId);
        
        SteerCANcoder = new CANcoder(CANcoderId);
        // DriveCANcoder = new CANcoder(driveMotorId);

        // SECTION: TALON FX CONFIGURATION.

        driveMotor = new TalonFX(driveMotorId, Const.CANivore);

        driveMotor_cfg = new TalonFXConfiguration();
        // driveMotor_cfg.Feedback.FeedbackRemoteSensorID = driveMotorId;
        var slot0Configs = driveMotor_cfg.Slot0;
        slot0Configs.kS = 0.05; // Add 0.05 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kP = Const.DriveConstants.DrivePID.kP; // An error of 1 rps results in 0.11 V output
        slot0Configs.kI = Const.DriveConstants.DrivePID.kI; // no output for integrated error
        slot0Configs.kD = Const.DriveConstants.DrivePID.kD; // no output for error derivative

        driveMotor_cfg.TorqueCurrent.PeakForwardTorqueCurrent = 40;
        driveMotor_cfg.TorqueCurrent.PeakReverseTorqueCurrent = -40;

        driveMotor_cfg.Voltage.PeakForwardVoltage = 8;
        driveMotor_cfg.Voltage.PeakReverseVoltage = -8;
        driveMotor.getConfigurator().apply(driveMotor_cfg);

        // ENDSECTION

        turningMotor = new SparkMax(turningMotorId, MotorType.kBrushless);

        // TODO Replace below
        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        turningEncoder = turningMotor.getEncoder();

        // NOTE: These mothods don't seem to exist any more, not sure what this will screw up
        //driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        //driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        //turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        //turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerS ec);

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0.001, 0.004, 1);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    public double sensorRotationToMeters(double sensorvalue){  
        return sensorvalue / 51.9;// 6.5765;
    }
    
    public double getDrivePosition() { 
        return driveMotor.getRotorPosition().refresh().getValueAsDouble() / 51.9;
    }

    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }

    public double getDriveVelocity() {
        // NOTE: This gets the original value which is in RPS and converts it to RPM.
        return (driveMotor.getRotorVelocity().refresh().getValueAsDouble()) / 60; //Convert to Real RPM
    }

    public double getTurningVelocity() {
        System.out.println("Turning Velocity: " + turningMotor.getDeviceId() + " | " + turningMotor.getEncoder().getVelocity());
        return turningMotor.getEncoder().getVelocity();
    }

    public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        driveMotor.getRotorPosition().refresh().getValueAsDouble(), new Rotation2d(turningEncoder.getPosition()));
    }

    public double getAbsoluteEncoderRad() {
        double angle = SteerCANcoder.getPosition().getValueAsDouble();
        SmartDashboard.putString("Swerve[" + SteerCANcoder.getDeviceID() + "] angle", String.valueOf(angle));
        angle *= 2.0 * Math.PI;
        angle -= absoluteEncoderOffsetRad;
        SmartDashboard.putString("Swerve[" + SteerCANcoder.getDeviceID() + "] angle Calced", String.valueOf(angle * (absoluteEncoderReversed ? -1.0 : 1.0)));
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);   
    }

    public void resetEncoders() {
        driveMotor.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
        
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        SmartDashboard.putNumber("Swerve Raw Power[" + driveMotor.getDeviceID() + "]", driveMotor.get());
        state.optimize(new Rotation2d(getTurningPosition()));
        driveMotor.set(state.speedMetersPerSecond);
        if (state.speedMetersPerSecond >= 0.1) {
            driveMotor.set(0.1);
        }
        else{
            driveMotor.set(state.speedMetersPerSecond);
        }

        if (state.speedMetersPerSecond <= -0.1) {
            driveMotor.set(-0.1);
        }
        else{
            driveMotor.set(state.speedMetersPerSecond);
        }

        System.out.println("State.ange.getRadians(): " + state.angle.getRadians());

        double turningpidout = turningPidController.calculate(getTurningPosition(), state.angle.getRadians());

        System.out.println("turningpidout:" + turningpidout);
        turningMotor.set(turningpidout);
        SmartDashboard.putString("Swerve[" + SteerCANcoder.getDeviceID() + "] state", state.toString());
        SmartDashboard.putNumber("SwerveDrive Drive Velocity[" + driveMotor.getDeviceID() + "] state", driveMotor.getRotorVelocity().refresh().getValueAsDouble());
        SmartDashboard.putNumber("SwerveDrive Drive Position[" + driveMotor.getDeviceID() + "] state", driveMotor.getRotorPosition().refresh().getValueAsDouble());
        SmartDashboard.putNumber("SwerveDrive Steer Velocity[" + turningMotor.getDeviceId() + "] state", getTurningVelocity());
        SmartDashboard.putNumber("SwerveDrive Steer Position[" + turningMotor.getDeviceId() + "] state", getTurningPosition());
        System.out.println("Drive Velocity: " + driveMotor.getDeviceID() + " | " + driveMotor.getRotorVelocity().refresh().getValueAsDouble());
        
        
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }
}
