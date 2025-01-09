package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoderJNI;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Const;
import frc.robot.Const.DriveConstants;
import frc.robot.Const.ModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {

    private final TalonFX driveMotor;
    private final TalonFXConfiguration driveMotor_cfg; 
    private final SparkMax turningMotor;

    //private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final PIDController turningPidController;

    //private final AnalogInput absoluteEncoder;
    private final CANcoder SteerCANcoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int CANcoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        
        //Switch over to absolute encoder from analog input to CANCoder 
        //absoluteEncoder = new AnalogInput(absoluteEncoderId);

        SteerCANcoder = new CANcoder(CANcoderId);

        driveMotor_cfg = new TalonFXConfiguration();
        driveMotor_cfg.Feedback.FeedbackRemoteSensorID = CANcoderId;
        driveMotor_cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;


        // driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
        driveMotor = new TalonFX(driveMotorId, Const.CANivore);
        driveMotor.getConfigurator().apply(driveMotor_cfg);

        turningMotor = new SparkMax(turningMotorId, MotorType.kBrushless);

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        
        turningEncoder = turningMotor.getEncoder();

        // NOTE: These mothods don't seem to exist any more, not sure what this will screw up
        //driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        //driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        //turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        //turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerS ec);

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    public double getDrivePosition() {
        //return driveEncoder.getPosition();
        return driveMotor.getPosition().getValueAsDouble();
    }

    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return driveMotor.getVelocity().getValueAsDouble();
    }

    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

    public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        driveMotor.getPosition().getValueAsDouble(), new Rotation2d(turningEncoder.getPosition()));
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

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, new Rotation2d(getTurningPosition()));
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
        SmartDashboard.putString("Swerve[" + SteerCANcoder.getDeviceID() + "] state", state.toString());
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }
}
