package frc.robot.util.Constants;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

// import edu.wpi.first.wpilibj.motorcontrol.Spark;

import com.revrobotics.spark.config.SparkMaxConfig;

public class ClawConstants {

  public static class Roller {

    public static int kRollerID = 13;
    public static MotorType kMotorType = MotorType.kBrushless;
    public static IdleMode kIdleMode = IdleMode.kBrake;
    public static int kCurrentLimit = 40;
    public static double kP = 0.01; // TODO: Configure me!
    public static double kD = 0.0; // TODO: Configure me!
    public static double kVelocityFF = 0.0; // TODO: Configure me!

    public static double kMaxAcceleration = 250;
    public static double kMaxVelocity = 1000;
    public static double kTolerance = 1.75;

    public static double kForwardSoftLimit = 10014;
    public static double kReverseSoftLimit = 0;

    public static double kEncoderOffsetRev = 0; // In revolutions

    public static double kGearRatio = 1;

    public static double kPositionConversionFactor =
        kGearRatio;
            //* 360.0; // (Drum Circumference * Final Gear Ratio) / One Encoder Revolution (if its 1:1
    // with motor shaft)  // TODO: Configure me!
    public static double kVelocityConversionFactor = kPositionConversionFactor / 60.0; // RPM -> MPS

    public static final SparkMaxConfig kRollerConfig = new SparkMaxConfig();
    public static final SparkMaxConfig kAngleConfig = new SparkMaxConfig();

    static {
        kRollerConfig.idleMode(kIdleMode).smartCurrentLimit(kCurrentLimit);

      // kWristConfig
      //     .softLimit
      //     .forwardSoftLimit(kForwardSoftLimit)
      //     .reverseSoftLimit(kReverseSoftLimit);

      kRollerConfig
          .absoluteEncoder
          .positionConversionFactor(kPositionConversionFactor)
          .velocityConversionFactor(kVelocityConversionFactor)
          .zeroOffset(kEncoderOffsetRev);

      // kWristConfig
      //     .closedLoop
      //     .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      //     .pidf(kP, 0.0, kD, kVelocityFF)
      //     .outputRange(-1, 1);

      // kWristConfig
      //     .closedLoop
      //     .maxMotion
      //     .maxVelocity(kMaxVelocity)
      //     .maxAcceleration(kMaxAcceleration)
      //     .allowedClosedLoopError(kTolerance);
    }
  }

  public static class Wrist {
    public static int kMotorID = 12;
    public static MotorType kMotorType = MotorType.kBrushless;
    public static IdleMode kIdleMode = IdleMode.kBrake;
    public static int kCurrentLimit = 80;
    public static double kP = 0.08; // TODO: Configure me!
    public static double kD = 0.0; // TODO: Configure me!
    public static double kVelocityFF = 0.0; // TODO: Configure me!

    public static double kMaxAcceleration = 100;
    public static double kMaxVelocity = 100;
    public static double kTolerance = 1;

    public static double kForwardSoftLimit = 70;
    public static double kReverseSoftLimit = -58;
    public static double kGearRatio = 50.0 / 84.0;

    public static double kEncoderOffsetRev = 0.0623246; // In revolutions

    public static double kPositionConversionFactor = kGearRatio * 360.0;
    public static double kVelocityConversionFactor = kPositionConversionFactor / 60.0; // RPM -> MPS

    public static double kS = 0.0;
    public static double kG = 0.21;
    public static double kV = 2.05;
    public static double kA = 0.0;

    public static final SparkMaxConfig kWristConfig = new SparkMaxConfig();

    public enum ClawRollerVolt {
      INTAKE_CORAL(1),
      INTAKE_ALGAE(1),
      OUTTAKE_REEF(-4),
      OUTTAKE_BARGE(-5);

      public final double voltage;

      private ClawRollerVolt(double voltage) {
        this.voltage = voltage;
      }

      public double get() {
        return this.voltage;
      }
    };

    public enum Positions {
      BOTTOM(0),
      INTAKE(-50),
      L1(0),
      L2(50),
      L3(50),
      L4(65),
      SCORE(25),
      BARGE(0);

      public final double position;

      private Positions(double position) {
        this.position = position;
      }

      public double getPos() {
        return this.position;
      }
    };

    static {
      kWristConfig.idleMode(kIdleMode).smartCurrentLimit(kCurrentLimit);

      // kWristConfig
      //     .softLimit
      //     .forwardSoftLimit(kForwardSoftLimit)
      //     .reverseSoftLimit(kReverseSoftLimit);

      kWristConfig
          .absoluteEncoder
          .positionConversionFactor(kPositionConversionFactor)
          .velocityConversionFactor(kVelocityConversionFactor)
          .zeroOffset(kEncoderOffsetRev);

      // kWristConfig
      //     .closedLoop
      //     .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      //     .pidf(kP, 0.0, kD, kVelocityFF)
      //     .outputRange(-1, 1);

      // kWristConfig
      //     .closedLoop
      //     .maxMotion
      //     .maxVelocity(kMaxVelocity)
      //     .maxAcceleration(kMaxAcceleration)
      //     .allowedClosedLoopError(kTolerance);
    }
  }
}