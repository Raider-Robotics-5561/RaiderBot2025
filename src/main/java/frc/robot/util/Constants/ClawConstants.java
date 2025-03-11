package frc.robot.util.Constants;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ClawConstants {

  public static class Roller {

    public static int kRollerID = 13;
    public static MotorType kMotorType = MotorType.kBrushless;
    public static IdleMode kIdleMode = IdleMode.kBrake;
    public static int kCurrentLimit = 40;
    public static double kGearRatio = 1;

    public static final SparkMaxConfig kRollerConfig = new SparkMaxConfig();
    public static final SparkMaxConfig kAngleConfig = new SparkMaxConfig();

    static {
        kRollerConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(kCurrentLimit);
        //NOTE - This may be somehting to look into in the future!
        // .voltageCompensation(10)
        // .openLoopRampRate(0.5);

        kRollerConfig
          .softLimit
          .forwardSoftLimitEnabled(false)
          .reverseSoftLimitEnabled(false);

      kRollerConfig
        .limitSwitch
        .forwardLimitSwitchEnabled(false)
        .reverseLimitSwitchEnabled(false);


    }
  }

  public static class Wrist {
    public static int kMotorID = 12;
    public static MotorType kMotorType = MotorType.kBrushless;
    public static IdleMode kIdleMode = IdleMode.kBrake;
    public static int kCurrentLimit = 20;
    public static double kP = 0.1; 
    public static double kD = 0.0; 
    public static double kVelocityFF = 0.0; 

    public static double kForwardSoftLimit = 2.5;
    public static double kReverseSoftLimit = 7.2;
    public static double kGearRatio = 1 / 49.0;

    public static double kEncoderOffsetRev = 0.0623246; // In revolutions

    public static double kPositionConversionFactor = kGearRatio * 360.0;
    public static double kVelocityConversionFactor = kPositionConversionFactor / 60.0; // RPM -> MPS


    public static final SparkMaxConfig kWristConfig = new SparkMaxConfig();

    public static final double WristPosElevatorSafeyThreshTop = 6.19;
    public static final double WristPosElevatorSafeyThreshBottom = 4.30;

    public static final double WristElevatorPosClearHightTop = 16;
    public static final double WristElevatorPosClearHightBottom = 27;

    public static final double WristAngleLimit = 4.9;
    public static final double WristAngleLimitHight = 36.9; 

    public enum ClawRollerVolt {
      STOPPED(0),
      INTAKE_CORAL(0.5),
      INTAKE_ALGAE(0.5),
      OUTTAKE_REEF(-0.5),
      OUTTAKE_BARGE(-0.5);

      public final double voltage;

      private ClawRollerVolt(double voltage) {
        this.voltage = voltage;
      }

      public double get() {
        return this.voltage;
      }
    };

    public enum WristPositions {
      ZERO(6.79),
      Home(3.03),
      Floor(2.84),
      Intake(6.73),
      L1_L2_Coral(6.21),
      Auto(5.6),
      Intake2(2.73),
      Algae_Drive(3.06),
      Max(5.8),
      
      Elevator_Threh(WristPosElevatorSafeyThreshBottom);


      public final double position;

      private WristPositions(double position) {
        this.position = position;
      }

      public double getPos() {
        return this.position;
      }
    };

    static {
      kWristConfig
      .idleMode(kIdleMode)
      .smartCurrentLimit(kCurrentLimit);

      // kWristConfig
      //     .softLimit
      //     .forwardSoftLimit(kForwardSoftLimit)
      //     .reverseSoftLimit(kReverseSoftLimit);

      kWristConfig
          .absoluteEncoder
          .positionConversionFactor(kPositionConversionFactor)
          .velocityConversionFactor(kVelocityConversionFactor)
          .zeroOffset(kEncoderOffsetRev);

      kWristConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
          .pidf(kP, 0.0, kD, kVelocityFF)
          .outputRange(-0.9, 0.9);


    }
  }
}