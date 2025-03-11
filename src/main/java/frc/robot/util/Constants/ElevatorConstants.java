package frc.robot.util.Constants;


import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
// import edu.wpi.first.math.util.Units;

public class ElevatorConstants {
  public static int kMotorID = 10;
  public static int kFollowerMotorID = 11;
  public static MotorType kMotorType = MotorType.kBrushless;
  public static IdleMode kIdleMode = IdleMode.kCoast;
  public static int kCurrentLimit = 40;
  public static double kP = 0.1;
  public static double kI = 0.0;
  public static double kD = 0.08;
  public static double kVelocityFF = 0.0;

  public static double kMaxAcceleration = 250;
  public static double kMaxVelocity = 500;
  public static double kTolerance = 1;

  public static double kForwardSoftLimit = 84;
  public static double kReverseSoftLimit = 0;


  public static double kGearRatio = 1 / 12;

  public static double kS = 0.0;
  public static double kG = 0.9;
  public static double kV = 0.0;
  public static double kA = 0.0;

  public static double kPositionConversionFactor = kGearRatio  * 360 ;
  public static double kVelocityConversionFactor = kPositionConversionFactor / 60; // RPM -> MPS




  public static class ElevatorConfigs {

    public static final SparkMaxConfig kElevatorConfig = new SparkMaxConfig();
    public static final SparkMaxConfig kElevatorFollowerConfig = new SparkMaxConfig();
    //77
    //WRIST - 2.73
    public enum Positions {
      BOTTOM(1), 
      INTAKE(3.65), // TODO: Configure me!
      POSTINTAKE(6.7),  // TODO: Configure me!
      L2(12.57),  // TODO: Configure me!
      L3(44.59),  // TODO: Configure me!
      L4(84),  // TODO: Configure me!
      Algae_1(32), // TODO: Configure me!
      Algae_2(69), // TODO: Configure me!
      FloorIntake(1);
      public final double position;

      private Positions(double position) {
        this.position = position;
      }

      public double getPos() {
        return this.position;
      }
    };

  static {
    kElevatorFollowerConfig
    .follow(kMotorID, true)
    .idleMode(kIdleMode)
    .smartCurrentLimit(kCurrentLimit);

    kElevatorConfig
      .idleMode(kIdleMode)
      .smartCurrentLimit(kCurrentLimit);

    System.out.println("KVCF "+kVelocityConversionFactor);
    System.out.println("KPCF "+kPositionConversionFactor);


    // kElevatorConfig
    //     .absoluteEncoder
    //     .positionConversionFactor(kPositionConversionFactor)
    //     .velocityConversionFactor(kVelocityConversionFactor);

      

    kElevatorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(kP, 0.0, kD, kVelocityFF)
        .outputRange(-0.45, 0.45);

    // kElevatorConfig
    //     .closedLoop
    //     .maxMotion
    //     .maxVelocity(kMaxVelocity)
    //     .maxAcceleration(kMaxAcceleration)
    //     .allowedClosedLoopError(kTolerance);
  }
}
}