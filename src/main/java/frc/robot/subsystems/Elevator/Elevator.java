package frc.robot.subsystems.Elevator;


import edu.wpi.first.wpilibj.DutyCycleEncoder;

import com.revrobotics.spark.SparkMaxAlternateEncoder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TunableNumber;
import frc.robot.util.Constants.ElevatorConstants;


public class Elevator extends SubsystemBase {
  private final SparkMax mElevatorSparkMax;
  private final SparkMax mElevatorSparkMaxFollower;
  private final SparkClosedLoopController mElevatorController;
  private final SparkAbsoluteEncoder elevatorAbsEncoder;
  private final RelativeEncoder elevatorRelEncoder;
  // private final AbsoluteEncoder m_alternateEncoder;
  // private final double kCPR = 8192;

  private double motorVoltage = 0;

  private TunableNumber elevatorFF, elevatorP, elevatorI, elevatorD;
  private TunableNumber elevatorTunableSetpoint;

  public Elevator() {
    mElevatorSparkMax = new SparkMax(ElevatorConstants.kMotorID, MotorType.kBrushless);
    mElevatorSparkMaxFollower = new SparkMax(ElevatorConstants.kFollowerMotorID, MotorType.kBrushless);
    // kAltEncType = com.rev
    elevatorRelEncoder = mElevatorSparkMax.getEncoder();
    mElevatorController = mElevatorSparkMax.getClosedLoopController();
    elevatorAbsEncoder = mElevatorSparkMax.getAbsoluteEncoder();
    //  m_alternateEncoder = mElevatorSparkMax.GetAlternateEncoder(kAltEncType, kCPR);

 
    ElevatorConstants.ElevatorConfigs.kElevatorFollowerConfig
    .follow(ElevatorConstants.kMotorID, true);
    
    mElevatorSparkMaxFollower.configure(
      ElevatorConstants.ElevatorConfigs.kElevatorFollowerConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kNoPersistParameters);


    mElevatorSparkMax.configure(
        ElevatorConstants.ElevatorConfigs.kElevatorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);

    SmartDashboard.putNumber("Elevator Target Position", 0);
    SmartDashboard.putNumber("Elevator Target Velocity", 0);

    elevatorFF = new TunableNumber("Elevator/Elevator FF");
    elevatorP = new TunableNumber("Elevator/Elevator P");
    elevatorI = new TunableNumber("Elevator/Elevator I");
    elevatorD = new TunableNumber("Elevator/Elevator D");
    elevatorTunableSetpoint = new TunableNumber("Elevator/Setpoint");

    elevatorFF.setDefault(ElevatorConstants.kG);
    elevatorP.setDefault(ElevatorConstants.kP);
    elevatorI.setDefault(0);
    elevatorD.setDefault(ElevatorConstants.kD);
    elevatorTunableSetpoint.setDefault(getEncoderMeasurement());
  }

  public void setMotorVoltage(double pVoltage) {
    // if (pVoltage < 0.0) pVoltage *= 0.2; // Slows down downward movements
    mElevatorSparkMax.setVoltage(filterVoltage(pVoltage));
  }

  public void stepMotorVoltage(double value) {
    motorVoltage += value;
    setMotorVoltage(motorVoltage);
  }

  private double filterVoltage(double pVoltage) {
    return filterToLimits(MathUtil.clamp(pVoltage, -12.0, 12.0));
  }

  public double getEncoderMeasurement() {
    return elevatorRelEncoder.getPosition();
  }

  private double filterToLimits(double pInput) {
    return (pInput > 0 && elevatorRelEncoder.getPosition() >= ElevatorConstants.kForwardSoftLimit)
            || (pInput < 0 && elevatorRelEncoder.getPosition() <= ElevatorConstants.kReverseSoftLimit)
        ? 0.0
        : pInput;
  }

  private void stopIfLimit() {
    double motorOutput = getMotorOutput();
    if ((motorOutput > 0 && elevatorAbsEncoder.getPosition() >= ElevatorConstants.kForwardSoftLimit)
        || (motorOutput < 0 && elevatorAbsEncoder.getPosition() <= ElevatorConstants.kReverseSoftLimit)) {
        System.out.println("Elevator Stop Limit");
      setMotorVoltage(0);
    }
  }

  public void goToSetpoint(double pSetpoint) {
    System.out.println("setpoint:"+pSetpoint);
    mElevatorController.setReference(pSetpoint, ControlType.kPosition);
  }

  public boolean isAtSetpoint(double pSetpoint) {
    return (elevatorAbsEncoder.getPosition() >= pSetpoint - ElevatorConstants.kTolerance)
        && (elevatorAbsEncoder.getPosition() <= pSetpoint + ElevatorConstants.kTolerance);
  }

  public double getMotorOutput() {
    return mElevatorSparkMax.getAppliedOutput();
  }

  @Override
  public void periodic() {
    stopIfLimit();

    SmartDashboard.putNumber("Elevator ABS Position", elevatorAbsEncoder.getPosition());
    SmartDashboard.putNumber("Elevator ABS Velocity", elevatorAbsEncoder.getVelocity());
    SmartDashboard.putNumber("Elevator Rel Position", elevatorRelEncoder.getPosition());
    SmartDashboard.putNumber("Elevator Rel Velocity", elevatorRelEncoder.getVelocity());
    SmartDashboard.putNumber("Elevator Output", getMotorOutput());
    SmartDashboard.putNumber("Elevator Voltage", mElevatorSparkMax.getBusVoltage());
    SmartDashboard.putNumber("Stepped Elevator Voltage", motorVoltage);
  }
}