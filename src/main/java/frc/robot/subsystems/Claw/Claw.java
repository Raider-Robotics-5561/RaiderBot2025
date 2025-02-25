// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Claw;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TunableNumber;
import frc.robot.util.Constants.ClawConstants;
import frc.robot.util.Constants.ClawConstants.Roller;
import frc.robot.util.Constants.ClawConstants.Wrist.ClawRollerVolt;

import com.revrobotics.spark.SparkLimitSwitch;

public class Claw extends SubsystemBase {
  private final SparkMax mWristSparkMax;
  private final SparkMax kRollerID;


  private final SparkClosedLoopController mWristController;
  private AbsoluteEncoder mWristEncoder;

  private TunableNumber wristP, wristD, wristG, wristV, wristA;
  private TunableNumber tunablePosition;

  private SparkLimitSwitch m_forwardLimit;
  private SparkLimitSwitch m_reverseLimit;

  public String kEnable;
  public String kDisable;

  public Claw() {
    this.kRollerID =
        new SparkMax(ClawConstants.Roller.kRollerID, ClawConstants.Roller.kMotorType);

    this.mWristSparkMax = new SparkMax(ClawConstants.Wrist.kMotorID, ClawConstants.Wrist.kMotorType);
    this.mWristController = mWristSparkMax.getClosedLoopController();
    this.mWristEncoder = mWristSparkMax.getAbsoluteEncoder();

      mWristSparkMax.configure(
        ClawConstants.Wrist.kWristConfig, 
        ResetMode.kResetSafeParameters, 
        PersistMode.kNoPersistParameters);
    
      kRollerID.configure(
        ClawConstants.Roller.kRollerConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);

    // m_forwardLimit = kRollerID.getForwardLimitSwitch(SparkLimitSwitch.class.);
    m_forwardLimit = kRollerID.getForwardLimitSwitch();
    m_reverseLimit = kRollerID.getReverseLimitSwitch();

    wristP = new TunableNumber("Wrist/kP", ClawConstants.Wrist.kP);
    wristD = new TunableNumber("Wrist/kD", ClawConstants.Wrist.kD);
    wristV = new TunableNumber("Wrist/kVelocity", ClawConstants.Wrist.kV);
    wristA = new TunableNumber("Wrist/kAcceleration", ClawConstants.Wrist.kA);
    tunablePosition = new TunableNumber("Wrist/Tunable Setpoint", 0);
    // wristP.setDefault(0.0);
    // wristD.setDefault(0.0);
    // wristV.setDefault(0.0);
    // wristA.setDefault(0.0);
  }

  public boolean getForwardLimitSwitch1() {
    return m_forwardLimit.isPressed(); 
  }

  public boolean getReverseLimitSwitch1() {
    return m_reverseLimit.isPressed(); 
  }

  public void setRollerPower(ClawRollerVolt pVoltage) {
    setRollerPower(pVoltage.get());
  }

  public void setRollerPower(double pVoltage) {
    kRollerID.setVoltage(filterVoltage(pVoltage));
  }

  public void setWrist(double pVoltage) {
    mWristSparkMax.setVoltage(filterVoltage(pVoltage));
  }

  private double filterVoltage(double pVoltage) {
    return filterToLimits(MathUtil.clamp(pVoltage, -12.0, 12.0));
  }

  public double getEncoderMeasurement() {
    double encoderMeasurement = mWristEncoder.getPosition();
    // if (encoderMeasurement > ClawConstants.Wrist.kPositionConversionFactor / 2.0)
    //   encoderMeasurement -= ClawConstants.Wrist.kPositionConversionFactor;
    // Why would we want to do this? 
    return encoderMeasurement;
  }

  // private double filterToLimits(double pInput) {
  //   return (pInput > 0 && mWristEncoder.getPosition() >= ElevatorConstants.kForwardSoftLimit)
  //           || (pInput < 0 && mWristEncoder.getPosition() <= ElevatorConstants.kReverseSoftLimit)
  //       ? 0.0
  //       : pInput;
  // }

  private double filterToLimits(double pInput) {
    return (pInput > 0 && getEncoderMeasurement() <= ClawConstants.Wrist.kForwardSoftLimit)
            || (pInput < 0 && getEncoderMeasurement() >= ClawConstants.Wrist.kReverseSoftLimit)
        ? 0.0
        : pInput;
  }

  private void stopIfLimit() {
    double motorOutput = getMotorOutput();
    if ((motorOutput > 0 && getEncoderMeasurement() <= ClawConstants.Wrist.kForwardSoftLimit)
        || (motorOutput < 0 && getEncoderMeasurement() >= ClawConstants.Wrist.kReverseSoftLimit)) {
      System.out.println("Claw Stop Limit Reached");
      setWrist(0);
    }
  }

  public double getMotorOutput() {
    return mWristSparkMax.getAppliedOutput();
  }

  public void goToSetpoint(double pSetpoint) {
    mWristController.setReference(pSetpoint, ControlType.kPosition);
  }

  @Override
  public void periodic() {
    stopIfLimit();
    SmartDashboard.putNumber("Wrist/Position", getEncoderMeasurement());
    SmartDashboard.putNumber("Wrist/Position/Abs",  mWristEncoder.getPosition());
    SmartDashboard.putNumber("Wrist/Voltage", mWristSparkMax.getBusVoltage());
    SmartDashboard.putBoolean("Wrist/ForwardLimitSwitch", getForwardLimitSwitch1());
    SmartDashboard.putBoolean("Wrist/ReverseLimitSwitch", getReverseLimitSwitch1());

    if (wristP.hasChanged()) ClawConstants.Wrist.kP = wristP.get();
    // SmartDashboard.putNumber("Tuning/Wrist/Current P", Wrist.kP);
    if (wristD.hasChanged()) ClawConstants.Wrist.kD = wristD.get();
    if (wristV.hasChanged()) ClawConstants.Wrist.kMaxVelocity = wristV.get();
    if (wristD.hasChanged()) ClawConstants.Wrist.kMaxAcceleration = wristA.get();
  }
}