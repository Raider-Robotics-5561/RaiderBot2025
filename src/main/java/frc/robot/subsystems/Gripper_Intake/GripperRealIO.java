package frc.robot.subsystems.Gripper_Intake;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.subsystems.misc.IRBeam;

public class GripperRealIO implements GripperBaseIO {

	private SparkMax AngleGripper;
	private SparkMax wheelMotor;

	private IRBeam toggleSensor;

	public GripperRealIO() {

		AngleGripper = new SparkMax(12, MotorType.kBrushless);
		wheelMotor = new SparkMax(13, MotorType.kBrushless);
		toggleSensor = new IRBeam(0);
	}

	@Override
	public void updateInputs(GripperInputs inputs) {

		inputs.armAngleDegrees = AngleGripper.getEncoder().getPosition();
		inputs.armMotorCurrent = AngleGripper.getOutputCurrent();
		inputs.wheelMotorCurrent = wheelMotor.getOutputCurrent();
		inputs.wheelRPM = wheelMotor.getEncoder().getVelocity();
		inputs.toggleSensor = toggleSensor.getState();
		inputs.distanceSensorCM = 0;
	}

	@Override
	public void setArmAngle(double angle) {
		AngleGripper.getClosedLoopController().setReference(angle, ControlType.kMAXMotionPositionControl);
	}

	@Override
	public void setIntakeSpeed(double speed) {
		wheelMotor.set(speed);
	}
}