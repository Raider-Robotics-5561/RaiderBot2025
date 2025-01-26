package frc.robot.subsystems.Elevator;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;


public class ElevatorRealIO implements ElevatorBaseIO {

	private SparkMax ElevatormotorLeft;
	private SparkMax ElevatormotorRight;

	public ElevatorRealIO() {

		ElevatormotorLeft = new SparkMax(0, MotorType.kBrushless);
		ElevatormotorRight = new SparkMax(0, MotorType.kBrushless);
	}

	// public final MotorConfig(){
	// 	ElevatormotorRight.follow(ElevatormotorLeft);
	// 	return ElevatormotorRight;
	// }
	
	@Override
	public void updateInputs(ElevatorInputs inputs) {
		inputs.heightM = ElevatormotorLeft.getEncoder().getPosition();
		inputs.velocityMPS = ElevatormotorLeft.getEncoder().getVelocity();
		inputs.ElevatormotorLeftCurrent = ElevatormotorLeft.getOutputCurrent();
		inputs.ElevatormotorRightCurrent = ElevatormotorRight.getOutputCurrent();
		
	}

	@Override
	public void setPositionM(double positionM) {
		ElevatormotorLeft.set(positionM);
		
	}
}