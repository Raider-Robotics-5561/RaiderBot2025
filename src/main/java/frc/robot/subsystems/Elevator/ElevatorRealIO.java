package frc.robot.subsystems.Elevator;

import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import edu.wpi.first.wpilibj.Encoder;
// import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;;

//TODO - ADD THROUGH BORE ENCODER

public class ElevatorRealIO implements ElevatorBaseIO {

	private SparkMax ElevatormotorLeft;
	private SparkMax ElevatormotorRight;
	public double ElevatorENCPOS;

	public ElevatorRealIO() {

		ElevatormotorLeft = new SparkMax(10, MotorType.kBrushless);
		ElevatormotorRight = new SparkMax(11, MotorType.kBrushless);


		SparkMaxConfig globalConfig = new  SparkMaxConfig();
		SparkMaxConfig ElevatorLeftConfig = new SparkMaxConfig();
		SparkMaxConfig ElevatorRightConfig = new SparkMaxConfig();

		
		globalConfig
		.smartCurrentLimit(40)
		.idleMode(IdleMode.kBrake);

		ElevatorLeftConfig
		.apply(globalConfig);

		ElevatorRightConfig
		.apply(globalConfig)
		.follow(ElevatormotorLeft)
		.inverted(true);
	

		ElevatormotorLeft.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		ElevatormotorRight.configure(ElevatorRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

	}
	
	
	// public void GetEncoderPOS(double ElevatorENCPOS){
	// 	// Initializes an encoder on DIO pins 0 and 1
	// 	// 2X encoding and non-inverted
	// 	final Encoder ElevatorENC = new Encoder(0, 1, false, Encoder.EncodingType.k2X);
	// 	// Configures the encoder to return a distance of 4 for every 256 pulses
	// 	// Also changes the units of getRate
	// 	ElevatorENC.setDistancePerPulse(4.0/256.0);
	// 	// Configures the encoder to consider itself stopped when its rate is below 10
	// 	ElevatorENC.setMinRate(10);
	// 	// Reverses the direction of the encoder
	// 	ElevatorENC.setReverseDirection(true);
	// 	// Configures an encoder to average its period measurement over 5 samples
	// 	// Can be between 1 and 127 samples
	// 	ElevatorENC.setSamplesToAverage(5);
	// 	ElevatorENC.getDistance();
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
		// positionM = ElevatormotorLeft.getAbsoluteEncoder().getPosition();
		ElevatormotorLeft.set(positionM);
		
	}
}