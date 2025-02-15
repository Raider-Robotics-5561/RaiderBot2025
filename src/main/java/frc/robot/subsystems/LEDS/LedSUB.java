package frc.robot.subsystems.LEDS;


import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;


public class LedSUB {
  AddressableLED led;
  AddressableLEDBuffer ledBuffer;
  
 
  public LedSUB(){
    led = new AddressableLED (0);
    ledBuffer = new AddressableLEDBuffer(20);
    led.setLength(20);
    led.setLength(ledBuffer.getLength());
    led.start();
    
  }

  }
  
 
 
