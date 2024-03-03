package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDS extends SubsystemBase {
	CANdle leds = new CANdle(23);
	private double LED_LENGTH = 34;

	public LEDS() {}

	public void init() {
		Color color = Color.kChartreuse;
		leds.animate(new RainbowAnimation(0.1, 0.1, 34));
		// leds.animate(new FireAnimation(0.2, 0.05, 40, 0.1, 0.1, false, 8));
	}

	public void flashLeds(Color color) {
		leds.animate(new StrobeAnimation((int) color.red, (int) color.green, (int) color.blue));
		System.out.println("LEDS FLASHING!");
	}

	@Override
	public void periodic() {
		// if(shooter.isReady()) {
		// 	leds.setLEDs(0, 255, 0);
		// }
		// else {
		// 	leds.setLEDs(255,0,255);
		// }
	}
}
