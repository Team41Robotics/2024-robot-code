package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.LarsonAnimation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDS extends SubsystemBase {
	CANdle leds = new CANdle(23);

	public LEDS() {}

	public void init() {
		Color color = Color.kChartreuse;

		leds.animate(new LarsonAnimation(
				((int) (255 * color.red)),
				((int) (255 * color.green)),
				((int) (255 * color.blue)),
				1,
				0.5,
				40,
				LarsonAnimation.BounceMode.Center,
				4,
				8));
		// leds.animate(new RainbowAnimation(0.1,0.1,8));
		leds.setLEDs(100, 0, 0, 100, 0, 8);
		// leds.animate(new FireAnimation(0.2, 0.05, 40, 0.1, 0.1, false, 8));
	}
}
