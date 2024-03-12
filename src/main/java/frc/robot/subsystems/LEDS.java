package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDS extends SubsystemBase {
	CANdle leds = new CANdle(23);
	private int LED_LENGTH = 19;

	public LEDS() {}

	public void init() {
		Color color = Color.kChartreuse;
		leds.animate(new RainbowAnimation(0.5, 0.1, LED_LENGTH, false, 8));
		// leds.animate(new FireAnimation(0.2, 0.05, 40, 0.1, 0.1, false, 8));
	}

	public void flashLeds(Color color) {
		leds.animate(new StrobeAnimation(
				(int) (color.red * 255), (int) (255 * color.green), (int) (color.blue) * 255, 0, 0.125, LED_LENGTH, 8));
		// System.out.println("LEDS FLASHING!");
	}

	public void animate(Animation animation) {
		leds.animate(animation);
	}

	public Command twinkle(Color color) {
		return this.runOnce(() -> leds.animate(new TwinkleAnimation(
				(int) (color.red * 255),
				(int) (255 * color.green),
				(int) (color.blue) * 255,
				0,
				0.6,
				LED_LENGTH,
				TwinklePercent.Percent88,
				8)));
	}

	public Command RGBFade() {
		return this.runOnce(() -> leds.animate(new RgbFadeAnimation(1.0, 0.6, LED_LENGTH, 8)));
	}

	public Command fade(Color color) {
		return this.runOnce(() -> leds.animate(new SingleFadeAnimation(
				(int) (color.red * 255), (int) (255 * color.green), (int) (color.blue) * 255, 0, 0.6, LED_LENGTH, 8)));
	}

	public Command rainbow() {
		return this.runOnce(() -> leds.animate(new RainbowAnimation(0.5, 0.1, LED_LENGTH, false, 8)));
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
