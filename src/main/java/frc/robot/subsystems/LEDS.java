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

/**
 * The LEDS class represents the subsystem for controlling LEDs on the robot.
 */
public class LEDS extends SubsystemBase {
	CANdle leds = new CANdle(23);
	private int LED_LENGTH = 19;

	public LEDS() {}

	/**
	 * Initializes the LEDS subsystem.
	 */
	public void init() {
		leds.animate(new RainbowAnimation(0.5, 0.1, LED_LENGTH, false, 8));
	}

	/**
	 * Flashes the LEDs with the specified color.
	 *
	 * @param color the color to flash the LEDs with
	 */
	public void flashLeds(Color color) {
		leds.animate(new StrobeAnimation(
				(int) (color.red * 255), (int) (255 * color.green), (int) (color.blue) * 255, 0, 0.125, LED_LENGTH, 8));
	}

	/**
	 * Animates the LEDs with the specified animation.
	 *
	 * @param animation the animation to apply to the LEDs
	 */
	public void animate(Animation animation) {
		leds.animate(animation);
	}

	/**
	 * Creates a command to twinkle the LEDs with the specified color.
	 *
	 * @param color the color to twinkle the LEDs with
	 * @return the command to twinkle the LEDs
	 */
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

	/**
	 * Creates a command to fade the LEDs through RGB colors.
	 *
	 * @return the command to fade the LEDs
	 */
	public Command RGBFade() {
		return this.runOnce(() -> leds.animate(new RgbFadeAnimation(1.0, 0.6, LED_LENGTH, 8)));
	}

	/**
	 * Creates a command to fade the LEDs with the specified color.
	 *
	 * @param color the color to fade the LEDs with
	 * @return the command to fade the LEDs
	 */
	public Command fade(Color color) {
		return this.runOnce(() -> leds.animate(new SingleFadeAnimation(
				(int) (color.red * 255), (int) (255 * color.green), (int) (color.blue) * 255, 0, 0.6, LED_LENGTH, 8)));
	}

	/**
	 * Creates a command to animate the LEDs in a rainbow pattern.
	 *
	 * @return the command to animate the LEDs in a rainbow pattern
	 */
	public Command rainbow() {
		return this.runOnce(() -> leds.animate(new RainbowAnimation(0.5, 0.1, LED_LENGTH, false, 8)));
	}

	@Override
	public void periodic() {
		// if(shooter.isReady()) {
		//     leds.setLEDs(0, 255, 0);
		// }
		// else {
		//     leds.setLEDs(255,0,255);
		// }
	}
}
