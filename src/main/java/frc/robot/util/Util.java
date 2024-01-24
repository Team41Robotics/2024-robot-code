package frc.robot.util;

import static java.lang.Math.*;

public class Util {
	private static double a = 3;

	

	public static double sensCurve(double w, double d) {
		// quadratic sensitivity curve
		if (abs(w) < d) return 0;
		double wn = (w - d) / (1 - d);

		wn = sigmoid(wn) / sigmoid(1);
		return wn * signum(w);
	}
	private static double sigmoid(double x){
		return 2/(1.0+exp(-x*a))-1;
	}
}
