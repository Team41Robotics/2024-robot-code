package frc.robot.util;

import static java.lang.Math.*;

public class Util {

        public static double sensCurve(double w, double d) {
                // quadratic sensitivity curve
                if(abs(w) < d) return 0;
                double wn = (w-d) / (1-d);
                wn = wn * wn;
                return wn * signum(w);
        }
        
}
