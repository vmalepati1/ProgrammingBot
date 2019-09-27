package lib.math;

public class Special {

    public static double DOUBLE_EPSILON = 1.11022302462515654042e-16;

    public static double epslon(double x)
    {
        double a, b, c, eps;

        a = 4.0/3.0;

        b = a - 1.0;
        c = b + b + b;
        eps = Math.abs(c - 1.0);

        if (eps == 0.0) {
            System.out.println("Failure: arithmetic base may be a multiple of 3");
        } else {
            eps = eps/2.0;
            if (1.0 + eps == 1.0) {
                eps = 2.0 * eps;
            }
        }

        return eps * Math.abs(x);
    }

    public static double clip(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

}
