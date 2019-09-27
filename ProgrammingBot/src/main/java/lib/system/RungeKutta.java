package lib.system;

import org.ejml.simple.SimpleMatrix;

public class RungeKutta {

    public static SimpleMatrix rungeKutta(NonlinearFunction f, SimpleMatrix x, SimpleMatrix u, double dt) {
        double halfDt = dt * 0.5;
        SimpleMatrix k1 = f.findStateDerivative(x, u);
        SimpleMatrix k2 = f.findStateDerivative(x.plus(k1.scale(halfDt)), u);
        SimpleMatrix k3 = f.findStateDerivative(x.plus(k2.scale(halfDt)), u);
        SimpleMatrix k4 = f.findStateDerivative(x.plus(k3.scale(dt)), u);
        return x.plus(k1.plus(k2.scale(2.0)).plus(k3.scale(2.0)).plus(k4).scale(dt / 6.0));
    }

}
