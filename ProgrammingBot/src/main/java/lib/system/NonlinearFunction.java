package lib.system;

import org.ejml.simple.SimpleMatrix;

public interface NonlinearFunction {

    SimpleMatrix findStateDerivative(SimpleMatrix x, SimpleMatrix u);

}
