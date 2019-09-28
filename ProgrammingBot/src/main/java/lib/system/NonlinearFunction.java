package lib.system;

import org.ejml.simple.SimpleMatrix;

/**
 * @author Vikas Malepati
 */
public interface NonlinearFunction {

    SimpleMatrix findStateDerivative(SimpleMatrix x, SimpleMatrix u);

}
