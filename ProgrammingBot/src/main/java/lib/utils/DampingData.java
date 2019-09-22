package lib.utils;

import org.apache.commons.math3.complex.Complex;
import org.nd4j.linalg.api.ndarray.INDArray;

import java.util.List;

public class DampingData {

    // Natural frequencies of each system pole
    public INDArray wn;
    // Damping ratio for each system pole
    public INDArray zeta;
    // List of system poles
    public List<Complex> poles;

    public DampingData(INDArray wn, INDArray zeta, List<Complex> poles) {
        this.wn = wn;
        this.zeta = zeta;
        this.poles = poles;
    }

}
