package lib.control;

import lib.utils.DampingData;
import lib.utils.Timebase;
import org.apache.commons.math3.complex.Complex;
import org.nd4j.linalg.api.ndarray.INDArray;
import org.nd4j.linalg.factory.Nd4j;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.DoubleStream;

public abstract class LTI {

    protected long inputs;
    protected long outputs;
    protected Timebase timebase;

    public LTI(long inputs, long outputs, Timebase timebase) {
        this.inputs = inputs;
        this.outputs = outputs;
        this.timebase = timebase;
    }

    public boolean isDTime(boolean strict) {
        if (timebase.noTimebase) {
            return !strict;
        }

        return timebase.dt > 0;
    }

    public boolean isCTime(boolean strict) {
        if (timebase.noTimebase) {
            return !strict;
        }

        return timebase.dt == 0;
    }

    public boolean isSISO() {
        return inputs == 1 && outputs == 1;
    }

    public abstract List<Complex> pole();

    public DampingData damp() {
        List<Complex> poles = pole();

        List<Complex> splanePoles = new ArrayList<>();

        for (Complex c : poles) {
            if (isDTime(true)) {
                splanePoles.add(c.log().divide(timebase.dt));
            } else {
                splanePoles.add(c);
            }
        }

        INDArray wn = Nd4j.create(splanePoles.stream().flatMapToDouble(x -> DoubleStream.of(x.abs())).toArray());
        INDArray zeta = Nd4j.create(splanePoles.stream().flatMapToDouble(x -> DoubleStream.of(-x.getReal())).toArray()).div(wn);

        return new DampingData(wn, zeta, poles);
    }

}
