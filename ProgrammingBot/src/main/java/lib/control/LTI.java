package lib.control;

import lib.utils.DampingData;
import lib.utils.Timebase;

import java.util.List;

public abstract class LTI {

    protected int inputs;
    protected int outputs;
    protected Timebase timebase;

    public LTI(int inputs, int outputs, Timebase timebase) {
        this.inputs = inputs;
        this.outputs = outputs;
        this.timebase = timebase;
    }

    public boolean isDTime(boolean strict) {
        if (timebase == null) {
            return !strict;
        }

        return timebase.noTimebase || timebase.dt > 0;
    }

    public boolean isCTime(boolean strict) {
        if (timebase == null) {
            return !strict;
        }

        return !timebase.noTimebase && timebase.dt == 0;
    }

    public boolean isSISO() {
        return inputs == 1 && outputs == 1;
    }

    public abstract List<Double> pole();

    public DampingData damp() {
        /*
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
        */
        return null;
    }

    public static boolean isSISO(Object sys, boolean strict) throws Exception {
        if (sys instanceof Number && !strict) {
            return true;
        } else if (!(sys instanceof LTI)) {
            throw new Exception("Object is not an LTI system");
        }

        return ((LTI) sys).isSISO();
    }

    public static Double timebase(Object sys) throws Exception {
        if (sys instanceof Number) {
            return null;
        } else if (!(sys instanceof LTI)) {
            throw new Exception("Timebase not defined");
        }

        if (((LTI) sys).timebase == null) {
            return null;
        }

        return ((LTI) sys).timebase.noTimebase ? 1 : ((LTI) sys).timebase.dt;
    }

    public static boolean timebaseEqual(LTI sys1, LTI sys2) {
        if (sys1.timebase == null || sys2.timebase == null) {
            return true;
        } else if (sys1.timebase.noTimebase || sys2.timebase.noTimebase) {
            return sys1.timebase.noTimebase && sys2.timebase.noTimebase;
        } else {
            return sys1.timebase.dt == sys2.timebase.dt;
        }
    }

    public static boolean isDTime(Object sys, boolean strict) {
        if (sys instanceof Number) {
            return !strict;
        }

        if (sys instanceof LTI) {
            return ((LTI) sys).isDTime(strict);
        }

        return false;
    }

    public static boolean isCTime(Object sys, boolean strict) {
        if (sys instanceof Number) {
            return !strict;
        }

        if (sys instanceof LTI) {
            return ((LTI) sys).isCTime(strict);
        }

        return false;
    }

}
