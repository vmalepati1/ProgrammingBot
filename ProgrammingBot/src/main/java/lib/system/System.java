package lib.system;

import edu.wpi.first.wpilibj.command.Subsystem;
import org.ejml.data.Complex_F64;
import org.ejml.simple.SimpleMatrix;

import static lib.math.Special.clip;

public abstract class System extends Subsystem {

    private NonlinearFunction f;
    private StateSpace sysc;
    private double dt;
    private StateSpace sysd;

    private SimpleMatrix x;
    private SimpleMatrix u;
    private SimpleMatrix y;

    private SimpleMatrix r;

    private SimpleMatrix xHat;

    private SimpleMatrix uMin;
    private SimpleMatrix uMax;

    private SimpleMatrix K;
    private SimpleMatrix Kff;

    private SimpleMatrix P;
    private SimpleMatrix kalmanGain;

    private SimpleMatrix Q;
    private SimpleMatrix R;

    public System(SimpleMatrix uMin, SimpleMatrix uMax, double dt,
                  SimpleMatrix states, SimpleMatrix inputs) throws Exception {
        this(uMin, uMax, dt, states, inputs, null);
    }

    public System(SimpleMatrix uMin, SimpleMatrix uMax, double dt,
                  SimpleMatrix states, SimpleMatrix inputs, NonlinearFunction nonlinearFunc) throws Exception {
        this.f = nonlinearFunc;
        this.sysc = createModel(states.copy(), inputs.copy());
        this.dt = dt;
        this.sysd = sysc.sample(dt, "zoh", null);

        this.x = states.copy();
        this.u = inputs.copy();
        this.y = new SimpleMatrix(sysc.C.numRows(), 1);

        this.r = new SimpleMatrix(sysc.A.numRows(), 1);

        this.xHat = states.copy();

        this.uMin = uMin.copy();
        this.uMax = uMax.copy();

        this.K = new SimpleMatrix(sysc.B.numCols(), sysc.B.numRows());
        this.Kff = new SimpleMatrix(sysc.B.numCols(), sysc.B.numRows());

        this.P = new SimpleMatrix(sysc.A.numRows(), sysc.A.numCols());
        this.kalmanGain = new SimpleMatrix(sysc.A.numRows(), sysc.C.numRows());

        designControllerObserver();
    }

    public void update() {
        updatePlant();
        correctObserver();
        updateController(null);
        predictObserver();
    }

    protected void updatePlant() {
        if (f != null) {
            x = RungeKutta.rungeKutta(f, x, u, dt);
        } else {
            x = sysd.A.mult(x).plus(sysd.B.mult(u));
        }

        y = sysd.C.mult(x).plus(sysd.D.mult(u));
    }

    protected void predictObserver() {
        if (f != null) {
            xHat = RungeKutta.rungeKutta(f, x, u, dt);
            P = sysd.A.mult(P).mult(sysd.A.transpose()).plus(Q);
        } else {
            xHat = sysd.A.mult(xHat).plus(sysd.B.mult(u));
        }
    }

    protected void correctObserver() {
        if (f != null) {
            kalmanGain = P.mult(sysd.C.transpose())
                    .mult(sysd.C.mult(P).mult(sysd.C.transpose()).plus(R).invert());
            P = SimpleMatrix.identity(sysd.A.numRows()).minus(kalmanGain.mult(sysd.C)).mult(P);
        }
        xHat = xHat.plus(kalmanGain.mult(y.minus(sysd.C.mult(xHat)).minus(sysd.D.mult(u))));
    }

    protected void updateController(SimpleMatrix nextR) {
        u = K.mult(r.minus(xHat));
        SimpleMatrix uff;

        if (nextR != null) {
            if (f != null) {
                SimpleMatrix rDot = nextR.minus(r).scale(1/dt);
                uff = Kff.mult(rDot.minus(
                        f.findStateDerivative(r, new SimpleMatrix(u.numRows(), u.numCols()))));
            } else {
                uff = Kff.mult(nextR.minus(sysd.A.mult(r)));
                r = nextR;
            }
        } else {
            if (f != null) {
                uff = Kff.negative().mult(f.findStateDerivative(r,
                        new SimpleMatrix(u.numRows(), u.numCols())));
            } else {
                uff = Kff.mult(r.minus(sysd.A.mult(r)));
            }
        }

        for (int y = 0; y < u.numRows(); y++) {
            for (int x = 0; x < u.numCols(); x++) {
                u.set(y, x, clip(u.get(y, x), uMin.get(y, x), uMax.get(y, x)));
            }
        }
    }

    protected abstract StateSpace createModel(SimpleMatrix states, SimpleMatrix inputs);

    protected abstract void designControllerObserver();

    // TODO: Write all of the following methods properly
    protected void designLQR(SimpleMatrix lqr) {
        K = lqr.copy();
    }

    protected void placeControllerPoles(Complex_F64[] poles) throws Exception {
        throw new Exception("Not implemented yet!");
    }

    protected void designKalmanFilter(SimpleMatrix qElems, SimpleMatrix rElems,
                                      SimpleMatrix kalmanGain) {
        Q = makeCovMatrix(qElems);
        R = makeCovMatrix(rElems);

        this.kalmanGain = kalmanGain;
    }

    protected void placeObserverPoles(Complex_F64[] poles) throws Exception {
        throw new Exception("Not implemented yet!");
    }

    protected void designTwoStateFeedforward(SimpleMatrix qElems, SimpleMatrix rElems) {
        if (qElems != null && rElems != null) {
            Q = makeCostMatrix(qElems);
            R = makeCostMatrix(rElems);
            if (f != null) {
                Kff = sysc.B.transpose().mult(Q).mult(sysc.B).
                        plus(R.transpose()).invert().mult(sysc.B.transpose()).mult(Q);
            } else {
                Kff = sysd.B.transpose().mult(Q).mult(sysd.B).
                        plus(R.transpose()).invert().mult(sysd.B.transpose()).mult(Q);
            }
        } else {
            if (f != null) {
                Kff = sysc.B.pseudoInverse();
            } else {
                Kff = sysd.B.pseudoInverse();
            }
        }
    }

    private SimpleMatrix makeCostMatrix(SimpleMatrix elems) {
        return SimpleMatrix.diag(elems.elementPower(-2).getDDRM().data);
    }

    private SimpleMatrix makeCovMatrix(SimpleMatrix elems) {
        return SimpleMatrix.diag(elems.elementPower(2).getDDRM().data);
    }

}
