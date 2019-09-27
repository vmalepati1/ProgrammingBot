package lib.system;

import lib.math.GEVDSolver;
import lib.utils.FrequencyResponseData;
import lib.utils.SetOperations;
import lib.utils.Timebase;
import org.ejml.data.ComplexPolar_F64;
import org.ejml.data.Complex_F64;
import org.ejml.data.ZMatrixRMaj;
import org.ejml.dense.row.CommonOps_ZDRM;
import org.ejml.dense.row.decompose.lu.LUDecompositionAlt_ZDRM;
import org.ejml.dense.row.linsol.lu.LinearSolverLu_ZDRM;
import org.ejml.equation.Equation;
import org.ejml.simple.SimpleMatrix;

import java.util.*;

import static org.ejml.dense.row.CommonOps_ZDRM.*;

public class StateSpace extends LTI {

    public SimpleMatrix A;
    public SimpleMatrix B;
    public SimpleMatrix C;
    public SimpleMatrix D;
    public int states;

    public StateSpace(SimpleMatrix A, SimpleMatrix B, SimpleMatrix C, SimpleMatrix D, Timebase timebase) throws Exception {
        super(preprocessFeedthroughMatrix(D, B, C).numCols(), D.numRows(), timebase);

        A = A.copy();
        B = B.copy();
        C = C.copy();

        this.A = A;
        this.B = B;
        this.C  = C;
        this.D = D;
        this.states = A.numCols();

        if (states == 0) {
            A.reshape(0, 0);
            B.reshape(0, inputs);
            C.reshape(outputs, 0);
        }

        if (states != A.numRows()) {
            throw new Exception("A must be square.");
        }
        if (states != B.numRows()) {
            throw new Exception("A and B must have the same number of rows.");
        }
        if (states != C.numCols()) {
            throw new Exception("A and C must have the same number of columns.");
        }
        if (inputs != B.numCols()) {
            throw new Exception("B and D must have the same number of columns.");
        }
        if (outputs != C.numRows()) {
            throw new Exception("C and D must have the same number of rows.");
        }

        removeUselessStates();
    }

    private static SimpleMatrix preprocessFeedthroughMatrix(SimpleMatrix D, SimpleMatrix B, SimpleMatrix C) throws Exception {
        D = D.copy();

        if (D.numRows() == 1 && D.numCols() == 1 && D.get(0) == 0 && B.numCols() > 0 && C.numRows() > 0) {
            D = new SimpleMatrix(C.numRows(), B.numCols());
        }

        return D;
    }

    private Set<Integer> getAxisZeroIndices(SimpleMatrix matrix, int axis) {
        SimpleMatrix a = matrix.copy();

        if (axis == 0) {
            a = a.transpose();
        }

        Set<Integer> result = new HashSet<>();

        int rowNum = 0;

        for (int i = 0; i < a.getNumElements(); i += a.numCols()) {
            boolean any = false;

            for (int j = 0; j < a.numCols(); j++) {
                if (a.get(i + j) != 0) {
                    any = true;
                    break;
                }
            }

            if (!any) {
                result.add(rowNum);
            }

            rowNum++;
        }

        return result;
    }

    private SimpleMatrix deleteOnAxis(SimpleMatrix matrix, Collection<Integer> indices, int axis) {
        SimpleMatrix a = matrix.copy();

        if (axis == 1) {
            a = a.transpose();
        }

        SimpleMatrix result = new SimpleMatrix(a.numRows() - 1, a.numCols());

        for (Integer index : indices) {
            if (index == 0) {
                result = a.extractMatrix(1, a.numRows(), 0, a.numCols());
            } else if (index == a.numRows() - 1) {
                result = a.extractMatrix(0, a.numRows() - 1, 0, a.numCols());
            } else {
                SimpleMatrix partitionUpper = a.extractMatrix(0, index, 0, a.numCols());
                SimpleMatrix partitionLower = a.extractMatrix(index + 1, a.numRows(), 0, a.numCols());
                result = partitionUpper.concatRows(partitionLower);
            }
        }

        return axis == 1 ? result.transpose() : result;
    }

    private void removeUselessStates() {
        Set<Integer> ax1A = getAxisZeroIndices(A, 1);
        Set<Integer> ax1B = getAxisZeroIndices(B, 1);
        Set<Integer> ax0A = getAxisZeroIndices(A, 0);
        Set<Integer> ax0C = getAxisZeroIndices(C, 0);

        Set<Integer> useless1 = SetOperations.intersection(ax1A, ax1B);
        Set<Integer> useless2 = SetOperations.intersection(ax0A, ax0C);
        Set<Integer> useless = SetOperations.union(useless1, useless2);

        if (!useless.isEmpty()) {
            A = deleteOnAxis(A, useless, 0);
            A = deleteOnAxis(A, useless, 1);
            B = deleteOnAxis(B, useless, 0);
            C = deleteOnAxis(C, useless, 1);
        }

        states = A.numRows();
        inputs = B.numCols();
        outputs = C.numRows();
    }

    public StateSpace negative() throws Exception {
        return new StateSpace(A, B, C.negative(), D.negative(), timebase);
    }

    public StateSpace add(Object object) throws Exception {
        SimpleMatrix A = this.A.copy();
        SimpleMatrix B = this.B.copy();
        SimpleMatrix C = this.C.copy();
        SimpleMatrix D = this.D.copy();
        Timebase timebase;

        if (object instanceof Number) {
            Equation eq = new Equation();
            eq.alias(D, "D", ((Number) object).doubleValue(), "other");
            eq.process("D = D + other");

            timebase = this.timebase == null ? null : this.timebase.copy();
        } else {
            StateSpace other = (StateSpace) object;

            if (inputs != other.inputs || outputs != other.outputs) {
                throw new Exception("Systems have different shapes.");
            }

            if (this.timebase == null && other.timebase != null) {
                timebase = other.timebase.copy();
            } else if ((other.timebase == null && this.timebase != null) || timebaseEqual(this, other)) {
                timebase = this.timebase == null ? null : this.timebase.copy();
            } else {
                throw new Exception("Systems have different sampling times");
            }

            A = this.A.concatColumns(new SimpleMatrix(this.A.numRows(), other.A.numCols()))
                    .concatRows(new SimpleMatrix(other.A.numRows(), this.A.numCols()).concatColumns(other.A));
            B = this.B.concatRows(other.B);
            C = this.C.concatColumns(other.C);
            D = this.D.plus(other.D);
        }

        return new StateSpace(A, B, C, D, timebase);
    }

    public StateSpace subtract(Object other) throws Exception {
        if (other instanceof Number) {
            return add(-((Number) other).doubleValue());
        } else {
            return add(((StateSpace) other).negative());
        }
    }

    public StateSpace multiply(Object object) throws Exception {
        SimpleMatrix A = this.A.copy();
        SimpleMatrix B = this.B.copy();
        SimpleMatrix C = this.C.copy();
        SimpleMatrix D = this.D.copy();
        Timebase timebase;

        if (object instanceof Number) {
            Equation eq = new Equation();
            eq.alias(C, "C", D, "D", ((Number) object).doubleValue(), "other");
            eq.process("C = C * other");
            eq.process("D = D * other");

            timebase = this.timebase == null ? null : this.timebase.copy();
        } else {
            StateSpace other = (StateSpace) object;

            if (inputs != other.outputs) {
                throw new Exception(
                        String.format("C = A * B: A has %d column(s) (input(s)), but B has %d row(s)\n(output(s)).",
                                inputs, outputs));
            }

            if (this.timebase == null && other.timebase != null) {
                timebase = other.timebase.copy();
            } else if ((other.timebase == null && this.timebase != null) || timebaseEqual(this, other)) {
                timebase = this.timebase == null ? null : this.timebase.copy();
            } else {
                throw new Exception("Systems have different sampling times");
            }

            A = other.A.concatColumns(new SimpleMatrix(other.A.numRows(), this.A.numCols()))
                    .concatRows(this.B.mult(this.C).concatColumns(this.A));
            B = other.B.concatRows(this.B.mult(other.D));
            C = this.D.mult(other.C).concatColumns(this.C);
            D = this.D.mult(other.D);
        }

        return new StateSpace(A, B, C, D, timebase);
    }

    public StateSpace rightMultiply(Object object) throws Exception {
        if (object instanceof Number) {
            SimpleMatrix A = this.A.copy();
            SimpleMatrix C = this.C.copy();
            SimpleMatrix B = this.B.copy();
            SimpleMatrix D = this.D.copy();

            Equation eq = new Equation();
            eq.alias(B, "B", D, "D", ((Number) object).doubleValue(), "other");
            eq.process("B = B * other");
            eq.process("D = D * other");

            return new StateSpace(A, B, C, D, timebase);
        }

        if (object instanceof StateSpace) {
            return ((StateSpace) object).multiply(this);
        }

        if (object instanceof SimpleMatrix) {
            SimpleMatrix X = (SimpleMatrix) object;
            SimpleMatrix C = X.mult(this.C);
            SimpleMatrix D = X.mult(this.D);

            return new StateSpace(A, B, C, D, timebase);
        }

        throw new Exception("can't interconnect system");
    }

    public StateSpace divide(Object other) throws Exception {
        throw new Exception("StateSpace.divide is not implemented yet.");
    }

    public StateSpace rightDivide(Object other) throws Exception {
        throw new Exception("StateSpace.rightDivide is not implemented yet.");
    }

    public ZMatrixRMaj evalFr(double omega) throws Exception {
        Complex_F64 s;

        if (isDTime(this, true)) {
            Double dt = timebase(this);
            assert(dt != null);
            s = new Complex_F64(Math.cos(omega * dt), Math.sin(omega * dt));
            if (omega * dt > Math.PI) {
                java.lang.System.err.println("evalFr: frequency evaluation above Nyquist frequency");
            }
        } else {
            s = new Complex_F64(0, omega);
        }

        return horner(s);
    }

    private ZMatrixRMaj convertSimpleMatToZMat(SimpleMatrix mat) {
        ZMatrixRMaj result = new ZMatrixRMaj(mat.numRows(), mat.numCols());

        for (int y = 0; y < result.numRows; y++) {
            for (int x = 0; x < result.numCols; x++) {
                result.setReal(y, x, mat.get(y, x));
            }
        }

        return result;
    }

    private ZMatrixRMaj horner(Complex_F64 s) throws Exception {
        ZMatrixRMaj resp = new ZMatrixRMaj(D.numRows(), D.numCols());
        ZMatrixRMaj A = convertSimpleMatToZMat(this.A);
        ZMatrixRMaj B = convertSimpleMatToZMat(this.B);
        ZMatrixRMaj C = convertSimpleMatToZMat(this.C);
        ZMatrixRMaj D = convertSimpleMatToZMat(this.D);

        ZMatrixRMaj eyeStates = identity(states);
        scale(s.real, s.imaginary, eyeStates);

        ZMatrixRMaj coef = A.copy();
        CommonOps_ZDRM.subtract(eyeStates, A, coef);

        ZMatrixRMaj solution = B.copy();
        LinearSolverLu_ZDRM solver = new LinearSolverLu_ZDRM(new LUDecompositionAlt_ZDRM());
        solver.setA(coef);
        solver.solve(B, solution);

        ZMatrixRMaj dot = new ZMatrixRMaj(C.numRows, solution.numCols);

        mult(C, solution, dot);
        CommonOps_ZDRM.add(dot, D, resp);

        return resp;
    }

    public FrequencyResponseData freqResp(List<Double> omega) throws Exception {
        FrequencyResponseData result = new FrequencyResponseData();

        int numFreqs = omega.size();

        result.mag = new double[outputs][inputs][numFreqs];
        result.phase = new double[outputs][inputs][numFreqs];
        result.omega = new ArrayList<>(omega);

        Collections.sort(result.omega);

        List<Complex_F64> cmplxFrqs = new ArrayList<>();

        if (isDTime(this, true)) {
            Double dt = timebase(this);
            assert(dt != null);
            for (double w : result.omega) {
                if (Math.abs(w) * dt > Math.PI) {
                    java.lang.System.err.println("freqResp: frequency evaluation above Nyquist frequency");
                }
                cmplxFrqs.add(new Complex_F64(Math.cos(w * dt), Math.sin(w * dt)));
            }
        } else {
            for (double w : result.omega) {
                cmplxFrqs.add(new Complex_F64(0, w));
            }
        }

        for (int kk = 0; kk < cmplxFrqs.size(); kk++) {
            Complex_F64 cmplxFreqsKK = cmplxFrqs.get(kk);

            ZMatrixRMaj fr = horner(cmplxFreqsKK);

            for (int y = 0; y < fr.numRows; y++) {
                for (int x = 0; x < fr.numCols; x++) {
                    Complex_F64 z = new Complex_F64();
                    fr.get(y, x, z);

                    ComplexPolar_F64 zp = new ComplexPolar_F64(z);

                    result.mag[y][x][kk] = zp.r;
                    result.phase[y][x][kk] = zp.theta;
                }
            }
        }

        return result;
    }

    public StateSpace sample(double ts, String method, Double alpha) throws Exception {
        if(!isCTime(false)) {
            throw new Exception("System must be continuous time system");
        }

        return contToDiscrete(A, B, C, D, ts, method, alpha);
    }

    private StateSpace contToDiscrete(SimpleMatrix a, SimpleMatrix b, SimpleMatrix c, SimpleMatrix d,
                                      double dt, String method, Double alpha) throws Exception {
        SimpleMatrix ad;
        SimpleMatrix bd;
        SimpleMatrix cd;
        SimpleMatrix dd;

        if (method.equals("gbt")) {
            if (alpha == null) {
                throw new Exception("Alpha parameter must be specified for the " +
                        "generalized bilinear transform (gbt) method");
            } else if (alpha < 0 || alpha > 1) {
                throw new Exception("Alpha parameter must be within the interval [" +
                        "0,1] for the gbt method");
            }
        }

        if (method.equals("gbt")) {
            SimpleMatrix ima = SimpleMatrix.identity(A.numRows()).minus(a.scale(alpha * dt));
            ad = ima.solve(SimpleMatrix.identity(a.numRows()).plus(a.scale((1.0 - alpha) * dt)));
            bd = ima.solve(b.scale(dt));

            cd = ima.transpose().solve(c.transpose());
            cd = cd.transpose();
            dd = d.plus(c.mult(bd).scale(alpha));
        } else if (method.equals("bilinear") || method.equals("tustin")) {
            return contToDiscrete(a, b, c, d, dt, "gbt", 0.5);
        } else if (method.equals("euler") || method.equals("forward_diff")) {
            return contToDiscrete(a, b, c, d, dt, "gbt", 0.0);
        } else if (method.equals("backward_diff")) {
            return contToDiscrete(a, b, c, d, dt, "gbt", 1.0);
        } else if (method.equals("zoh")) {
            ad = new SimpleMatrix(a.numRows(), a.numCols());
            bd = new SimpleMatrix(a.numRows(), a.numCols());

            SimpleMatrix emUpper = a.concatColumns(b);
            SimpleMatrix emLower = new SimpleMatrix(b.numCols(), a.numRows()).concatColumns(new SimpleMatrix(b.numCols(), b.numCols()));
            SimpleMatrix em = emUpper.concatRows(emLower).scale(dt);

            // Taylor Series
            int n = em.numRows();
            SimpleMatrix eA = SimpleMatrix.identity(n);
            SimpleMatrix trm = SimpleMatrix.identity(n);
            for (int k = 1; k < 20; k++) {
                trm = trm.mult(em).scale(1.0 / k);
                eA = eA.plus(trm);
            }

            Equation eq = new Equation();
            eq.alias(ad, "ad", bd, "bd", a.numRows(), "arow", a.numCols(), "acol", eA, "eA");
            eq.process("eA = eA(0:(arow-1), :)");
            eq.process("ad = eA(:, 0:(acol-1))");
            eq.process("bd = eA(:, acol:)");
            cd = c.copy();
            dd = d.copy();
        } else {
            throw new Exception(String.format("Unknown transformation method '%s'", method));
        }

        return new StateSpace(ad, bd, cd, dd, new Timebase(dt));
    }

    @Override
    public String toString() {
        return "A = {" + A + "}\n\n"
                + "B = {" + B + "}\n\n"
                + "C = {" + C + "}\n\n"
                + "D = {" + D + "}\n\n"
                + (timebase == null ? "" : timebase);
    }

    @Override
    public List<Complex_F64> pole() {
        return new ArrayList<>(A.eig().getEigenvalues());
    }

    @Override
    public List<Complex_F64> zero() throws Exception {
        if (states == 0) {
            return new ArrayList<>();
        }

        if (C.numRows() != D.numCols()) {
            throw new Exception("StateSpace.zero only supports " +
                    "systems with the same number of inputs as outputs.");
        }

        SimpleMatrix L = A.concatColumns(B).concatRows(C.concatColumns(D));

        int numRows = L.numRows();
        int numCols = L.numCols();

        double[][] LD = new double[numRows][numCols];
        double[][] MD = new double[numRows][numCols];

        for (int y = 0; y < numRows; y++) {
            for (int x = 0; x < numCols; x++) {
                LD[y][x] = L.get(y, x);
                if (x == y && y < C.numRows() && x < B.numCols()) MD[x][y] = 1;
            }
        }

        List<Complex_F64> result = new ArrayList<>();

        for (Complex_F64 z : new GEVDSolver(LD, MD).getEigenvalues()) {
            if (!Double.isInfinite(z.real) && !Double.isInfinite(z.imaginary)) {
                result.add(z);
            }
        }

        return result;
    }
}
