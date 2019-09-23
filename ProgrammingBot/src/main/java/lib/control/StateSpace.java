package lib.control;

import lib.utils.SetOperations;
import lib.utils.Timebase;
import org.ejml.data.Complex_F32;
import org.ejml.data.Complex_F64;
import org.ejml.data.ZMatrix;
import org.ejml.equation.Equation;
import org.ejml.ops.ComplexMath_F64;
import org.ejml.simple.SimpleMatrix;

import java.util.Collection;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class StateSpace extends LTI {

    private SimpleMatrix A;
    private SimpleMatrix B;
    private SimpleMatrix C;
    private SimpleMatrix D;
    private int states;

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

            Equation eq = new Equation();
            eq.alias(D, "A", this.D, "B", other.D, "C");
            eq.process("A = B + C");
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
            } else if ((other.timebase == null && this.timebase != null) || LTI.timebaseEqual(this, other)) {
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

    public SimpleMatrix evalFr(double omega) throws Exception {
        Complex_F64 s;

        if (isDTime(true)) {
            Double dt = timebase(this);
            assert(dt != null);
            s = new Complex_F64(Math.cos(omega * dt), Math.sin(omega * dt));
            if (omega * dt > Math.PI) {
                System.err.println("evalFr: frequency evaluation above Nyquist frequency");
            }
        } else {
            s = new Complex_F64(0, omega);
        }

        // ZMatrix
        return horner(s);
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
    public List<Double> pole() {
        return null;
    }
}
