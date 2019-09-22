package lib.control;

import lib.utils.Nd4jCustomOps;
import lib.utils.Timebase;
import org.apache.commons.math3.complex.Complex;
import org.nd4j.linalg.api.ndarray.INDArray;
import org.nd4j.linalg.factory.Nd4j;
import org.nd4j.util.SetUtils;

import java.util.Arrays;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;

public class StateSpace extends LTI {

    private INDArray A;
    private INDArray B;
    private INDArray C;
    private INDArray D;
    private long states;

    public StateSpace(INDArray A, INDArray B, INDArray C, INDArray D, Timebase timebase) throws Exception {
        super(preprocessFeedthroughMatrix(D, B, C).shape()[1], D.shape()[0], timebase);

        A = A.dup();
        B = B.dup();
        C = C.dup();

        this.A = convertToSSMatrix(A);
        this.B = convertToSSMatrix(B);
        this.C  = convertToSSMatrix(C);
        this.D = D;
        this.states = A.shape()[1];

        if (states == 0) {
            A.reshape(0, 0);
            B.reshape(0, inputs);
            C.reshape(outputs, 0);
        }

        if (states != A.shape()[0]) {
            throw new Exception("A must be square.");
        }
        if (states != B.shape()[0]) {
            throw new Exception("A and B must have the same number of rows.");
        }
        if (states != C.shape()[1]) {
            throw new Exception("A and C must have the same number of columns.");
        }
        if (inputs != B.shape()[1]) {
            throw new Exception("B and D must have the same number of columns.");
        }
        if (outputs != C.shape()[0]) {
            throw new Exception("C and D must have the same number of rows.");
        }

        removeUselessStates();
    }

    private static INDArray preprocessFeedthroughMatrix(INDArray D, INDArray B, INDArray C) throws Exception {
        D = D.dup();

        if (D.isScalar() && D.getDouble(0) == 0 && B.shape()[1] > 0 & C.shape()[0] > 0) {
            D.broadcast(C.shape()[0], B.shape()[1]);
        }

        D = convertToSSMatrix(D);

        return D;
    }

    private static INDArray convertToSSMatrix(INDArray data) throws Exception {
        INDArray arr = data.dup();

        int ndim = arr.rank();
        long[] shape = arr.shape();

        if (ndim > 2) {
            throw new Exception("State-space matrix must be 2-dimensional");
        } else if ((ndim == 2 && Arrays.equals(shape, new long[]{1, 0}))
                    || (ndim == 1 && shape[0] == 0)) {
            shape = new long[]{0, 0};
        } else if (ndim == 1) {
            shape = new long[]{1, shape[0]};
        } else if (ndim == 0) {
            shape = new long[]{1, 1};
        }

        return arr.reshape(shape);
    }

    private boolean[] matrixAny(INDArray matrix, int axis) {
        INDArray a = matrix.dup();

        if (axis == 0) {
            a = a.transpose();
        }

        boolean[] result = new boolean[a.rows()];

        for (int i = 0; i < a.rows(); i++) {
            INDArray row = a.getRow(i);

            result[i] = !row.any();
        }

        return result;
    }

    private void removeUselessStates() {
        Set<Integer> ax1A =
                Arrays.stream(Nd4j.where(Nd4j.create(matrixAny(A, 1)), null, null)[0].toIntVector())
                        .boxed().collect(Collectors.toSet());

        Set<Integer> ax1B =
                Arrays.stream(Nd4j.where(Nd4j.create(matrixAny(B, 1)), null, null)[0].toIntVector())
                        .boxed().collect(Collectors.toSet());

        Set<Integer> ax0A =
                Arrays.stream(Nd4j.where(Nd4j.create(matrixAny(A, 0)), null, null)[0].toIntVector())
                        .boxed().collect(Collectors.toSet());

        Set<Integer> ax0C =
                Arrays.stream(Nd4j.where(Nd4j.create(matrixAny(C, 0)), null, null)[0].toIntVector())
                        .boxed().collect(Collectors.toSet());

        Set<Integer> useless1 = SetUtils.intersection(ax1A, ax1B);
        Set<Integer> useless2 = SetUtils.intersection(ax0A, ax0C);
        Set<Integer> useless = SetUtils.union(useless1, useless2);

        for (int i : useless) {
            A = Nd4jCustomOps.delete(0, A, i);
            A = Nd4jCustomOps.delete(1, A, i);
            B = Nd4jCustomOps.delete(0, B, i);
            C = Nd4jCustomOps.delete(1, C, i);
        }

        states = A.shape()[0];
        inputs = B.shape()[1];
        outputs = C.shape()[0];

        System.out.println(A.toString());
        System.out.println(B.toString());
        System.out.println(C.toString());
        System.out.println(D.toString());
    }

    @Override
    public List<Complex> pole() {
        return null;
    }
}
