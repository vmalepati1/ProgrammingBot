package lib.math;

import org.ejml.dense.row.NormOps_DDRM;
import org.ejml.simple.SimpleMatrix;

import static lib.math.Special.epslon;

public class RiccatiSolver {

    private static class RicattiArguments {
        SimpleMatrix a;
        SimpleMatrix b;
        SimpleMatrix q;
        SimpleMatrix r;
        SimpleMatrix e;
        SimpleMatrix s;
        int m;
        int n;
        boolean genOrNot;

        public RicattiArguments(SimpleMatrix a, SimpleMatrix b, SimpleMatrix q, SimpleMatrix r, SimpleMatrix e, SimpleMatrix s, int m, int n, boolean genOrNot) {
            this.a = a;
            this.b = b;
            this.q = q;
            this.r = r;
            this.e = e;
            this.s = s;
            this.m = m;
            this.n = n;
            this.genOrNot = genOrNot;
        }
    }

    // TODO: Finish this
    public static void solveContinuousARE(SimpleMatrix a, SimpleMatrix b, SimpleMatrix q, SimpleMatrix r,
                                          SimpleMatrix e, SimpleMatrix s, boolean balanced) throws Exception {
        RicattiArguments ra = areValidateArguments(a, b, q, r, e, s, "care");
    }

    private static RicattiArguments areValidateArguments(SimpleMatrix a, SimpleMatrix b, SimpleMatrix q, SimpleMatrix r,
                                                         SimpleMatrix e, SimpleMatrix s, String eqType) throws Exception {
        if (!eqType.toLowerCase().contains("dare") && !eqType.toLowerCase().contains("care")) {
            throw new Exception("Equation type unknown. Only 'care' and 'dare' is understood");
        }

        a = a.copy();
        b = b.copy();
        q = q.copy();
        r = r.copy();

        a = asMatrixValidated(a, true, false);
        b = asMatrixValidated(b, true, false);
        q = asMatrixValidated(q, true, false);
        r = asMatrixValidated(r, true, false);

        int index = 0;
        for (SimpleMatrix m : new SimpleMatrix[]{a, q, r}) {
            if (m.numRows() != m.numCols()) {
                throw new Exception(String.format("Matrix %s should be square.", "aqr".charAt(index)));
            }
            index++;
        }

        int m = b.numRows();
        int n = b.numCols();

        if (m != a.numRows()) {
            throw new Exception("Matrix a and b should have the same number of rows.");
        }
        if (m != q.numRows()) {
            throw new Exception("Matrix a and q should have the same shape.");
        }
        if (n != r.numRows()) {
            throw new Exception("Matrix b and R should have the same number of cols.");
        }

        if (eqType.equals("care")) {
            int lastSIndex = r.svd().getSingularValues().length - 1;
            double minSv = r.svd().getSingleValue(lastSIndex);
            if (minSv == 0.0 || minSv < epslon(1.0) * NormOps_DDRM.normP1(r.getDDRM())) {
                throw new Exception("Matrix R is numerically singular.");
            }
        }

        boolean generalizedCase = (e != null || s != null);

        if (generalizedCase) {
            if (e != null) {
                e = e.copy();
                e = asMatrixValidated(e, true, false);
                if (e.numRows() != e.numCols()) {
                    throw new Exception("Matrix e should be square.");
                }
                if (m != e.numRows()) {
                    throw new Exception("Matrix a and e should have the same shape.");
                }

                int lastSIndex = e.svd().getSingularValues().length - 1;
                double minSv = e.svd().getSingleValue(lastSIndex);
                if (minSv == 0.0 || minSv < epslon(1.0) * NormOps_DDRM.normP1(r.getDDRM())) {
                    throw new Exception("Matrix R is numerically singular.");
                }
            }

            if (s != null) {
                s = s.copy();
                s = asMatrixValidated(s, true, false);
                if (s.numRows() != b.numRows() || s.numCols() != b.numCols()) {
                    throw new Exception("Matrix b and s should have the same shape.");
                }
            }
        }

        return new RicattiArguments(a, b, q, r, e, s, m, n, generalizedCase);
    }

    private static SimpleMatrix asMatrixValidated(SimpleMatrix a, boolean checkFinite, boolean sparseOk) throws Exception {
        if (!sparseOk) {
            a.convertToDense();
        }

        if (checkFinite) {
            for (double d : a.getDDRM().data) {
                if (Double.isInfinite(d) || Double.isNaN(d)) {
                    throw new Exception("Infinite or NaN entries are not supported");
                }
            }
        }

        return a;
    }

}
