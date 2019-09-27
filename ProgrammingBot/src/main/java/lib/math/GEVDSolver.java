package lib.math;

import org.ejml.data.Complex_F64;

import static lib.math.Special.DOUBLE_EPSILON;
import static lib.math.Special.Epslon;

public class GEVDSolver {

    private int n;
    private double[] ar;
    private double[] ai;
    private double[] beta;
    private double[][] Z;

    public GEVDSolver(double[][] a, double[][] b) throws Exception {
        if (a == null)
            throw new Exception("Matrix A cannot be null.");

        if (b == null)
            throw new Exception("Matrix B cannot be null.");

        if (a.length != a[0].length)
            throw new Exception("Matrix is not a square matrix.");

        if (b.length != b[0].length)
            throw new Exception("Matrix is not a square matrix.");

        if (a.length != b.length || a[0].length != b[0].length)
            throw new Exception("Matrix dimensions do not match");


        n = a.length;
        ar = new double[n];
        ai = new double[n];
        beta = new double[n];
        Z = new double[n][n];
        double[][] A = a.clone();
        double[][] B = b.clone();

        boolean matz = true;
        int[] ierr = new int[1];

        // reduces A to upper Hessenberg form and B to upper
        // triangular form using orthogonal transformations
        qzhes(n, A, B, matz, Z);

        // reduces the Hessenberg matrix A to quasi-triangular form
        // using orthogonal transformations while maintaining the
        // triangular form of the B matrix.
        qzit(n, A, B, DOUBLE_EPSILON, matz, Z, ierr);

        // reduces the quasi-triangular matrix further, so that any
        // remaining 2-by-2 blocks correspond to pairs of complex
        // eigenvalues, and returns quantities whose ratios give the
        // generalized eigenvalues.
        qzval(n, A, B, ar, ai, beta, matz, Z);

        // computes the eigenvectors of the triangular problem and
        // transforms the results back to the original coordinate system.
        qzvec(n, A, B, ar, ai, beta, Z);
    }

    public double[] getRealAlphas() {
        return ar;
    }

    public double[] getImaginaryAlphas() {
        return ai;
    }

    public double[] getBetas() {
        return beta;
    }

    public boolean isSingular() {
        for (int i = 0; i < n; i++)
            if (beta[i] == 0)
                return true;
        return false;
    }

    public boolean isDegenerate() {
        for (int i = 0; i < n; i++)
            if (beta[i] == 0 && ar[i] == 0)
                return true;
        return false;
    }

    public double[] getRealEigenvalues() {
        // ((alfr+i*alfi)/beta)
        double[] eval = new double[n];
        for (int i = 0; i < n; i++)
            eval[i] = ar[i] / beta[i];
        return eval;
    }

    public double[] getImaginaryEigenvalues() {
        // ((alfr+i*alfi)/beta)
        double[] eval = new double[n];
        for (int i = 0; i < n; i++)
            eval[i] = ai[i] / beta[i];
        return eval;
    }

    public Complex_F64[] getEigenvalues() {
        Complex_F64[] eigenvalues = new Complex_F64[n];
        double[] realParts = getRealEigenvalues();
        double[] imaginaryParts = getImaginaryEigenvalues();

        for (int i = 0; i < n; i++) {
            eigenvalues[i] = new Complex_F64(realParts[i], imaginaryParts[i]);
        }

        return eigenvalues;
    }

    public double[][] getEigenvectors() {
        return Z;
    }

    public double[][] getDiagonalMatrix() {
        double[][] x = new double[n][n];

        for (int i = 0; i < n; i++)
        {
            for (int j = 0; j < n; j++)
                x[i][j] = 0.0;

            x[i][i] = ar[i] / beta[i];
            if (ai[i] > 0)
                x[i][i + 1] = ai[i] / beta[i];
                    else if (ai[i] < 0)
            x[i][i - 1] = ai[i] / beta[i];
        }

        return x;
    }

    private static int qzhes(int n, double[][] a, double[][] b, boolean matz, double[][] z)
    {
        int i, j, k, l;
        double r, s, t;
        int l1;
        double u1, u2, v1, v2;
        int lb, nk1;
        double rho;


        if (matz)
        {
            // If we are interested in computing the
            //  eigenvectors, set Z to identity(n,n)
            for (j = 0; j < n; ++j)
            {
                for (i = 0; i < n; ++i)
                    z[i][j] = 0.0;
                z[j][j] = 1.0;
            }
        }

        // Reduce b to upper triangular form
        if (n <= 1) return 0;
        for (l = 0; l < n - 1; ++l)
        {
            l1 = l + 1;
            s = 0.0;

            for (i = l1; i < n; ++i)
                s += (Math.abs(b[i][l]));

            if (s == 0.0) continue;
            s += (Math.abs(b[l][l]));
            r = 0.0;

            for (i = l; i < n; ++i)
            {
                // Computing 2nd power
                b[i][l] /= s;
                r += b[i][l] * b[i][l];
            }

            r = Math.copySign(Math.sqrt(r), b[l][l]);
            b[l][l] += r;
            rho = r * b[l][l];

            for (j = l1; j < n; ++j)
            {
                t = 0.0;
                for (i = l; i < n; ++i)
                    t += b[i][l] * b[i][j];
                t = -t / rho;
                for (i = l; i < n; ++i)
                    b[i][j] += t * b[i][l];
            }

            for (j = 0; j < n; ++j)
            {
                t = 0.0;
                for (i = l; i < n; ++i)
                    t += b[i][l] * a[i][j];
                t = -t / rho;
                for (i = l; i < n; ++i)
                    a[i][j] += t * b[i][l];
            }

            b[l][l] = -s * r;
            for (i = l1; i < n; ++i)
                b[i][l] = 0.0;
        }

        // Reduce a to upper Hessenberg form, while keeping b triangular
        if (n == 2) return 0;
        for (k = 0; k < n - 2; ++k)
        {
            nk1 = n - 2 - k;

            // for l=n-1 step -1 until k+1 do
            for (lb = 0; lb < nk1; ++lb)
            {
                l = n - lb - 2;
                l1 = l + 1;

                // Zero a(l+1,k)
                s = (Math.abs(a[l][k])) + (Math.abs(a[l1][k]));

                if (s == 0.0) continue;
                u1 = a[l][k] / s;
                u2 = a[l1][k] / s;
                r = Math.copySign(Math.sqrt(u1 * u1 + u2 * u2), u1);
                v1 = -(u1 + r) / r;
                v2 = -u2 / r;
                u2 = v2 / v1;

                for (j = k; j < n; ++j)
                {
                    t = a[l][j] + u2 * a[l1][j];
                    a[l][j] += t * v1;
                    a[l1][j] += t * v2;
                }

                a[l1][k] = 0.0;

                for (j = l; j < n; ++j)
                {
                    t = b[l][j] + u2 * b[l1][j];
                    b[l][j] += t * v1;
                    b[l1][j] += t * v2;
                }

                // Zero b(l+1,l)
                s = (Math.abs(b[l1][l1])) + (Math.abs(b[l1][l]));

                if (s == 0.0) continue;
                u1 = b[l1][l1] / s;
                u2 = b[l1][l] / s;
                r = Math.copySign(Math.sqrt(u1 * u1 + u2 * u2), u1);
                v1 = -(u1 + r) / r;
                v2 = -u2 / r;
                u2 = v2 / v1;

                for (i = 0; i <= l1; ++i)
                {
                    t = b[i][l1] + u2 * b[i][l];
                    b[i][l1] += t * v1;
                    b[i][l] += t * v2;
                }

                b[l1][l] = 0.0;

                for (i = 0; i < n; ++i)
                {
                    t = a[i][l1] + u2 * a[i][l];
                    a[i][l1] += t * v1;
                    a[i][l] += t * v2;
                }

                if (matz)
                {
                    for (i = 0; i < n; ++i)
                    {
                        t = z[i][l1] + u2 * z[i][l];
                        z[i][l1] += t * v1;
                        z[i][l] += t * v2;
                    }
                }
            }
        }

        return 0;
    }

    private static int qzit(int n, double[][] a, double[][] b, double eps1, boolean matz, double[][] z, int[] ierr)
    {
        int i, j, k, l = 0;
        double r, s, t, a1 = 0, a2 = 0, a3 = 0;
        int k1, k2, l1 = 0, ll;
        double u1, u2, u3;
        double v1, v2, v3;
        double a11 = 0, a12 = 0, a21 = 0, a22 = 0, a33 = 0, a34 = 0, a43 = 0, a44 = 0;
        double b11 = 0, b12, b22 = 0, b33, b34 = 0, b44;
        int na = 0, en, ld = 0;
        double ep;
        double sh = 0;
        int km1, lm1 = 0;
        double ani, bni;
        int ish = 0, itn, its = 0, enm2 = 0, lor1;
        double epsa, epsb, anorm = 0, bnorm = 0;
        int enorn;
        boolean notlas;


        ierr[0] = 0;

        for (i = 0; i < n; ++i)
        {
            ani = 0.0;
            bni = 0.0;

            if (i != 0)
                ani = (Math.abs(a[i][(i - 1)]));

            for (j = i; j < n; ++j)
            {
                ani += Math.abs(a[i][j]);
                bni += Math.abs(b[i][j]);
            }

            if (ani > anorm) anorm = ani;
            if (bni > bnorm) bnorm = bni;
        }

        if (anorm == 0.0) anorm = 1.0;
        if (bnorm == 0.0) bnorm = 1.0;

        ep = eps1;
        if (ep == 0.0)
        {
            // Use round-off level if eps1 is zero
            ep = Epslon(1.0);
        }

        epsa = ep * anorm;
        epsb = ep * bnorm;


        // Reduce a to quasi-triangular form][while keeping b triangular
        lor1 = 0;
        enorn = n;
        en = n - 1;
        itn = n * 30;

        // Begin QZ step

        int line = 60;
        boolean running = true;
        while(running)
        {
            switch(line)
            {
                case 60:
                    if (en <= 1) {
                        line = 1001;
                        break;
                    }

                    if (!matz) enorn = en + 1;

                    its = 0;
                    na = en - 1;
                    enm2 = na;

                case 70:
                    ish = 2;
                    boolean exitFlag = false;
                    // Check for convergence or reducibility.
                    for (ll = 0; ll <= en; ++ll)
                    {
                        lm1 = en - ll - 1;
                        l = lm1 + 1;

                        if (l + 1 == 1) {
                            exitFlag = true;
                            break;
                        }

                        if ((Math.abs(a[l][lm1])) <= epsa)
                            break;
                    }

                    if (exitFlag) {
                        line = 95;
                        break;
                    }

                case 90:
                    a[l][lm1] = 0.0;
                    if (l < na) {
                        line = 95;
                        break;
                    }

                    // 1-by-1 or 2-by-2 block isolated
                    en = lm1;
                    line = 60;
                    break;

                    // Check for small top of b
                case 95:
                    ld = l;

                case 100:
                    l1 = l + 1;
                    b11 = b[l][l];

                    if (Math.abs(b11) > epsb) {
                        line = 120;
                        break;
                    }

                    b[l][l] = 0.0;
                    s = (Math.abs(a[l][l]) + Math.abs(a[l1][l]));
                    u1 = a[l][l] / s;
                    u2 = a[l1][l] / s;
                    r = Math.copySign(Math.sqrt(u1 * u1 + u2 * u2), u1);
                    v1 = -(u1 + r) / r;
                    v2 = -u2 / r;
                    u2 = v2 / v1;

                    for (j = l; j < enorn; ++j)
                    {
                        t = a[l][j] + u2 * a[l1][j];
                        a[l][j] += t * v1;
                        a[l1][j] += t * v2;

                        t = b[l][j] + u2 * b[l1][j];
                        b[l][j] += t * v1;
                        b[l1][j] += t * v2;
                    }

                    if (l != 0)
                        a[l][lm1] = -a[l][lm1];

                    lm1 = l;
                    l = l1;
                    line = 90;
                    break;

                case 120:
                    a11 = a[l][l] / b11;
                    a21 = a[l1][l] / b11;
                    if (ish == 1) {
                        line = 140;
                        break;
                    }

                    // Iteration strategy
                    if (itn == 0) {
                        line = 1000;
                        break;
                    }
                    if (its == 10) {
                        line = 155;
                        break;
                    }

                    // Determine type of shift
                    b22 = b[l1][l1];
                    if (Math.abs(b22) < epsb) b22 = epsb;
                    b33 = b[na][na];
                    if (Math.abs(b33) < epsb) b33 = epsb;
                    b44 = b[en][en];
                    if (Math.abs(b44) < epsb) b44 = epsb;
                    a33 = a[na][na] / b33;
                    a34 = a[na][en] / b44;
                    a43 = a[en][na] / b33;
                    a44 = a[en][en] / b44;
                    b34 = b[na][en] / b44;
                    t = (a43 * b34 - a33 - a44) * .5;
                    r = t * t + a34 * a43 - a33 * a44;
                    if (r < 0.0) {
                        line = 150;
                        break;
                    }

                    // Determine single shift zero-th column of a
                    ish = 1;
                    r = Math.sqrt(r);
                    sh = -t + r;
                    s = -t - r;
                    if (Math.abs(s - a44) < Math.abs(sh - a44))
                        sh = s;

                    boolean exitFlag140 = false;
                    boolean exitFlag100 = false;

                    // Look for two consecutive small sub-diagonal elements of a.
                    for (ll = ld; ll + 1 <= enm2; ++ll)
                    {
                        l = enm2 + ld - ll - 1;

                        if (l == ld) {
                            exitFlag140 = true;
                            break;
                        }

                        lm1 = l - 1;
                        l1 = l + 1;
                        t = a[l + 1][l + 1];

                        if (Math.abs(b[l][l]) > epsb)
                            t -= sh * b[l][l];

                        if (Math.abs(a[l][lm1]) <= (Math.abs(t / a[l1][l])) * epsa) {
                            exitFlag100 = true;
                            break;
                        }
                    }

                    if (exitFlag140) {
                        line = 140;
                        break;
                    }

                    if (exitFlag100) {
                        line = 100;
                        break;
                    }

                case 140:
                    a1 = a11 - sh;
                    a2 = a21;
                    if (l != ld)
                        a[l][lm1] = -a[l][lm1];
                    line = 160;
                    break;

                    // Determine double shift zero-th column of a
                case 150:
                    a12 = a[l][l1] / b22;
                    a22 = a[l1][l1] / b22;
                    b12 = b[l][l1] / b22;
                    a1 = ((a33 - a11) * (a44 - a11) - a34 * a43 + a43 * b34 * a11) / a21 + a12 - a11 * b12;
                    a2 = a22 - a11 - a21 * b12 - (a33 - a11) - (a44 - a11) + a43 * b34;
                    a3 = a[l1 + 1][l1] / b22;
                    line = 160;
                    break;

                    // Ad hoc shift
                case 155:
                    a1 = 0.0;
                    a2 = 1.0;
                    a3 = 1.1605;

                case 160:
                    ++its;
                    --itn;

                    if (!matz) lor1 = ld;

                    // Main loop
                    for (k = l; k <= na; ++k)
                    {
                        notlas = k != na && ish == 2;
                        k1 = k + 1;
                        k2 = k + 2;

                        km1 = Math.max(k, l + 1) - 1; // Computing MAX
                        ll = Math.min(en, k1 + ish);  // Computing MIN

                        int fline = 165;
                        boolean frunning = true;
                        while (frunning) {
                            switch(fline) {
                                case 165:
                                    if (notlas) {
                                        fline = 190;
                                        break;
                                    }

                                    // Zero a(k+1,k-1)
                                    if (k == l) {
                                        fline = 170;
                                        break;
                                    }
                                    a1 = a[k][km1];
                                    a2 = a[k1][km1];

                                case 170:
                                    s = Math.abs(a1) + Math.abs(a2);
                                    if (s == 0.0) {
                                        fline = 70;
                                        break;
                                    }
                                    u1 = a1 / s;
                                    u2 = a2 / s;
                                    r = Math.copySign(Math.sqrt(u1 * u1 + u2 * u2), u1);
                                    v1 = -(u1 + r) / r;
                                    v2 = -u2 / r;
                                    u2 = v2 / v1;

                                    for (j = km1; j < enorn; ++j)
                                    {
                                        t = a[k][j] + u2 * a[k1][j];
                                        a[k][j] += t * v1;
                                        a[k1][j] += t * v2;

                                        t = b[k][j] + u2 * b[k1][j];
                                        b[k][j] += t * v1;
                                        b[k1][j] += t * v2;
                                    }

                                    if (k != l)
                                        a[k1][km1] = 0.0;
                                    fline = 240;
                                    break;

                                    // Zero a(k+1,k-1) and a(k+2,k-1)
                                case 190:
                                    if (k == l) {
                                        fline = 200;
                                        break;
                                    }
                                    a1 = a[k][km1];
                                    a2 = a[k1][km1];
                                    a3 = a[k2][km1];

                                case 200:
                                    s = Math.abs(a1) + Math.abs(a2) + Math.abs(a3);
                                    if (s == 0.0) {
                                        fline = 260;
                                        break;
                                    }
                                    u1 = a1 / s;
                                    u2 = a2 / s;
                                    u3 = a3 / s;
                                    r = Math.copySign(Math.sqrt(u1 * u1 + u2 * u2 + u3 * u3), u1);
                                    v1 = -(u1 + r) / r;
                                    v2 = -u2 / r;
                                    v3 = -u3 / r;
                                    u2 = v2 / v1;
                                    u3 = v3 / v1;

                                    for (j = km1; j < enorn; ++j)
                                    {
                                        t = a[k][j] + u2 * a[k1][j] + u3 * a[k2][j];
                                        a[k][j] += t * v1;
                                        a[k1][j] += t * v2;
                                        a[k2][j] += t * v3;

                                        t = b[k][j] + u2 * b[k1][j] + u3 * b[k2][j];
                                        b[k][j] += t * v1;
                                        b[k1][j] += t * v2;
                                        b[k2][j] += t * v3;
                                    }

                                    if (k == l) {
                                        fline = 220;
                                        break;
                                    }
                                    a[k1][km1] = 0.0;
                                    a[k2][km1] = 0.0;

                                    // Zero b(k+2,k+1) and b(k+2,k)
                                case 220:
                                    s = (Math.abs(b[k2][k2])) + (Math.abs(b[k2][k1])) + (Math.abs(b[k2][k]));
                                    if (s == 0.0) {
                                        fline = 240;
                                        break;
                                    }
                                    u1 = b[k2][k2] / s;
                                    u2 = b[k2][k1] / s;
                                    u3 = b[k2][k] / s;
                                    r = Math.copySign(Math.sqrt(u1 * u1 + u2 * u2 + u3 * u3), u1);
                                    v1 = -(u1 + r) / r;
                                    v2 = -u2 / r;
                                    v3 = -u3 / r;
                                    u2 = v2 / v1;
                                    u3 = v3 / v1;

                                    for (i = lor1; i < ll + 1; ++i)
                                    {
                                        t = a[i][k2] + u2 * a[i][k1] + u3 * a[i][k];
                                        a[i][k2] += t * v1;
                                        a[i][k1] += t * v2;
                                        a[i][k] += t * v3;

                                        t = b[i][k2] + u2 * b[i][k1] + u3 * b[i][k];
                                        b[i][k2] += t * v1;
                                        b[i][k1] += t * v2;
                                        b[i][k] += t * v3;
                                    }

                                    b[k2][k] = 0.0;
                                    b[k2][k1] = 0.0;

                                    if (matz)
                                    {
                                        for (i = 0; i < n; ++i)
                                        {
                                            t = z[i][k2] + u2 * z[i][k1] + u3 * z[i][k];
                                            z[i][k2] += t * v1;
                                            z[i][k1] += t * v2;
                                            z[i][k] += t * v3;
                                        }
                                    }

                                    // Zero b(k+1,k)
                                case 240:
                                    s = (Math.abs(b[k1][k1])) + (Math.abs(b[k1][k]));
                                    if (s == 0.0) {
                                        fline = 260;
                                        break;
                                    }
                                    u1 = b[k1][k1] / s;
                                    u2 = b[k1][k] / s;
                                    r = Math.copySign(Math.sqrt(u1 * u1 + u2 * u2), u1);
                                    v1 = -(u1 + r) / r;
                                    v2 = -u2 / r;
                                    u2 = v2 / v1;

                                    for (i = lor1; i < ll + 1; ++i)
                                    {
                                        t = a[i][k1] + u2 * a[i][k];
                                        a[i][k1] += t * v1;
                                        a[i][k] += t * v2;

                                        t = b[i][k1] + u2 * b[i][k];
                                        b[i][k1] += t * v1;
                                        b[i][k] += t * v2;
                                    }

                                    b[k1][k] = 0.0;

                                    if (matz)
                                    {
                                        for (i = 0; i < n; ++i)
                                        {
                                            t = z[i][k1] + u2 * z[i][k];
                                            z[i][k1] += t * v1;
                                            z[i][k] += t * v2;
                                        }
                                    }
                                default: frunning = false;
                                    break;
                            }
                        }
                    }

                    line = 70; // End QZ step
                    break;

                    // Set error -- all eigenvalues have not converged after 30*n iterations
                case 1000:
                    ierr[0] = en + 1;

                    // Save epsb for use by qzval and qzvec
                case 1001:
                    if (n > 1)
                        b[n - 1][0] = epsb;
                default: running = false;
                    break;
            }
        }

        return 0;
    }

    private static int qzval(int n, double[][] a, double[][] b, double[] alfr, double[] alfi, double[] beta, boolean matz, double[][] z)
    {
        int i = 0, j;
        int na, en, nn;
        double c = 0, d = 0, e = 0;
        double r, s = 0, t = 0;
        double a1 = 0, a2 = 0, u1, u2, v1, v2;
        double a11 = 0, a12 = 0, a21 = 0, a22 = 0;
        double b11 = 0, b12 = 0, b22 = 0;
        double di = 0, ei = 0;
        double an = 0, bn = 0;
        double cq = 0, dr = 0;
        double cz = 0, ti = 0, tr = 0;
        double a1i = 0, a2i = 0, a11i, a12i, a22i = 0, a11r, a12r, a22r = 0;
        double sqi = 0, ssi = 0, sqr = 0, szi = 0, ssr = 0, szr = 0;

        double epsb = b[n - 1][0];
        int isw = 1;


        // Find eigenvalues of quasi-triangular matrices.
        for (nn = 0; nn < n; ++nn)
        {
            en = n - nn - 1;
            na = en - 1;

            boolean running = true;
            int line = 405;
            while (running) {
                switch (line) {
                    case 405:
                        if (isw == 2) {
                            line = 505;
                            break;
                        }
                        if (en == 0) {
                            line = 410;
                            break;
                        }
                        if (a[en][na] != 0.0) {
                            line = 420;
                            break;
                        }

                        // 1-by-1 block][one real root
                    case 410:
                        alfr[en] = a[en][en];
                        if (b[en][en] < 0.0) {
                            alfr[en] = -alfr[en];
                        }
                        beta[en] = (Math.abs(b[en][en]));
                        alfi[en] = 0.0;
                        line = 510;
                        break;

                    // 2-by-2 block
                    case 420:
                        if (Math.abs(b[na][na]) <= epsb) {
                            line = 455;
                            break;
                        }
                        if (Math.abs(b[en][en]) > epsb) {
                            line = 430;
                            break;
                        }
                        a1 = a[en][en];
                        a2 = a[en][na];
                        bn = 0.0;
                        line = 435;
                        break;

                    case 430:
                        an = Math.abs(a[na][na]) + Math.abs(a[na][en]) + Math.abs(a[en][na]) + Math.abs(a[en][en]);
                        bn = Math.abs(b[na][na]) + Math.abs(b[na][en]) + Math.abs(b[en][en]);
                        a11 = a[na][na] / an;
                        a12 = a[na][en] / an;
                        a21 = a[en][na] / an;
                        a22 = a[en][en] / an;
                        b11 = b[na][na] / bn;
                        b12 = b[na][en] / bn;
                        b22 = b[en][en] / bn;
                        e = a11 / b11;
                        ei = a22 / b22;
                        s = a21 / (b11 * b22);
                        t = (a22 - e * b22) / b22;

                        if (Math.abs(e) <= Math.abs(ei)) {
                            line = 431;
                            break;
                        }

                        e = ei;
                        t = (a11 - e * b11) / b11;

                    case 431:
                        c = (t - s * b12) * .5;
                        d = c * c + s * (a12 - e * b12);
                        if (d < 0.0) {
                            line = 480;
                            break;
                        }

                        // Two real roots. Zero both a(en,na) and b(en,na)
                        e += c + Math.copySign(Math.sqrt(d), c);
                        a11 -= e * b11;
                        a12 -= e * b12;
                        a22 -= e * b22;

                        if (Math.abs(a11) + Math.abs(a12) < Math.abs(a21) + Math.abs(a22)) {
                            line = 432;
                            break;
                        }

                        a1 = a12;
                        a2 = a11;
                        line = 435;
                        break;

                    case 432:
                        a1 = a22;
                        a2 = a21;

                        // Choose and apply real z
                    case 435:
                        s = Math.abs(a1) + Math.abs(a2);
                        u1 = a1 / s;
                        u2 = a2 / s;
                        r = Math.copySign(Math.sqrt(u1 * u1 + u2 * u2), u1);
                        v1 = -(u1 + r) / r;
                        v2 = -u2 / r;
                        u2 = v2 / v1;

                        for (i = 0; i <= en; ++i) {
                            t = a[i][en] + u2 * a[i][na];
                            a[i][en] += t * v1;
                            a[i][na] += t * v2;

                            t = b[i][en] + u2 * b[i][na];
                            b[i][en] += t * v1;
                            b[i][na] += t * v2;
                        }

                        if (matz) {
                            for (i = 0; i < n; ++i) {
                                t = z[i][en] + u2 * z[i][na];
                                z[i][en] += t * v1;
                                z[i][na] += t * v2;
                            }
                        }

                        if (bn == 0.0) {
                            line = 475;
                            break;
                        }
                        if (an < Math.abs(e) * bn) {
                            line = 455;
                            break;
                        }
                        a1 = b[na][na];
                        a2 = b[en][na];
                        line = 460;
                        break;

                    case 455:
                        a1 = a[na][na];
                        a2 = a[en][na];

                        // Choose and apply real q
                    case 460:
                        s = Math.abs(a1) + Math.abs(a2);
                        if (s == 0.0) {
                            line = 475;
                            break;
                        }
                        u1 = a1 / s;
                        u2 = a2 / s;
                        r = Math.copySign(Math.sqrt(u1 * u1 + u2 * u2), u1);
                        v1 = -(u1 + r) / r;
                        v2 = -u2 / r;
                        u2 = v2 / v1;

                        for (j = na; j < n; ++j) {
                            t = a[na][j] + u2 * a[en][j];
                            a[na][j] += t * v1;
                            a[en][j] += t * v2;

                            t = b[na][j] + u2 * b[en][j];
                            b[na][j] += t * v1;
                            b[en][j] += t * v2;
                        }

                    case 475:
                        a[en][na] = 0.0;
                        b[en][na] = 0.0;
                        alfr[na] = a[na][na];
                        alfr[en] = a[en][en];

                        if (b[na][na] < 0.0)
                            alfr[na] = -alfr[na];

                        if (b[en][en] < 0.0)
                            alfr[en] = -alfr[en];

                        beta[na] = (Math.abs(b[na][na]));
                        beta[en] = (Math.abs(b[en][en]));
                        alfi[en] = 0.0;
                        alfi[na] = 0.0;
                        line = 505;
                        break;

                    // Two complex roots
                    case 480:
                        e += c;
                        ei = Math.sqrt(-d);
                        a11r = a11 - e * b11;
                        a11i = ei * b11;
                        a12r = a12 - e * b12;
                        a12i = ei * b12;
                        a22r = a22 - e * b22;
                        a22i = ei * b22;

                        if (Math.abs(a11r) + Math.abs(a11i) +
                                Math.abs(a12r) + Math.abs(a12i) <
                                Math.abs(a21) + Math.abs(a22r)
                                        + Math.abs(a22i)) {
                            line = 482;
                            break;
                        }

                        a1 = a12r;
                        a1i = a12i;
                        a2 = -a11r;
                        a2i = -a11i;
                        line = 485;
                        break;

                    case 482:
                        a1 = a22r;
                        a1i = a22i;
                        a2 = -a21;
                        a2i = 0.0;

                        // Choose complex z
                    case 485:
                        cz = Math.sqrt(a1 * a1 + a1i * a1i);
                        if (cz == 0.0) {
                            line = 487;
                            break;
                        }
                        szr = (a1 * a2 + a1i * a2i) / cz;
                        szi = (a1 * a2i - a1i * a2) / cz;
                        r = Math.sqrt(cz * cz + szr * szr + szi * szi);
                        cz /= r;
                        szr /= r;
                        szi /= r;
                        line = 490;
                        break;

                    case 487:
                        szr = 1.0;
                        szi = 0.0;

                    case 490:
                        if (an < (Math.abs(e) + ei) * bn) {
                            line = 492;
                            break;
                        }
                        a1 = cz * b11 + szr * b12;
                        a1i = szi * b12;
                        a2 = szr * b22;
                        a2i = szi * b22;
                        line = 495;
                        break;

                    case 492:
                        a1 = cz * a11 + szr * a12;
                        a1i = szi * a12;
                        a2 = cz * a21 + szr * a22;
                        a2i = szi * a22;

                        // Choose complex q
                    case 495:
                        cq = Math.sqrt(a1 * a1 + a1i * a1i);
                        if (cq == 0.0) {
                            line = 497;
                            break;
                        }
                        sqr = (a1 * a2 + a1i * a2i) / cq;
                        sqi = (a1 * a2i - a1i * a2) / cq;
                        r = Math.sqrt(cq * cq + sqr * sqr + sqi * sqi);
                        cq /= r;
                        sqr /= r;
                        sqi /= r;
                        line = 500;
                        break;

                    case 497:
                        sqr = 1.0;
                        sqi = 0.0;

                        // Compute diagonal elements that would result if transformations were applied
                    case 500:
                        ssr = sqr * szr + sqi * szi;
                        ssi = sqr * szi - sqi * szr;
                        i = 0;
                        tr = cq * cz * a11 + cq * szr * a12 + sqr * cz * a21 + ssr * a22;
                        ti = cq * szi * a12 - sqi * cz * a21 + ssi * a22;
                        dr = cq * cz * b11 + cq * szr * b12 + ssr * b22;
                        di = cq * szi * b12 + ssi * b22;
                        line = 503;
                        break;

                    case 502:
                        i = 1;
                        tr = ssr * a11 - sqr * cz * a12 - cq * szr * a21 + cq * cz * a22;
                        ti = -ssi * a11 - sqi * cz * a12 + cq * szi * a21;
                        dr = ssr * b11 - sqr * cz * b12 + cq * cz * b22;
                        di = -ssi * b11 - sqi * cz * b12;

                    case 503:
                        t = ti * dr - tr * di;
                        j = na;

                        if (t < 0.0)
                            j = en;

                        r = Math.sqrt(dr * dr + di * di);
                        beta[j] = bn * r;
                        alfr[j] = an * (tr * dr + ti * di) / r;
                        alfi[j] = an * t / r;
                        if (i == 0) {
                            line = 502;
                            break;
                        }

                    case 505:
                        isw = 3 - isw;

                    default:
                        running = false;
                        break;
                }
            }
        }

        b[n - 1][0] = epsb;

        return 0;
    }

    private static int qzvec(int n, double[][] a, double[][] b, double[] alfr, double[] alfi, double[] beta, double[][] z)
    {
        int i = 0, j, k, m;
        int na = 0, ii, en = 0, jj, nn, enm2;
        double d, q;
        double r = 0, s = 0, t = 0, w = 0, x = 0, y = 0, t1 = 0, t2 = 0, w1, x1 = 0, z1 = 0, di = 0;
        double ra, dr = 0, sa;
        double ti = 0, rr, tr = 0, zz = 0;
        double alfm, almi, betm, almr;

        double epsb = b[n - 1][0];
        int isw = 1;


        // for en=n step -1 until 1 do --
        for (nn = 0; nn < n; ++nn)
        {
            boolean running = true;
            int line = 705;
            while (running) {
                switch(line) {
                    case 705:
                        en = n - nn - 1;
                        na = en - 1;
                        if (isw == 2) {
                            line = 795;
                            break;
                        }
                        if (alfi[en] != 0.0) {
                            line = 710;
                            break;
                        }

                        // Real vector
                        m = en;
                        b[en][en] = 1.0;
                        if (na == -1) {
                            line = 800;
                            break;
                        }
                        alfm = alfr[m];
                        betm = beta[m];

                        // for i=en-1 step -1 until 1 do --
                        for (ii = 0; ii <= na; ++ii)
                        {
                            boolean frunning = true;
                            int fline = 625;
                            while (frunning) {
                                switch (fline) {
                                    case 625:
                                        i = en - ii - 1;
                                        w = betm * a[i][i] - alfm * b[i][i];
                                        r = 0.0;

                                        for (j = m; j <= en; ++j)
                                            r += (betm * a[i][j] - alfm * b[i][j]) * b[j][en];

                                        if (i == 0 || isw == 2) {
                                            fline = 630;
                                            break;
                                        }

                                        if (betm * a[i][i - 1] == 0.0) {
                                            fline = 630;
                                            break;
                                        }

                                        zz = w;
                                        s = r;
                                        fline = 690;
                                        break;

                                    case 630:
                                        m = i;
                                        if (isw == 2) {
                                            fline = 640;
                                            break;
                                        }

                                        // Real 1-by-1 block
                                        t = w;
                                        if (w == 0.0)
                                            t = epsb;
                                        b[i][en] = -r / t;
                                        fline = 700;
                                        break;

                                    // Real 2-by-2 block
                                    case 640:
                                        x = betm * a[i][i + 1] - alfm * b[i][i + 1];
                                        y = betm * a[i + 1][i];
                                        q = w * zz - x * y;
                                        t = (x * s - zz * r) / q;
                                        b[i][en] = t;
                                        if (Math.abs(x) <= Math.abs(zz)) {
                                            fline = 650;
                                            break;
                                        }
                                        b[i + 1][en] = (-r - w * t) / x;
                                        fline = 690;
                                        break;

                                    case 650:
                                        b[i + 1][en] = (-s - y * t) / zz;

                                    case 690:
                                        isw = 3 - isw;

                                    default: frunning = false;
                                        break;
                                }
                            }
                        }
                        // End real vector
                        line = 800;
                        break;

                        // Complex vector
                    case 710:
                        m = na;
                        almr = alfr[m];
                        almi = alfi[m];
                        betm = beta[m];

                        // last vector component chosen imaginary so that eigenvector matrix is triangular
                        y = betm * a[en][na];
                        b[na][na] = -almi * b[en][en] / y;
                        b[na][en] = (almr * b[en][en] - betm * a[en][en]) / y;
                        b[en][na] = 0.0;
                        b[en][en] = 1.0;
                        enm2 = na;
                        if (enm2 == 0) {
                            line = 795;
                            break;
                        }

                        // for i=en-2 step -1 until 1 do --
                        for (ii = 0; ii < enm2; ++ii)
                        {
                            i = na - ii - 1;
                            w = betm * a[i][i] - almr * b[i][i];
                            w1 = -almi * b[i][i];
                            ra = 0.0;
                            sa = 0.0;

                            for (j = m; j <= en; ++j)
                            {
                                x = betm * a[i][j] - almr * b[i][j];
                                x1 = -almi * b[i][j];
                                ra = ra + x * b[j][na] - x1 * b[j][en];
                                sa = sa + x * b[j][en] + x1 * b[j][na];
                            }

                            boolean frunning = true;
                            int fline = 665;
                            while (frunning) {
                                switch(fline) {
                                    case 665:
                                        if (i == 0 || isw == 2) {
                                            fline = 770;
                                            break;
                                        }
                                        if (betm * a[i][i - 1] == 0.0) {
                                            fline = 770;
                                            break;
                                        }

                                        zz = w;
                                        z1 = w1;
                                        r = ra;
                                        s = sa;
                                        isw = 2;
                                        fline = 790;
                                        break;

                                    case 770:
                                        m = i;
                                        if (isw == 2) {
                                            fline = 780;
                                            break;
                                        }

                                        // Complex 1-by-1 block
                                        tr = -ra;
                                        ti = -sa;

                                    case 773:
                                        dr = w;
                                        di = w1;

                                        // Complex divide (t1,t2) = (tr,ti) / (dr,di)
                                    case 775:
                                        if (Math.abs(di) > Math.abs(dr)) {
                                            fline = 777;
                                            break;
                                        }
                                        rr = di / dr;
                                        d = dr + di * rr;
                                        t1 = (tr + ti * rr) / d;
                                        t2 = (ti - tr * rr) / d;

                                        if (isw == 1) {
                                            fline = 787;
                                            break;
                                        }

                                        if (isw == 2) {
                                            fline = 782;
                                            break;
                                        }

                                    case 777:
                                        rr = dr / di;
                                        d = dr * rr + di;
                                        t1 = (tr * rr + ti) / d;
                                        t2 = (ti * rr - tr) / d;
                                        if (isw == 1) {
                                            fline = 787;
                                            break;
                                        }

                                        if (isw == 2) {
                                            fline = 782;
                                            break;
                                        }

                                        // Complex 2-by-2 block
                                    case 780:
                                        x = betm * a[i][i + 1] - almr * b[i][i + 1];
                                        x1 = -almi * b[i][i + 1];
                                        y = betm * a[i + 1][i];
                                        tr = y * ra - w * r + w1 * s;
                                        ti = y * sa - w * s - w1 * r;
                                        dr = w * zz - w1 * z1 - x * y;
                                        di = w * z1 + w1 * zz - x1 * y;
                                        if (dr == 0.0 && di == 0.0)
                                            dr = epsb;
                                        fline = 775;
                                        break;

                                    case 782:
                                        b[i + 1][na] = t1;
                                        b[i + 1][en] = t2;
                                        isw = 1;
                                        if (Math.abs(y) > Math.abs(w) + Math.abs(w1)) {
                                            fline = 785;
                                            break;
                                        }
                                        tr = -ra - x * b[(i + 1)][na] + x1 * b[(i + 1)][en];
                                        ti = -sa - x * b[(i + 1)][en] - x1 * b[(i + 1)][na];
                                        fline = 773;
                                        break;

                                    case 785:
                                        t1 = (-r - zz * b[(i + 1)][na] + z1 * b[(i + 1)][en]) / y;
                                        t2 = (-s - zz * b[(i + 1)][en] - z1 * b[(i + 1)][na]) / y;

                                    case 787:
                                        b[i][na] = t1;
                                        b[i][en] = t2;

                                    case 790:
                                        System.out.println("Hello");

                                    default: frunning = false;
                                        break;
                                }
                            }
                        }

                        // End complex vector
                    case 795:
                        isw = 3 - isw;

                    default: running = false;
                        break;
                }
            }
        }

        // End back substitution. Transform to original coordinate system.
        for (jj = 0; jj < n; ++jj)
        {
            j = n - jj - 1;

            for (i = 0; i < n; ++i)
            {
                zz = 0.0;
                for (k = 0; k <= j; ++k)
                    zz += z[i][k] * b[k][j];
                z[i][j] = zz;
            }
        }

        // Normalize so that modulus of largest component of each vector is 1.
        // (isw is 1 initially from before)
        for (j = 0; j < n; ++j)
        {
            d = 0.0;
            boolean running = true;
            int line = 915;
            while (running) {
                switch(line) {
                    case 915:
                        if (isw == 2) {
                            line = 920;
                            break;
                        }
                        if (alfi[j] != 0.0) {
                            line = 945;
                            break;
                        }

                        for (i = 0; i < n; ++i)
                        {
                            if ((Math.abs(z[i][j])) > d)
                                d = (Math.abs(z[i][j]));
                        }

                        for (i = 0; i < n; ++i)
                            z[i][j] /= d;

                        line = 950;
                        break;

                    case 920:
                        for (i = 0; i < n; ++i)
                        {
                            r = Math.abs(z[i][j - 1]) + Math.abs(z[i][j]);
                            if (r != 0.0)
                            {
                                // Computing 2nd power
                                double u1 = z[i][j - 1] / r;
                                double u2 = z[i][j] / r;
                                r *= Math.sqrt(u1 * u1 + u2 * u2);
                            }
                            if (r > d)
                                d = r;
                        }

                        for (i = 0; i < n; ++i)
                        {
                            z[i][j - 1] /= d;
                            z[i][j] /= d;
                        }

                    case 945:
                        isw = 3 - isw;

                    default: running = false;
                        break;
                }
            }
        }

        return 0;
    }

}
