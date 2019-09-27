package frc.robot;

import lib.control.StateSpace;
import org.ejml.data.Complex_F64;
import org.ejml.simple.SimpleMatrix;
import org.junit.Test;

public class StateSpaceSandbox {

    @Test
    public void stateSpaceTesting() {
        double Kt = 2.42 / 133.0;
        double R = 12.0 / 133.0;
        double r = 0.08255 / 2.0;
        double Gl = 60.0 / 11.0;
        double Gr = 60.0 / 11.0;
        double rb = 0.59055 / 2.0;
        double J = 6.0;
        double m = 52;
        double Kv = 5310.0 / (12.0 - R * 2.7);

        double C1 = -Kt / (Kv * R * Math.pow(r, 2));
        double C2 = Gl * Kt / (R * r);
        double C3 = -Math.pow(Gr, 2) * Kt / (Kv * R * Math.pow(r, 2));
        double C4 = Gr * Kt / (R * r);

        double AD[][] = {
                {(1 / m + Math.pow(rb, 2) / J) * C1, (1 / m - Math.pow(rb, 2) / J) * C3},
                {(1 / m - Math.pow(rb, 2) / J) * C1, (1 / m + Math.pow(rb, 2) / J) * C3}
        };

        double BD[][] = {
                {(1 / m + Math.pow(rb, 2) / J) * C2, (1 / m - Math.pow(rb, 2) / J) * C4},
                {(1 / m - Math.pow(rb, 2) / J) * C2, (1 / m + Math.pow(rb, 2) / J) * C4}
        };

        double CD[][] = {
                {1, 0},
                {0, 1}
        };

        double DD[][] = {
                {0, 0},
                {0, 0}
        };

        double[][] MD = {
                {1, 0, 0, 0},
                {0, 1, 0, 0},
                {0, 0, 0, 0},
                {0, 0, 0, 0},
        };

        SimpleMatrix A = new SimpleMatrix(AD);
        SimpleMatrix B = new SimpleMatrix(BD);
        SimpleMatrix C = new SimpleMatrix(CD);
        SimpleMatrix D = new SimpleMatrix(DD);

        try {
            StateSpace ss = new StateSpace(A, B, C, D, null);

            for (Complex_F64 z : ss.zero()) {
                System.out.println(z);
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

}
