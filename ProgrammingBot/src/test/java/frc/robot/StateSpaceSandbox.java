package frc.robot;

import lib.control.StateSpace;
import lib.utils.Timebase;
import org.junit.Test;
import org.nd4j.linalg.factory.Nd4j;

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
        double C3 = Math.pow(-Gr, 2) * Kt / (Kv * R * Math.pow(r, 2));
        double C4 = Gr * Kt / (R * r);

        double A[][] = {
                {(1 / m + Math.pow(rb, 2) / J) * C1, (1 / m - Math.pow(rb, 2) / J) * C3},
                {(1 / m - Math.pow(rb, 2) / J) * C1, (1 / m + Math.pow(rb, 2) / J) * C3}
        };

        double B[][] = {
                {(1 / m + Math.pow(rb, 2) / J) * C2, (1 / m - Math.pow(rb, 2) / J) * C4},
                {(1 / m - Math.pow(rb, 2) / J) * C2, (1 / m + Math.pow(rb, 2) / J) * C4}
        };

        double C[][] = {
                {1, 0},
                {0, 1}
        };

        double D[][] = {
                {0, 0},
                {0, 0}
        };

        try {
            StateSpace ss = new StateSpace(Nd4j.create(A), Nd4j.create(B), Nd4j.create(C), Nd4j.create(D), new Timebase());
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

}