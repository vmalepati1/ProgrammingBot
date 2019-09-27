package lib.system;

import org.ejml.simple.SimpleMatrix;

public class Models {

    public static class DcBrushedMotor {

        public double nominalVoltage;
        public double stallTorque;
        public double stallCurrent;
        public double freeCurrent;

        public double freeSpeed;

        public double R;

        public double kV;

        public double kT;

        public DcBrushedMotor(double nominalVoltage, double stallTorque, double stallCurrent,
                              double freeCurrent, double freeSpeed) {
            this.nominalVoltage = nominalVoltage;
            this.stallTorque = stallTorque;
            this.stallCurrent = stallCurrent;
            this.freeCurrent = freeCurrent;

            this.freeSpeed = freeSpeed / 60 * (2.0 * Math.PI);

            this.R = nominalVoltage / stallCurrent;

            this.kV = this.freeSpeed / (nominalVoltage - R * freeCurrent);

            this.kT = stallTorque / stallCurrent;
        }
    }

    public static DcBrushedMotor MOTOR_CIM =
            new DcBrushedMotor(12.0, 2.42, 133.0, 2.7, 5310.0);

    public static DcBrushedMotor MOTOR_MINI_CIM =
            new DcBrushedMotor(12.0, 1.41, 89.0, 3.0, 5840.0);

    public static DcBrushedMotor MOTOR_BAG =
            new DcBrushedMotor(12.0, 0.43, 53.0, 1.8, 13180.0);

    public static DcBrushedMotor MOTOR_775PRO =
            new DcBrushedMotor(12.0, 0.71, 134.0, 0.7, 18730.0);

    public static DcBrushedMotor MOTOR_AM_RS775_125 =
            new DcBrushedMotor(12.0, 0.28, 18.0, 1.6, 5800.0);

    public static DcBrushedMotor MOTOR_BB_RS775 =
            new DcBrushedMotor(12.0, 0.72, 97.0, 2.7, 13050.0);

    public static DcBrushedMotor MOTOR_AM_9015 =
            new DcBrushedMotor(12.0, 0.36, 71.0, 3.7, 14270.0);

    public static DcBrushedMotor MOTOR_BB_RS550 =
            new DcBrushedMotor(12.0, 0.38, 84.0, 0.4, 19000.0);

    public static DcBrushedMotor MOTOR_NEO =
            new DcBrushedMotor(12.0, 2.6, 105.0, 1.8, 5676.0);

    public static DcBrushedMotor gearbox(DcBrushedMotor motor, int numMotors) {
        return new DcBrushedMotor(motor.nominalVoltage,
                motor.stallTorque * numMotors,
                motor.stallCurrent,
                motor.freeCurrent,
                motor.freeSpeed / (2.0 * Math.PI) * 60);
    }

    public static StateSpace elevator(DcBrushedMotor motor, int numMotors, double m, double r, double G) throws Exception {
        DcBrushedMotor gearbox = gearbox(motor, numMotors);

        SimpleMatrix A = new SimpleMatrix(
                new double[][]{
                        {0, 1},
                        {0, -Math.pow(G, 2) * gearbox.kT / (gearbox.R * Math.pow(r, 2) * m * gearbox.kV)},
                }
        );

        SimpleMatrix B = new SimpleMatrix(
                new double[][]{
                        {0},
                        {G * gearbox.kT / (gearbox.R * r * m)},
                }
        );

        SimpleMatrix C = new SimpleMatrix(new double[][]{{1.0, 0.0}});
        SimpleMatrix D = new SimpleMatrix(new double[][]{{0.0}});

        return new StateSpace(A, B, C, D, null);
    }

    public static StateSpace flywheel(DcBrushedMotor motor, int numMotors, double J, double G) throws Exception {
        DcBrushedMotor gearbox = gearbox(motor, numMotors);

        SimpleMatrix A = new SimpleMatrix(
                new double[][]{
                        {-Math.pow(G, 2) * gearbox.kT / (gearbox.kV * gearbox.R * J)}
                }
        );

        SimpleMatrix B = new SimpleMatrix(
                new double[][]{
                        {G * gearbox.kT / (gearbox.R * J)},
                }
        );

        SimpleMatrix C = new SimpleMatrix(new double[][]{{1.0}});
        SimpleMatrix D = new SimpleMatrix(new double[][]{{0.0}});

        return new StateSpace(A, B, C, D, null);
    }

    public static StateSpace differentialDrive(DcBrushedMotor motor, int numMotors, double m,
                                               double r, double rb, double J, double Gl, double Gr) throws Exception {
        DcBrushedMotor gearbox = gearbox(motor, numMotors);

        double C1 = -Math.pow(Gl, 2) * gearbox.kT / (gearbox.kV * gearbox.R * Math.pow(r, 2));
        double C2 = Gl * gearbox.kT / (gearbox.R * r);
        double C3 = -Math.pow(Gr, 2) * gearbox.kT / (gearbox.kV * gearbox.R * Math.pow(r, 2));
        double C4 = Gr * gearbox.kT / (gearbox.R * r);

        SimpleMatrix A = new SimpleMatrix(
                new double[][]{
                        {0, 1, 0, 0},
                        {0, (1 / m + Math.pow(rb, 2) / J) * C1, 0, (1 / m - Math.pow(rb, 2) / J) * C3},
                        {0, 0, 0, 1},
                        {0, (1 / m - Math.pow(rb, 2) / J) * C1, 0, (1 / m + Math.pow(rb, 2) / J) * C3}
                }
        );

        SimpleMatrix B = new SimpleMatrix(
                new double[][]{
                        {0, 0},
                        {(1 / m + Math.pow(rb, 2) / J) * C2, (1 / m - Math.pow(rb, 2) / J) * C4},
                        {0, 0},
                        {(1 / m - Math.pow(rb, 2) / J) * C2, (1 / m + Math.pow(rb, 2) / J) * C4}
                }
        );

        SimpleMatrix C = new SimpleMatrix(
                new double[][]{
                        {1, 0, 0, 0},
                        {0, 0, 1, 0}
                }
        );

        SimpleMatrix D = new SimpleMatrix(
                new double[][]{
                        {0, 0},
                        {0, 0}
                }
        );

        return new StateSpace(A, B, C, D, null);
    }

    public static StateSpace singleJointedArm(DcBrushedMotor motor, int numMotors, double J, double G) throws Exception {
        DcBrushedMotor gearbox = gearbox(motor, numMotors);

        SimpleMatrix A = new SimpleMatrix(
                new double[][]{
                        {0, 1},
                        {0, -Math.pow(G, 2) * gearbox.kT / (gearbox.kV * gearbox.R * J)}
                }
        );

        SimpleMatrix B = new SimpleMatrix(
                new double[][]{
                        {0},
                        {G * gearbox.kT / (gearbox.R * J)}
                }
        );

        SimpleMatrix C = new SimpleMatrix(new double[][]{{1.0, 0.0}});
        SimpleMatrix D = new SimpleMatrix(new double[][]{{0.0}});

        return new StateSpace(A, B, C, D, null);
    }

}
