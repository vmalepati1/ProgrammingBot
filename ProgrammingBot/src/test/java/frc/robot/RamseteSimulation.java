package frc.robot;

import lib.graph.LinePlot;
import lib.motionControl.Pose2d;
import lib.motionControl.RamseteController;
import lib.motionControl.RamseteTuple;
import lib.system.Models;
import lib.system.StateSpace;
import lib.system.System;
import org.ejml.simple.SimpleMatrix;

import java.awt.*;
import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import static lib.system.Models.MOTOR_CIM;

/**
 * @author Vikas Malepati
 */
public class RamseteSimulation {

    private static class Drivetrain extends System {

        public boolean inLowGear = false;
        public double rb;

        public Drivetrain(double dt) throws Exception {
            super(new SimpleMatrix(new double[][]{{-12.0}, {-12.0}}),
                    new SimpleMatrix(new double[][]{{12.0}, {12.0}}),
                    dt, new SimpleMatrix(2, 1),
                    new SimpleMatrix(2, 1));
        }

        @Override
        protected StateSpace createModel(SimpleMatrix states, SimpleMatrix inputs) throws Exception {
            double numMotors = 3.0;

            double Glow = 60.0 / 11.0;
            double Ghigh = 60.0 / 11.0;

            double m = 52;
            double r = 0.08255 / 2.0;
            this.rb = 0.59055 / 2.0;
            double J = 6.0;

            double Gl = 0.0;
            double Gr = 0.0;

            if (inLowGear) {
                Gl = Glow;
                Gr = Glow;
            } else {
                Gl = Ghigh;
                Gr = Ghigh;
            }

            return drivetrain(MOTOR_CIM, numMotors, m, r, rb, J, Gl, Gr);
        }

        @Override
        protected void designControllerObserver() {
            // Have to use Matlab or Python to get LQR and Kalman gain matrix for now

            SimpleMatrix lqrMatrix = new SimpleMatrix(
                    new double[][]{
                            {7.11732344, -0.35178656},
                            {-0.35178656, 7.11732344},
                    }
            );

            designLQR(lqrMatrix);

            designTwoStateFeedforward(null, null);

            double qVel = 1.0;
            double rVel = 0.01;

            SimpleMatrix kalmanGainMatrix = new SimpleMatrix(
                    new double[][]{
                            {9.99900017e-01, -3.10515467e-10},
                            {-3.10515467e-10, 9.99900017e-01}
                    }
            );

            designKalmanFilter(new SimpleMatrix(new double[][]{{qVel, qVel}}), new SimpleMatrix(new double[][]{{rVel, rVel}}), kalmanGainMatrix);
        }

        private StateSpace drivetrain(Models.DcBrushedMotor motor, double numMotors, double m, double r,
                                      double rb, double J, double Gl, double Gr) throws Exception {
            Models.DcBrushedMotor gearbox = Models.gearbox(motor, numMotors);

            double C1 = -Math.pow(Gl, 2) * gearbox.kT / (gearbox.kV * gearbox.R * Math.pow(r, 2));
            double C2 = Gl * gearbox.kT / (gearbox.R * r);
            double C3 = -Math.pow(Gr, 2) * gearbox.kT / (gearbox.kV * gearbox.R * Math.pow(r, 2));
            double C4 = Gr * gearbox.kT / (gearbox.R * r);

            SimpleMatrix A = new SimpleMatrix(
                    new double[][]{
                            {(1 / m + Math.pow(rb, 2) / J) * C1, (1 / m - Math.pow(rb, 2) / J) * C3},
                            {(1 / m - Math.pow(rb, 2) / J) * C1, (1 / m + Math.pow(rb, 2) / J) * C3},
                    }
            );

            SimpleMatrix B = new SimpleMatrix(
                    new double[][]{
                            {(1 / m + Math.pow(rb, 2) / J) * C2, (1 / m - Math.pow(rb, 2) / J) * C4},
                            {(1 / m - Math.pow(rb, 2) / J) * C2, (1 / m + Math.pow(rb, 2) / J) * C4},
                    }
            );

            SimpleMatrix C = new SimpleMatrix(
                    new double[][]{
                            {1, 0},
                            {0, 1}
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

        // For testing purposes
        /*
        @Override
        protected void initDefaultCommand() {

        }
        */
    }

    private static double[] getDiffVels(double v, double omega, double d) {
        return new double[]{v - omega * d / 2.0, v + omega * d / 2.0};
    }

    public static void main(String[] args) {
        double dt = 0.02;
        Drivetrain drivetrain = null;

        try {
            drivetrain = new Drivetrain(dt);
        } catch (Exception e) {
            e.printStackTrace();
        }

        String csvFile = "C:\\Users\\User\\Documents\\Programming\\Python\\RamseteTesting\\ramsete_traj.csv";
        BufferedReader br = null;
        String line = "";
        String cvsSplitBy = ",";

        List<Double> t = new ArrayList<>();
        List<Double> xprof = new ArrayList<>();
        List<Double> yprof = new ArrayList<>();
        List<Double> thetaprof = new ArrayList<>();
        List<Double> vprof = new ArrayList<>();
        List<Double> omegaprof = new ArrayList<>();

        try {

            br = new BufferedReader(new FileReader(csvFile));
            // skip first line
            br.readLine();
            while ((line = br.readLine()) != null) {

                // use comma as separator
                String[] trajectoryData = line.split(cvsSplitBy);

                t.add(Double.parseDouble(trajectoryData[0]));
                xprof.add(Double.parseDouble(trajectoryData[1]));
                yprof.add(Double.parseDouble(trajectoryData[2]));
                thetaprof.add(Double.parseDouble(trajectoryData[3]));
                vprof.add(Double.parseDouble(trajectoryData[4]));
                omegaprof.add(Double.parseDouble(trajectoryData[5]));
            }

        } catch (FileNotFoundException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        } finally {
            if (br != null) {
                try {
                    br.close();
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        }

        Pose2d pose = new Pose2d(xprof.get(0) + 0.5, yprof.get(0) + 0.5, Math.PI);
        Pose2d desiredPose = new Pose2d();

        RamseteController controller = new RamseteController();

        double vl = Double.POSITIVE_INFINITY;
        double vr = Double.POSITIVE_INFINITY;

        double[] xRec = new double[t.size() - 1];
        double[] yRec = new double[t.size() - 1];
        double[] thetaRec = new double[t.size() - 1];
        double[] vlRec = new double[t.size() - 1];
        double[] vlRefRec = new double[t.size() - 1];
        double[] vrRec = new double[t.size() - 1];
        double[] vrRefRec = new double[t.size() - 1];
        double[] ulRec = new double[t.size() - 1];
        double[] urRec = new double[t.size() - 1];

        int i = 0;

        while (i < t.size() - 1) {
            desiredPose.x = xprof.get(i);
            desiredPose.y = yprof.get(i);
            desiredPose.theta = thetaprof.get(i);

            RamseteTuple tuple = controller.ramsete(desiredPose, vprof.get(i), omegaprof.get(i), pose);
            double vref = tuple.v;
            double omegaref = tuple.omega;
            double[] diffVels = getDiffVels(vref, omegaref, drivetrain.rb * 2.0);
            double vlref = diffVels[0];
            double vrref = diffVels[1];
            SimpleMatrix nextR = new SimpleMatrix(new double[][]{{vlref}, {vrref}});
            drivetrain.update(nextR);
            double vc = (drivetrain.x.get(0, 0) + drivetrain.x.get(1, 0)) / 2.0;
            double omega = (drivetrain.x.get(1, 0) - drivetrain.x.get(0, 0)) / (2.0 * drivetrain.rb);
            double[] diffVels1 = getDiffVels(vc, omega, drivetrain.rb * 2.0);
            vl = diffVels1[0];
            vr = diffVels1[1];

            pose.x += vc * Math.cos(pose.theta) * dt;
            pose.y += vc * Math.sin(pose.theta) * dt;
            pose.theta += omega * dt;

            vlRefRec[i] = vlref;
            vrRefRec[i] = vrref;
            xRec[i] = pose.x;
            yRec[i] = pose.y;
            thetaRec[i] = pose.theta;
            vlRec[i] = vl;
            vrRec[i] = vr;
            ulRec[i] = drivetrain.u.get(0, 0);
            urRec[i] = drivetrain.u.get(1, 0);

            if (i < t.size() - 1) {
                i += 1;
            }
        }

        double[] xProfArray = new double[xprof.size()];
        double[] yProfArray = new double[yprof.size()];
        double[] thetaProfArray = new double[thetaprof.size()];

        for (int j = 0; j < xprof.size(); j++) {
            xProfArray[j] = xprof.get(j);
            yProfArray[j] = yprof.get(j);
            thetaProfArray[j] = thetaprof.get(j);
        }

        {
            LinePlot plot = new LinePlot(xRec, yRec, Color.RED);
            plot.addData(xProfArray, yProfArray, Color.BLUE);
            plot.setXLabel("x (m)");
            plot.setYLabel("y (m)");
            plot.setTitle("Ramsete Controller vs Reference Trajectory");
            plot.setYTic(Collections.min(yprof) - 1, Collections.max(yprof) + 1, 1);
        }

        double[] totalTimeDomain = new double[t.size()];

        for (int j = 0; j < xprof.size(); j++) {
            totalTimeDomain[j] = t.get(j);
        }

        double[] smallTimeDomain = new double[t.size() - 1];

        for (int j = 0; j < xprof.size() - 1; j++) {
            smallTimeDomain[j] = t.get(j);
        }

        {
            LinePlot plot = new LinePlot(smallTimeDomain, xRec);
            plot.addData(totalTimeDomain, xProfArray, Color.BLUE);
            plot.setXLabel("Time (s)");
            plot.setYLabel("x position (m)");
            plot.setTitle("Time Domain Responses Estimated vs Reference");
        }

        {
            LinePlot plot = new LinePlot(smallTimeDomain, yRec);
            plot.addData(totalTimeDomain, yProfArray, Color.BLUE);
            plot.setXLabel("Time (s)");
            plot.setYLabel("y position (m)");
            plot.setTitle("Time Domain Responses Estimated vs Reference");
        }

        {
            LinePlot plot = new LinePlot(smallTimeDomain, thetaRec);
            plot.addData(totalTimeDomain, thetaProfArray, Color.BLUE);
            plot.setXLabel("Time (s)");
            plot.setYLabel("Theta (rad)");
            plot.setTitle("Time Domain Responses Estimated vs Reference");
        }

        {
            LinePlot plot = new LinePlot(smallTimeDomain, vlRec);
            plot.addData(smallTimeDomain, vlRefRec, Color.BLUE);
            plot.setXLabel("Time (s)");
            plot.setYLabel("Left Velocity (m/s)");
            plot.setTitle("Time Domain Responses Estimated vs Reference");
        }

        {
            LinePlot plot = new LinePlot(smallTimeDomain, vrRec);
            plot.addData(smallTimeDomain, vrRefRec, Color.BLUE);
            plot.setXLabel("Time (s)");
            plot.setYLabel("Right Velocity (m/s)");
            plot.setTitle("Time Domain Responses Estimated vs Reference");
        }

        {
            LinePlot plot = new LinePlot(smallTimeDomain, ulRec);
            plot.setXLabel("Time (s)");
            plot.setYLabel("Left Voltage");
            plot.setTitle("Control Effort");
        }

        {
            LinePlot plot = new LinePlot(smallTimeDomain, urRec);
            plot.setXLabel("Time (s)");
            plot.setYLabel("Right Voltage");
            plot.setTitle("Control Effort");
        }
    }

}
