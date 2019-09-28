package lib.motionControl;

import org.ejml.simple.SimpleMatrix;

/**
 * @author Vikas Malepati
 */
public class Pose2d {

    public double x;
    public double y;
    public double theta;

    public Pose2d() {
        this(0.0, 0.0, 0.0);
    }

    public Pose2d(double x, double y, double theta) {
        this.x = x;
        this.y = y;
        this.theta = theta;
    }

    public Pose2d relativeTo(Pose2d pose) {
        SimpleMatrix R = new SimpleMatrix(
                new double[][]{
                        {Math.cos(pose.theta), -Math.sin(pose.theta), 0},
                        {Math.sin(pose.theta), Math.cos(pose.theta), 0},
                        {0, 0, 1}
                }
        ).transpose();

        SimpleMatrix temp = new SimpleMatrix(
                new double[][]{
                        {x - pose.x},
                        {y - pose.y},
                        {getContinuousError(theta - pose.theta)}
                }
        );

        temp = R.mult(temp);

        return new Pose2d(temp.get(0, 0), temp.get(1, 0), temp.get(2, 0));
    }

    public double getContinuousError(double error) {
        error = error % (Math.PI * 2);

        if (Math.abs(error) > Math.PI) {
            if (error > 0) {
                return error - 2 * Math.PI;
            } else {
                return error + 2 * Math.PI;
            }
        }

        return error;
    }

}
