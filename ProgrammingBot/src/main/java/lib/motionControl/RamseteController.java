package lib.motionControl;

import static lib.math.Special.sinc;

/**
 * @author Vikas Malepati
 */
public class RamseteController {

    public double b;
    public double zeta;

    public RamseteController() {
        this(2, 0.7);
    }

    public RamseteController(double b, double zeta) {
        this.b = b;
        this.zeta = zeta;
    }

    public RamseteTuple ramsete(Pose2d poseDesired, double vDesired, double omegaDesired, Pose2d pose) {
        Pose2d e = poseDesired.relativeTo(pose);

        double k = 2 * zeta * Math.sqrt(Math.pow(omegaDesired, 2) + b * Math.pow(vDesired, 2));
        double v = vDesired * Math.cos(e.theta) + k * e.x;
        double omega = omegaDesired + k * e.theta + b * vDesired * sinc(e.theta) * e.y;

        return new RamseteTuple(v, omega);
    }

}
