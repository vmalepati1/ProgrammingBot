package lib.utils;

public class Timebase {

    public boolean noTimebase;
    public double dt;

    /**
     * No timebase specified
     */
    public Timebase() {
        this.noTimebase = true;
        this.dt = 0;
    }

    public Timebase(double dt) {
        this.noTimebase = false;
        this.dt = dt;
    }

}
