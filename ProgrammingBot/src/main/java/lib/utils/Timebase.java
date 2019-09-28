package lib.utils;

/**
 * @author Vikas Malepati
 */
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

    public Timebase(Timebase other) {
        this.noTimebase = other.noTimebase;
        this.dt = other.dt;
    }

    public Timebase copy() {
        return new Timebase(this);
    }

    @Override
    public String toString() {
        if (noTimebase) {
            return "dt unspecified";
        } else {
            return "dt = " + dt;
        }
    }

    @Override
    public boolean equals(Object object) {
        if (this == object) return true;
        if (object == null) return false;
        if (!(object instanceof Timebase)) return false;
        Timebase tb = (Timebase) object;
        return tb.noTimebase == noTimebase && tb.dt == dt;
    }

    @Override
    public int hashCode() {
        long result = 17;
        result = 31 * result + (noTimebase ? 1 : 0);
        result = 31 * result + Double.doubleToLongBits(dt);
        return (int) result;
    }

}
