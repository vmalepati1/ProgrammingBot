package lib.utils;

import java.util.List;

public class FrequencyResponseData {

    // The magnitude (absolute value, not dB or log10) of the system
    // frequency response.
    public double[][][] mag;

    // The wrapped phase in radians of the system frequency response.
    public double[][][] phase;

    // The list of sorted frequencies at which the response was evaluated.
    public List<Double> omega;

}
