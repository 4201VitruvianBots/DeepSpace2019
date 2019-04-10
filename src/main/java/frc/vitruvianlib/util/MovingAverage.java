package frc.vitruvianlib.util;

public class MovingAverage {

    private double[] samples;
    private double total = 0;
    private int size;
    private int index = 0;

    public MovingAverage(int size) {
        this.size = size;
        samples = new double[size];
    }

    public void increment(double value) {
        samples[index] = value;
        total = 0;
        for(double sample:samples)
            total += sample;
        index = ++index % size;
    }

    public double getAverage() {
        return total / size;
    }
}
