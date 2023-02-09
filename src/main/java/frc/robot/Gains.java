package frc.robot;

public class Gains {
	public final double kP;
	public final double kI;
	public final double kD;

	public final double kFF;


	public Gains(double _kP, double _kI, double _kD, double _kFF){
		kP = _kP;
		kI = _kI;
		kD = _kD;
		kFF = _kFF;
	}

	@Override
	public String toString() {
		return "P: " + kP + " I: " + kI + " D: " + kD;
	}
}