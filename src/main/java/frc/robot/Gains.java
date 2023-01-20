package frc.robot;

public class Gains {
	public final double kP;
	public final double kI;
	public final double kD;

	public Gains(double kP, double kI, double kD){
		this.kP = kP;
		this.kI = kI;
		this.kD = kD;
	}

	@Override
	public String toString() {
		return "kP: " + kP + " kI: " + kI + " kD: " + kD;
	}
}