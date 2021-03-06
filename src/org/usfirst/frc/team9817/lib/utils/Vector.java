package org.usfirst.frc.team9817.lib.utils;

import org.usfirst.frc.team9817.lib.utils.Vector;

public class Vector {
	public double x = 0;
	public double y = 0;

	public Vector(double x, double y) {
		this.x = x;
		this.y = y;
	}

	public double getAngle() {
		return Math.atan2(y, x);
	}

	public double getMagnitude() {
		return Math.hypot(x, y);
	}

	public void scale(double scalar) {
		x *= scalar;
		y *= scalar;
	}

	public void add(Vector v) {
		x += v.x;
		y += v.y;
	}

	public void subtract(Vector v) {
		x -= v.x;
		y -= v.y;
	}

	public void makePerpendicular() {
		double temp = x;
		x = y;
		y = -temp;
	}

}
