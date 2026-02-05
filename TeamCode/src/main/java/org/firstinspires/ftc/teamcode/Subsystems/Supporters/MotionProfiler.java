package org.firstinspires.ftc.teamcode.Subsystems.Supporters;

public class MotionProfiler {
    private double maxAcc;   // ticks/sec^2
    private double maxVel;   // ticks/sec
    private double distance; // ticks
    private double sign;

    private double accelDt;
    private double cruiseDt;
    private double totalDt;
    private double accelDist;
    private double cruiseDist;

    public MotionProfiler(double distance, double maxVel, double maxAcc) {
        this.maxAcc = maxAcc;
        this.maxVel = maxVel;
        this.distance = Math.abs(distance);
        sign = Math.signum(distance);

        // Time to accelerate to max velocity
        accelDt = maxVel / maxAcc;

        double halfDist = distance / 2.0;
        double accelDistCheck = 0.5 * maxAcc * accelDt * accelDt;

        // Check if we can reach maxVel, else use triangular profile
        if (accelDistCheck > halfDist) {
            accelDt = Math.sqrt(halfDist / (0.5 * maxAcc));
        }

        accelDist = 0.5 * maxAcc * accelDt * accelDt;
        this.maxVel = maxAcc * accelDt; // recompute achievable max velocity
        cruiseDist = distance - 2 * accelDist;
        cruiseDt = (cruiseDist > 0) ? cruiseDist / maxVel : 0;
        totalDt = 2 * accelDt + cruiseDt;
    }

    public double getTotalTime() {
        return totalDt;
    }

    // Reference position at elapsedTime
    public double getReference(double t) {
        if (t >= totalDt) return distance;

        double ref;
        // Acceleration phase
        if (t < accelDt) {
            ref = 0.5 * maxAcc * t * t;
        }

        // Cruise phase
        else if (t < accelDt + cruiseDt) {
            double cruiseTime = t - accelDt;
            ref = accelDist + maxVel * cruiseTime;
        }

        // Deceleration phase
        else {
            double decelTime = t - (accelDt + cruiseDt);
            ref = accelDist + cruiseDist + maxVel * decelTime - 0.5 * maxAcc * decelTime * decelTime;
        }

        return sign * Math.min(ref, distance);
    }

    public double getVelocity(double t) {
        if (t < accelDt) {
            return maxAcc * t;
        } else if (t < accelDt + cruiseDt) {
            return maxVel;
        } else if (t < totalDt) {
            return maxVel - maxAcc * (t - accelDt - cruiseDt);
        } else {
            return 0;
        }
    }
}
