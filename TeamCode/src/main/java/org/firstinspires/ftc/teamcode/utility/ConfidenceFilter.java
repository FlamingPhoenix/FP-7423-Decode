package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.robotcore.util.ElapsedTime;

public class ConfidenceFilter {
    private final ElapsedTime elapsedTime = new ElapsedTime();

    private long windowMs;
    private double threshold;
    private double falsePenaltyMultiplier;

    private double evidenceMs = 0;
    private Long lastUpdateMs = null;
    /**
     * Creates a confidence filter that requires the input to be consistently true for a certain percentage of a time window before isConfident() returns true. False readings will reduce confidence faster based on the falsePenaltyMultiplier.
     * @param windowMs the time window in milliseconds for evaluating confidence
     * @param threshold the percentage of the time window that must be true to be confident (0.0 to 1.0)
     */
    public ConfidenceFilter(long windowMs, double threshold) {
        this(windowMs, threshold, 1.0);
    }

    public ConfidenceFilter(long windowMs, double threshold, double falsePenaltyMultiplier) {
        this.windowMs = Math.max(1, windowMs);
        this.threshold = clamp(threshold, 0.0, 1.0);
        this.falsePenaltyMultiplier = Math.max(0.0, falsePenaltyMultiplier);
        elapsedTime.reset();
    }

    public boolean update(boolean isTrueNow) {
        return update(isTrueNow, (long) elapsedTime.milliseconds());
    }

    public boolean update(boolean isTrueNow, long nowMs) {
        if (lastUpdateMs == null) {
            lastUpdateMs = nowMs;
            evidenceMs = 0;
            return false;
        }

        long deltaMs = Math.max(0, nowMs - lastUpdateMs);
        lastUpdateMs = nowMs;

        double evidenceDelta = isTrueNow ? deltaMs : -(deltaMs * falsePenaltyMultiplier);
        evidenceMs = clamp(evidenceMs + evidenceDelta, 0.0, windowMs);

        return isConfident();
    }

    public boolean isConfident() {
        return evidenceMs >= (windowMs * threshold);
    }

    public double getConfidence() {
        return clamp(evidenceMs / windowMs, 0.0, 1.0);
    }

    public void reset() {
        evidenceMs = 0;
        lastUpdateMs = null;
    }

    public void setWindowMs(long windowMs) {
        this.windowMs = Math.max(1, windowMs);
        evidenceMs = clamp(evidenceMs, 0.0, this.windowMs);
    }

    public void setThreshold(double threshold) {
        this.threshold = clamp(threshold, 0.0, 1.0);
    }

    public void setFalsePenaltyMultiplier(double falsePenaltyMultiplier) {
        this.falsePenaltyMultiplier = Math.max(0.0, falsePenaltyMultiplier);
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
