package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.HashMap;
import java.util.Map;


public class Debounce {
    private static final String DEFAULT_KEY = "__default__";

    private long debounceTimeMs;
    private final ElapsedTime elapsedTime = new ElapsedTime();
    private final Map<String, Boolean> previousPressedByKey = new HashMap<>();
    private final Map<String, Long> lastAcceptedEdgeMsByKey = new HashMap<>();

    // allows for button presses to register once in teleop loop for any button
    // if a button is held in loop normally, the code would execute every time

    public Debounce(long debounceTimeMs) {
        this.debounceTimeMs = debounceTimeMs;
        elapsedTime.reset();
    }

    public boolean update(boolean isPressed) {
        return update(DEFAULT_KEY, isPressed, (long) elapsedTime.milliseconds());
    }

    public boolean update(boolean isPressed, long nowMs) {
        return update(DEFAULT_KEY, isPressed, nowMs);
    }

    public boolean update(String key, boolean isPressed) {
        return update(key, isPressed, (long) elapsedTime.milliseconds());
    }

    public boolean update(String key, boolean isPressed, long nowMs) {
        Boolean wasPressed = previousPressedByKey.get(key);
        if (wasPressed == null) {
            wasPressed = false;
        }

        Long lastAcceptedMs = lastAcceptedEdgeMsByKey.get(key);
        if (lastAcceptedMs == null) {
            lastAcceptedMs = -debounceTimeMs;
        }

        boolean isRisingEdge = isPressed && !wasPressed;
        boolean debounced = nowMs - lastAcceptedMs >= debounceTimeMs;

        boolean accepted = isRisingEdge && debounced;
        if (accepted) {
            lastAcceptedEdgeMsByKey.put(key, nowMs);
        }

        previousPressedByKey.put(key, isPressed);

        return accepted;
    }

    public boolean removeKey(String key) {
        boolean removedPrev = previousPressedByKey.remove(key) != null;
        boolean removedLast = lastAcceptedEdgeMsByKey.remove(key) != null;
        return removedPrev || removedLast;
    }

    public void resetKey(String key) {
        previousPressedByKey.put(key, false);
        lastAcceptedEdgeMsByKey.put(key, -debounceTimeMs);
    }

    public void resetAllKeys() {
        previousPressedByKey.clear();
        lastAcceptedEdgeMsByKey.clear();
        elapsedTime.reset();
    }

    public void reset() {
        resetKey(DEFAULT_KEY);
    }

    public void setDebounceTimeMs(long debounceTimeMs) {
        this.debounceTimeMs = debounceTimeMs;
    }
}