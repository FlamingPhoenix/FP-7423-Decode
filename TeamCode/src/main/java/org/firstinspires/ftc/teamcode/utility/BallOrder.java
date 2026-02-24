package org.firstinspires.ftc.teamcode.utility;

import java.util.ArrayDeque;
import java.util.Queue;

public class BallOrder {
    public BallColor front = BallColor.UNKNOWN;
    public BallColor middle = BallColor.UNKNOWN;
    public BallColor back = BallColor.UNKNOWN;

    public BallOrder(BallColor front, BallColor middle, BallColor back) {
        this.front = front;
        this.middle = middle;
        this.back = back;
    }
    public boolean isFull(){
        return (front != BallColor.UNKNOWN && middle != BallColor.UNKNOWN && back != BallColor.UNKNOWN);
    }


    //new better findoptimal order psuedocode:
    // take the current position and next target color
    // find the closest position with that color, add to queue, update current position
    // if no match, add the closest unknown -> closest, update current position
    // repeat for next target color until all target colors have been processed or no more known colors
    // if an unknown is present, best match it with a purple (purple more likely than green)
    // strict version only shoots exact matches.

    /**
     * Finds the most efficient order to shoot balls
     * Will ALWAYS shoot ALL balls in BEST ORDER POSSIBLE
     * Also sets static LED colors to show the shooting plan
     * Call LEDHandler.setColorsFromStatic() to apply LED changes
     * @param targetOrder the target order
     * @param currentPos the current position of the linkage
     * @return queue of positions to shoot
     */
    public Queue<POS> findOptimalOrder(BallOrder targetOrder, POS currentPos){
        Queue<POS> optimalOrder = new ArrayDeque<>();
        if(targetOrder == null || currentPos == null) {
            return optimalOrder;
        }

        // Track which positions have been used
        boolean[] used = new boolean[3]; // 0=FRONT, 1=MIDDLE, 2=BACK

        for(int i = 0; i<3; i++){ // for each target color in order
            BallColor target;
            switch (i){
                case 0:
                    target = targetOrder.front;
                    break;
                case 1:
                    target = targetOrder.middle;
                    break;
                case 2:
                    target = targetOrder.back;
                    break;
                default:
                    target = BallColor.UNKNOWN;
            }

            // Skip if target is UNKNOWN
            if(target == BallColor.UNKNOWN) {
                continue;
            }

            POS closestPos = null;
            int closestDistance = Integer.MAX_VALUE;

            // First pass: find closest position with matching color that hasn't been used
            for(int j = 0; j<3; j++){ // check positions in order of distance
                int posIndex = (currentPos.index + j) % 3;
                if(used[posIndex]) {
                    continue; // skip already-used positions
                }

                BallColor ballAtPos = getBallAtIndex(posIndex);
                if(ballAtPos == target) {
                    closestPos = getPOSFromIndex(posIndex);
                    closestDistance = j;
                    break; // found exact match at this distance, no need to continue
                }
            }

            // Second pass: if no match found, find closest UNKNOWN (prefer PURPLE over GREEN)
            if(closestPos == null) {
                for(int j = 0; j<3; j++){
                    int posIndex = (currentPos.index + j) % 3;
                    if(used[posIndex]) {
                        continue;
                    }

                    BallColor ballAtPos = getBallAtIndex(posIndex);
                    if(ballAtPos == BallColor.UNKNOWN) {
                        // If we haven't found any unknown yet, or this is closer, take it
                        if(closestPos == null) {
                            closestPos = getPOSFromIndex(posIndex);
                            closestDistance = j;
                        }
                        // If we already have an unknown, prefer PURPLE guess over GREEN
                        // (PURPLE is more likely to be in the payload)
                        if(target == BallColor.PURPLE && closestDistance == j) {
                            closestPos = getPOSFromIndex(posIndex);
                        }
                    }
                }
            }

            // Add the closest position to queue and mark as used
            if(closestPos != null) {
                optimalOrder.add(closestPos);
                used[getPOSIndex(closestPos)] = true;
                currentPos = closestPos; // update current position for next iteration
            }
        }

        // Set LED feedback for the optimal order
        setLEDsForQueue(targetOrder, optimalOrder);

        return optimalOrder;
    }
    /**
     * Only shoots balls that EXACTLY match the target order
     * Will NOT shoot balls that don't match or are UNKNOWN
     * Stops if a target color cannot be found or is UNKNOWN
     * Also sets static LED colors to show matching status
     * Call LEDHandler.setColorsFromStatic() to apply LED changes
     * @param targetOrder the exact order to match
     * @param currentPos the current position of the linkage
     * @return queue containing only balls that match the target order exactly
     */
    public Queue<POS> findOrderStrict(BallOrder targetOrder, POS currentPos) {
        Queue<POS> strictOrder = new ArrayDeque<>();
        if(targetOrder == null || currentPos == null) {
            return strictOrder;
        }

        // Track which positions have been used
        boolean[] used = new boolean[3]; // 0=FRONT, 1=MIDDLE, 2=BACK

        for(int i = 0; i<3; i++){ // for each target color in order
            BallColor target;
            switch (i){
                case 0:
                    target = targetOrder.front;
                    break;
                case 1:
                    target = targetOrder.middle;
                    break;
                case 2:
                    target = targetOrder.back;
                    break;
                default:
                    target = BallColor.UNKNOWN;
            }

            // Stop if target is UNKNOWN
            if(target == BallColor.UNKNOWN) {
                break; // can't match unknown, stop here
            }

            POS closestPos = null;
            int closestDistance = Integer.MAX_VALUE;

            // Find closest position with EXACT matching color that hasn't been used
            for(int j = 0; j<3; j++){ // check positions in order of distance
                int posIndex = (currentPos.index + j) % 3;
                if(used[posIndex]) {
                    continue; // skip already-used positions
                }

                BallColor ballAtPos = getBallAtIndex(posIndex);
                if(ballAtPos == target) {
                    closestPos = getPOSFromIndex(posIndex);
                    closestDistance = j;
                    break; // found exact match at this distance, no need to continue
                }
            }

            // If no exact match found, STOP - don't shoot remaining balls
            if(closestPos == null) {
                break;
            }

            // Add the closest position to queue and mark as used
            strictOrder.add(closestPos);
            used[getPOSIndex(closestPos)] = true;
            currentPos = closestPos; // update current position for next iteration
        }

        // Set LED feedback for strict order
        setLEDsForStrictQueue(strictOrder);

        return strictOrder;
    }

    // Helper method to get ball color at a given index
    private BallColor getBallAtIndex(int index) {
        switch(index) {
            case 0: return front;
            case 1: return middle;
            case 2: return back;
            default: return BallColor.UNKNOWN;
        }
    }

    // Helper method to get POS from index
    private POS getPOSFromIndex(int index) {
        switch(index) {
            case 0: return POS.FRONT;
            case 1: return POS.MIDDLE;
            case 2: return POS.BACK;
            default: return POS.BACK;
        }
    }

    // Helper method to get index from POS
    private int getPOSIndex(POS pos) {
        switch(pos) {
            case FRONT: return 0;
            case MIDDLE: return 1;
            case BACK: return 2;
            default: return 0;
        }
    }

    // ...existing code...

    /**
     * Sets static LED colors for findOptimalOrder results
     * LED1 (FRONT) -> LED3 (BACK) displays the shooting plan
     * Colors: LED_PURPLE/LED_GREEN for match, LED_WHITE for unknown, LED_ERROR for mismatch
     */
    private void setLEDsForQueue(BallOrder targetOrder, Queue<POS> shootQueue) {
        BallColor[] targets = new BallColor[]{targetOrder.front, targetOrder.middle, targetOrder.back};
        BallColor[] currentColors = new BallColor[]{front, middle, back};

        Queue<POS> tempQueue = new ArrayDeque<>(shootQueue); // copy to iterate
        double[] ledColors = new double[3];

        for(int i = 0; i < 3; i++) {
            double ledColor = LEDHandler.LED_OFF;

            if(!tempQueue.isEmpty()) {
                POS posToShoot = tempQueue.poll();
                if(posToShoot != null) {
                    int posIndex = getPOSIndex(posToShoot);
                    BallColor ballColor = currentColors[posIndex];
                    BallColor targetColor = targets[i];

                    if(targetColor == BallColor.UNKNOWN) {
                        ledColor = LEDHandler.LED_OFF; // don't care about this slot
                    } else if(ballColor == BallColor.UNKNOWN) {
                        ledColor = LEDHandler.LED_WHITE; // shooting an unknown
                    } else if(ballColor == targetColor) {
                        // Match - show the correct color
                        ledColor = (ballColor == BallColor.PURPLE) ? LEDHandler.LED_PURPLE : LEDHandler.LED_GREEN;
                    } else {
                        // Mismatch - different color than target
                        ledColor = LEDHandler.LED_ERROR;
                    }
                }
            } else {
                // No more balls to shoot, this slot won't be filled
                ledColor = LEDHandler.LED_OFF;
            }

            ledColors[i] = ledColor;
        }

        // Set static LED colors: LED1(0), LED2(1), LED3(2)
        LEDHandler.led1Color = ledColors[0];
        LEDHandler.led2Color = ledColors[1];
        LEDHandler.led3Color = ledColors[2];
    }

    /**
     * Sets static LED colors for findOrderStrict results
     * Matching balls show their color (LED_PURPLE/LED_GREEN)
     * Unshot balls show LED_ERROR
     */
    private void setLEDsForStrictQueue(Queue<POS> shootQueue) {
        // Track which positions will be shot
        boolean[] willBeShot = new boolean[3];
        for(POS pos : shootQueue) {
            willBeShot[getPOSIndex(pos)] = true;
        }

        // For each position (FRONT=LED1, MIDDLE=LED2, BACK=LED3)
        // If it will be shot, show color; otherwise show ERROR
        BallColor[] currentColors = new BallColor[]{front, middle, back};
        double[] ledColors = new double[3];

        for(int i = 0; i < 3; i++) {
            double ledColor;

            if(willBeShot[i]) {
                // This ball will be shot - show its color
                BallColor ballColor = currentColors[i];
                if(ballColor == BallColor.PURPLE) {
                    ledColor = LEDHandler.LED_PURPLE;
                } else if(ballColor == BallColor.GREEN) {
                    ledColor = LEDHandler.LED_GREEN;
                } else {
                    ledColor = LEDHandler.LED_ERROR; // unknown ball being shot
                }
            } else {
                // This ball will NOT be shot
                ledColor = LEDHandler.LED_ERROR;
            }

            ledColors[i] = ledColor;
        }

        // Set static LED colors: LED1(0), LED2(1), LED3(2)
        LEDHandler.led1Color = ledColors[0];
        LEDHandler.led2Color = ledColors[1];
        LEDHandler.led3Color = ledColors[2];
    }

    // ...existing code...
}
