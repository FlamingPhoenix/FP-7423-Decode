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

    /**
     * finds the best order to shoot the balls in the given order
     * vibecoded using copilot
     *
     * @param targetOrder the motif or whatever's left of it
     * @param currentPos the current position of the linkage to minimize movement time.
     * @return
     */
    public Queue<POS> findOptimalOrder(BallOrder targetOrder, POS currentPos){
        // made by chatgpt, not too shabby
        Queue<POS> optimalOrder = new ArrayDeque<>();
        if (targetOrder == null || currentPos == null) {
            return optimalOrder;
        }

        POS[] positions = new POS[]{POS.FRONT, POS.MIDDLE, POS.BACK};
        BallColor[] currentColors = new BallColor[]{
            front == null ? BallColor.UNKNOWN : front,
            middle == null ? BallColor.UNKNOWN : middle,
            back == null ? BallColor.UNKNOWN : back
        };
        BallColor[] targets = new BallColor[]{
            targetOrder.front == null ? BallColor.UNKNOWN : targetOrder.front,
            targetOrder.middle == null ? BallColor.UNKNOWN : targetOrder.middle,
            targetOrder.back == null ? BallColor.UNKNOWN : targetOrder.back
        };

        int[][] perms = new int[][]{
            {0, 1, 2}, {0, 2, 1},
            {1, 0, 2}, {1, 2, 0},
            {2, 0, 1}, {2, 1, 0}
        };

        int bestMismatch = Integer.MAX_VALUE;
        int bestExact = -1;
        int bestUnknown = Integer.MAX_VALUE;
        int bestCost = Integer.MAX_VALUE;
        int[] bestPerm = null;

        for (int[] perm : perms) {
            int mismatch = 0;
            int exact = 0;
            int unknown = 0;

            for (int i = 0; i < 3; i++) {
                BallColor target = targets[i];
                BallColor current = currentColors[perm[i]];
                if (target == BallColor.UNKNOWN) {
                    continue;
                }
                if (current == target) {
                    exact++;
                } else if (current == BallColor.UNKNOWN) {
                    unknown++;
                } else {
                    mismatch++;
                }
            }

            int cost = Math.abs(positions[perm[0]].index - currentPos.index)
                + Math.abs(positions[perm[1]].index - positions[perm[0]].index)
                + Math.abs(positions[perm[2]].index - positions[perm[1]].index);

            boolean better = mismatch < bestMismatch
                || (mismatch == bestMismatch && exact > bestExact)
                || (mismatch == bestMismatch && exact == bestExact && unknown < bestUnknown)
                || (mismatch == bestMismatch && exact == bestExact && unknown == bestUnknown && cost < bestCost);

            if (better) {
                bestMismatch = mismatch;
                bestExact = exact;
                bestUnknown = unknown;
                bestCost = cost;
                bestPerm = perm;
            }
        }

        if (bestPerm == null) {
            return optimalOrder;
        }

        optimalOrder.add(positions[bestPerm[0]]);
        optimalOrder.add(positions[bestPerm[1]]);
        optimalOrder.add(positions[bestPerm[2]]);
        return optimalOrder;
    }
}
