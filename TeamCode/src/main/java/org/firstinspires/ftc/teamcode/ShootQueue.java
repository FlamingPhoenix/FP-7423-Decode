package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.POSCONFIG.LIFTERDOWN;
import static org.firstinspires.ftc.teamcode.POSCONFIG.LIFTERBLOCKING;
import static org.firstinspires.ftc.teamcode.POSCONFIG.LIFTERUP;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayDeque;
import java.util.Queue;

public class ShootQueue {
    Servo back, middle, front, lock, linkage;
    ElapsedTime timer = new ElapsedTime();
    Queue<POS> queue = new ArrayDeque<>();

    // State machine variables
    int state = 0;  // 0=idle, 1=move linkage, 2=push ball, 3=reset
    POS currentPos = POS.BACK; // Default to back, will be set when we pull from queue
    double linkageMoveTime = 0; // Time to wait for linkage to move, set based on distance

    public boolean lockOverridden = false; // If true, lock will not engage and will stay disengaged

    public ShootQueue(HardwareMap hardwareMap) {
        back = hardwareMap.servo.get("back");
        middle = hardwareMap.servo.get("middle");
        front = hardwareMap.servo.get("front");
        lock = hardwareMap.servo.get("lock");
        linkage = hardwareMap.servo.get("linkage");
    }

    /**
        * Call this to add a new ball to the shoot queue. Will be processed in order added
        * Liters will BLOCK until RESET is reached
        * @param position the position of the ball to shoot (front, middle, back, reset)
     */
    public void add(POS position) {
        queue.add(position);
    }

    /** * Call this to add a new ball to the shoot queue and then immediately reset. Useful for manual control in teleop
     * Lifters will NOT BLOCK
     * @param position the position of the ball to shoot (front, middle, back)
     */
    public void addOne(POS position) {
        // add this and then a reset. to be used in manual control in teleop
        // safe; won't get us in any stuck state
        queue.add(position);
        queue.add(POS.RESET);

    }

    /**
     * Call this every loop to process the queue
     */
    public void update() {
        switch(state) {
            case 0: // check for new balls and begin linkage movement if found

                // poll queue for next position, if empty stay idle
                if(queue.isEmpty()) {
                    // ALL IDLE HANDLES BELONG HERE

                    return; // No balls to shoot, stay idle
                }
                POS newPos = queue.poll(); // Get next position from queue
                if(newPos == null) {
                    return; // Just in case, should never happen due to isEmpty check
                }
                if(newPos == POS.RESET){
                    state = 3;
                    return;
                }

                // Position has been pulled from queue, set as current position and start moving linkage

                // determine how long to wait for linakge to move based on distance from current position
                int distance = Math.abs(newPos.index - currentPos.index);
                if(distance == 0){
                    linkageMoveTime = 0; // skip waiting if we are already in position
                } else if(distance == 1) {
                    linkageMoveTime = POSCONFIG.singlewait; // Time to move one position
                } else if (distance == 2) {
                    linkageMoveTime = POSCONFIG.doublewait; // Time to move two positions (over middle)
                }

                currentPos = newPos; // Update current position to new position

                linkage.setPosition(currentPos.linkagePos); // Move linkage to position for this ball

                state = 1; // Move to next state
                timer.reset(); //reset timer


                break;

            case 1: // MOVE_LINKAGE - wait for linkage to get in position and initiate ball push
                if(timer.milliseconds() < linkageMoveTime) {
                    return; // Still waiting for linkage to move
                }

                // Linkage should be in position now, push the appropriate ball servo based on currentPos
                switch(currentPos) {
                    case FRONT:
                        front.setPosition(LIFTERUP);
                        break;
                    case MIDDLE:
                        middle.setPosition(LIFTERUP);
                        break;
                    case BACK:
                        back.setPosition(LIFTERUP);
                        break;
                }

                state = 2; // Move to next state
                timer.reset(); // Reset timer to wait for ball to shoot
                break;

            case 2: // PUSH_BALL - wait for ball to shoot
                if(timer.milliseconds() < POSCONFIG.ballwait) {
                    return; // Still waiting for ball to shoot
                }
                // reset the current servo to the BLOCKING position to also lock
                double pos;
                if(queue.peek() == POS.RESET) {
                    //If we plan to reset later, there is no use setting this to blocking
                    //Might as well reset it to down
                    pos = LIFTERDOWN;
                } else {
                    pos = LIFTERBLOCKING;
                }
                switch (currentPos){
                    case FRONT:
                        front.setPosition(pos);
                        break;
                    case MIDDLE:
                        middle.setPosition(pos);
                        break;
                    case BACK:
                        back.setPosition(pos);
                        break;
                }
                state = 0; // Move back to idle to check for next ball
                break;

            case 3:
                front.setPosition(LIFTERDOWN);
                middle.setPosition(LIFTERDOWN);
                back.setPosition(LIFTERDOWN);
                state = 0;

                break;
        }
    }

    public boolean isShooting() {
        return state != 0;
    }

}
