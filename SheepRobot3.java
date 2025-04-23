/*
 * Summer 2023
 * 
 * Hench Wu 
 *  
 * Program files: SheepRobot3.java(main program), NodeDL.java(Node class)
 */

import java.util.*;
import java.util.concurrent.TimeUnit;

public class SheepRobot3 {
    static boolean visual = true;  // set true to see the robot and sheep move. Set false to see the average time.
    public static void main(String[] args) throws InterruptedException {
        int test = 0; // 1 is for random location
        int trial = 40;
        int i = 0;
        int count = 0;
        int max = 0;
        if(visual)
            trial = 1;
        for (i = 0; i < trial; i++) {
            int x = 31, y = 31;
            NodeDL[][] map = matrixDL(x, y); // Create map
            NodeDL robot, sheep, goal;
            setPen(map, (x) / 2, (y) / 2); // Pen in the center
            goal = map[(x) / 2][(y) / 2]; // set goal in the center

            if (test == 1) {
                // Keep this in case we want to use it for test!
                int xR = (int) (Math.random() * 31);
                int yR = (int) (Math.random() * 31);
                if (map[xR][yR].block || map[xR][yR].goal) {
                    while (map[xR][yR].block || map[xR][yR].goal) {
                        xR = (int) (Math.random() * 31);
                        yR = (int) (Math.random() * 31);
                    }
                }
                map[xR][yR].robot = true; // Random robot position (as long as it's not a block)
                robot = map[xR][yR];

                int xS = (int) (Math.random() * 31);
                int yS = (int) (Math.random() * 31);
                if (map[xS][yS].block) {
                    while (map[xS][yS].block || map[xS][yS].robot) {
                        xS = (int) (Math.random() * 31);
                        yS = (int) (Math.random() * 31);
                    }
                }
                map[xS][yS].sheep = true; // Random sheep position (as long as it's not a block)
                sheep = map[xS][yS];
                probNeighbor(sheep);
            } else {
                map[30][30].robot = true; // Robot starting in bottom right corner
                robot = map[30][30];

                map[0][0].sheep = true; // Sheep starting in upper left corner
                sheep = map[0][0];
                probNeighbor(sheep);
            }
            
            // printmap(map);
            // System.out.println(); // print methods to visualize for ourselves, but not
            // necessary for actual assignment.

            int c = MDPOptimizebot(map, robot, sheep, goal);
            if (max < c)
                max = c;
            count += c;
            System.out.println(" " + (i + 1));

        }
        System.out.println();
        System.out.println("When in farthermost corners, robot took " + ((double) (count) / i)
                + "steps on average to secure the sheep. Max: " + max);
        System.out.println();

    }

    public static int MDPOptimizebot(NodeDL[][] map, NodeDL robot, NodeDL sheep, NodeDL goal)
            throws InterruptedException {
        int gx = (map.length) / 2;
        int gy = (map[0].length) / 2;
        boolean finalpath = false;
        int step = 0;
        while (true) {
            NodeDL detect = scan(map, sheep, 11, 11); // scan 11x11 cell to check for robot
            NodeDL RobotDistanceSheep = scanRobotAvoid(map, robot);
            if ((RobotDistanceSheep != null || finalpath)) { // chase robot if found
                // System.out.println("Detect robot!, sheep detect"); //just print for us to see
                // (WILL DELETE LATER)
                if (nextToSheep(robot) && robot != goal.U && finalpath == false) { // Part 2: Start running to goal
                                                                                   // after sheep is next to robot.
                    // due to sheep detect us, bot stop moving to make sheep move to bot until sheep
                    // is next to bot
                    // System.out.println("finallllll");
                    // System.out.println(robot+", " + sheep);
                    robot = robotMDPUpdateOptimize(map, robot, goal, true); // using slower movement to lead the sheep
                } else if (finalpath && (sheep == goal.U || robot == goal.U)) { // Part3: reach one space above goal
                    // change to move agile movement to dodge sheep and move to lower left of the
                    // outer pen to make sheep into goal
                    robot = robotUpdateToSheepOrEnd(map, robot, map[goal.current[0]][goal.current[1] + 2]);
                } else if (finalpath == true && sheep != goal.U) { // there is 50-50 chance of sheep escape due to
                                                                   // diagonal to sheep
                    if (sheep != goal) { // So if sheep escape, true off part 3 and go back to part 2
                        finalpath = false;
                        robot = leadingSheepPath(map, robot, sheep);
                    }
                } else if (robot == goal) { // just in case robot in goal and move it out
                    robot = robot.U;
                } else if ((robot == goal.U && (sheep == map[gx - 2][gy] || sheep == map[gx - 2][gy - 1]
                        || sheep == map[gx + 2][gy] || sheep == map[gx + 2][gy - 1])) ||
                        (robot == goal.U.U && (sheep == map[gx - 2][gy - 1] || sheep == map[gx + 2][gy - 1]))) { // prevent
                                                                                                                 // sheep
                                                                                                                 // stuck
                                                                                                                 // in
                                                                                                                 // infinite
                                                                                                                 // loop
                    robot = robotMDPUpdateOptimize(map, robot, sheep, false);
                }
                // else if(!nextToSheep(robot)&&RobotDistanceSheep==null&&finalpath==false){
                // robot=robotUpdateToSheepSlow(map, robot, sheep); //get closer to sheep when
                // it is charging at us but stop when it is 1 space away
                // }

                // System.out.println("Hellllllloo");
                // System.out.println(robot+", " + sheep);
                sheep = activeSheepUpdate(sheep, detect); // charging toward robot
                setChangeRewards(map, robot, sheep);
                if ((goal.U.robot && goal.U.U.sheep) || goal.U.sheep) { // if robot and sheep is in the position to
                                                                        // trick sheep in to the pen
                    finalpath = true;
                }
            } else { // sheep randomly move around
                if (detect != null) // sheep randomly move around
                    sheep = activeSheepUpdate(sheep, detect);
                else {
                    sheep = passiveSheepUpdate(sheep);
                    sheepTargetDist(sheep, goal); // change probability for favoring certain direction based on the goal
                                                  // and sheep
                }
                setChangeRewards(map, robot, sheep);
                robot = robotMDPUpdateOptimize(map, robot, sheep, false);
            }
            
            ++step;
            if(visual){
                TimeUnit.SECONDS.sleep(1);
                System.out.println("\n" + step);
            }
            if (sheep == goal) { // if sheep is in the pen
                blockNode(goal.U);
                System.out.print("Sheep locked in the pen!");
                break;
            }
            if (sheep == robot) { // if sheep catch bot
                System.out.print("Sheep destroy bot!");
                break;
            }
            if(visual)
                printmap(map);

        }
        if(visual){
            System.out.println();
            printmap(map);
        }
        return step;
    }

    public static NodeDL robotMDPUpdateOptimize(NodeDL[][] map, NodeDL robot, NodeDL sheep, boolean goalpath) {
        robot.robot = false;
        double gamma = 0.80; // Discount factor
        double epsilon = (1.0 - gamma) / gamma;
        ; // Convergence threshold
        int i = 0;
        ArrayList<NodeDL> actions = new ArrayList<>();
        if (!goalpath)
            actions.addAll(Arrays.asList(robot.U, robot.D, robot.L, robot.R, robot.UL, robot.UR, robot.DL, robot.DR));
        else
            actions.addAll(Arrays.asList(robot.U, robot.D, robot.L, robot.R));
        Map<NodeDL, Double> valueFunction = new HashMap<>();
        Map<NodeDL, Double> newValues = new HashMap<>();
        for (NodeDL neighbor : actions) {
            if (neighbor != null) {
                valueFunction.put(neighbor, 0.0);
                newValues.put(neighbor, 0.0);
            }
        }
        boolean converged = false;
        double maxChange = 0.0;
        while (!converged) {
            for (NodeDL neighborR : actions) {
                if (neighborR != null) {
                    double R = 0; // no reward by default
                    if (goalpath) // positive reward for going to goal
                        R = neighborR.rewardSet; // for bring sheep to goal in case hit wall
                    else if (neighborR == map[map.length / 2][map[0].length - 1])
                        R = 100; // this is like negative reward to prevent from entering the pen for robot when
                                 // searching for sheep
                    double newValue = Double.MAX_VALUE;
                    for (NodeDL neighborS : Arrays.asList(sheep.U, sheep.D, sheep.L, sheep.R)) {
                        if (neighborS != null) {
                            double T = calculateDistance(map, neighborR, neighborS);
                            // Bellman equation for utilities
                            newValue = Math.min(newValue, R + (neighborR.rewardChange)
                                    + (1.15 - neighborS.p) * (T + gamma * valueFunction.get(neighborR)));
                            // System.out.println(newValue);
                        }
                    }

                    // Add utility value into
                    newValues.put(neighborR, newValue);

                    double oldValue = valueFunction.get(neighborR);
                    newValue = newValues.get(neighborR);
                    double change = Math.abs(newValue - oldValue);
                    maxChange = Math.max(maxChange, change);

                    if (maxChange > epsilon && i != 0 && i < 50) { // when converge
                        converged = true; // converged = false to ex
                    }

                    valueFunction.put(neighborR, newValue);
                }
            }
            i++;
        }
        NodeDL bestAction = null;
        double bestValue = Double.MAX_VALUE;
        for (NodeDL neighbor : actions) {
            if (neighbor != null) {
                double value = valueFunction.get(neighbor);
                if (value < bestValue) {
                    bestValue = value;
                    bestAction = neighbor;
                }
                // System.out.println(value); //for debugging
            }
        }
        // System.out.println("best value: " + bestValue + " i = " + i); //for debugging
        bestAction.robot = true;
        return bestAction;
    }

    public static NodeDL scan(NodeDL[][] map, NodeDL n, int row, int col) {
        int x = n.current[0], y = n.current[1];
        int halfRow = row / 2;
        int halfCol = col / 2;
        int startX = x - halfCol;
        int startY = y - halfRow;
        int endX = x + halfCol;
        int endY = y + halfRow;

        if (startX < 0)
            startX = 0;
        if (startY < 0)
            startY = 0;
        if (endX >= map.length)
            endX = map.length - 1;
        if (endY >= map[x].length)
            endY = map[x].length - 1;

        for (int i = startX; i <= endX; i++) {
            for (int j = startY; j <= endY; j++) {
                if (map[i][j].robot)
                    return map[i][j];
            }
        }
        return null;
    }

    public static NodeDL scanRobotAvoid(NodeDL[][] map, NodeDL n) {

        int x = n.current[0]; // get x position of sheep
        int y = n.current[1]; // get y position of sheep
        int col = 5; // number of columns to scan
        int row = 5; // number of rows to scan

        if (x == 0 || x == map.length - 1) // if x near the very edge
            col = 3; // columns of scan becomes 3
        else if (x == 1 || x == map.length - 2) // if x is one space before edge
            col = 4; // columns of scan becomes 4
        if (y == 0 || y == map[x].length - 1) // if y near the very edge
            row = 3; // rows of scan becomes 3
        else if (x == 1 || y == map[x].length - 2) // if y xis one space before edge
            row = 4; // rows of scan becomes 4
        int xs = x - 2; // get the upper left
        int ys = y - 2; // get the upper left
        if (xs < 0) // if xs is negative
            xs = 0;
        if (ys < 0) // if ys is negative
            ys = 0;
        for (int i = xs; i < (xs + col); i++) {
            if (map[i][y].sheep)
                return map[i][y]; // if found sheep, return node of robot
        }
        for (int j = ys; j < (ys + row); j++) {
            if (map[x][j].sheep)
                return map[x][j]; // if found sheep, return node of robot
        }

        return scanSheep(map, n, 3, 3); // if no shee found, return null
    }

    public static NodeDL scanSheep(NodeDL[][] map, NodeDL n, int row, int col) {
        int x = n.current[0], y = n.current[1];
        int halfRow = row / 2;
        int halfCol = col / 2;
        int startX = x - halfCol;
        int startY = y - halfRow;
        int endX = x + halfCol;
        int endY = y + halfRow;

        if (startX < 0)
            startX = 0;
        if (startY < 0)
            startY = 0;
        if (endX >= map.length)
            endX = map.length - 1;
        if (endY >= map[x].length)
            endY = map[x].length - 1;

        for (int i = startX; i <= endX; i++) {
            for (int j = startY; j <= endY; j++) {
                if (map[i][j].sheep)
                    return map[i][j];
            }
        }
        return null;
    }

    public static NodeDL activeSheepUpdate(NodeDL sheep, NodeDL target) { // when sheep detect robot in 5x5 cell
        NodeDL direction = sheep; // default sheep not moving
        int dx = Math.abs(sheep.current[0] - target.current[0]); // compute x distance
        int dy = Math.abs(sheep.current[1] - target.current[1]); // compute y distance
        if (dx > dy) { // choose to move toward the longer axis x
            if (sheep.current[0] > target.current[0] && sheep.L != null)
                direction = sheep.L;
            else if (sheep.R != null)
                direction = sheep.R;
        } else if (dx < dy) { // choose to move toward the longer axis y
            if (sheep.current[1] > target.current[1] && sheep.U != null)
                direction = sheep.U;
            else if (sheep.D != null)
                direction = sheep.D;
        } else { // if both axis is the same distance
            NodeDL c1, c2;
            if (sheep.current[0] > target.current[0] && sheep.current[1] > target.current[1]) { // determine 2 possible
                                                                                                // direction
                c1 = sheep.L;
                c2 = sheep.U;
            } else if (sheep.current[0] < target.current[0] && sheep.current[1] < target.current[1]) {
                c1 = sheep.R;
                c2 = sheep.D;
            } else if (sheep.current[0] < target.current[0] && sheep.current[1] > target.current[1]) {
                c1 = sheep.R;
                c2 = sheep.U;
            } else {
                c1 = sheep.L;
                c2 = sheep.D;
            }
            if (Math.random() < .5) // choose based on 50-50
                direction = c1;
            else
                direction = c2;
        }
        if (direction == null) // just for in case if the choose direction is still null such as inside the pen
            direction = sheep; // then sheep not move

        sheep.sheep = false; // turn of previous sheep position
        clearProbNeighbor(sheep);
        direction.sheep = true; // update new sheep position
        activeProb(direction, target); // get active probability of sheep
        return direction;
    }

    public static NodeDL passiveSheepUpdate(NodeDL sheep) { // when sheep detect robot in 5x5 cell
        NodeDL direction = null; // default sheep not moving
        while (direction == null) {
            double r = Math.random();
            if (r < 0.25)
                direction = sheep.L; // sheep move left
            else if (r < 0.50)
                direction = sheep.R; // sheep move right
            else if (r < 0.75)
                direction = sheep.U; // sheep move up
            else
                direction = sheep.D; // Sheep move down
        }
        sheep.sheep = false; // turn of previous sheep position
        clearProbNeighbor(sheep);
        direction.sheep = true; // update new sheep position
        probNeighbor(direction);

        return direction;
    }

    public static boolean nextToSheep(NodeDL robot) { // check if bot is next to sheep
        if (robot.U != null && robot.U.sheep)
            return true;
        if (robot.R != null && robot.R.sheep)
            return true;
        if (robot.D != null && robot.D.sheep)
            return true;
        if (robot.L != null && robot.L.sheep)
            return true;
        return false;
    }

    public static NodeDL robotUpdateToSheepOrEnd(NodeDL[][] map, NodeDL robot, NodeDL Target) { // robot agile movement
        Stack<NodeDL> n = ComputePath(map, robot, Target); // able to move shortest distance and agile
        robot.robot = false; // removing previous bot position
        if (n != null && !n.empty()) {
            robot = n.pop(); // pop current position
            if (!n.empty()) // if there is more next move
                robot = n.pop(); // pop next position
        }
        robot.robot = true; // Adding current bot position
        return robot;
    }

    public static NodeDL leadingSheepPath(NodeDL[][] map, NodeDL robot, NodeDL Target) {
        Stack<NodeDL> n = ComputePath2(map, robot, Target); // slow movement for robot to lead sheep
        robot.robot = false; // removing previous bot position
        if (n == null) { // if there is no path to goal, it means sheep is block the entrance to goal
            if (robot.U != null)
                robot = robot.U; // so robot move up to lead sheep out of the way
        }
        if (n != null && !n.empty()) {
            robot = n.pop(); // pop current position
            if (!n.empty()) // if there is more next move
                robot = n.pop(); // pop next position
        }

        robot.robot = true; // Adding current bot position
        return robot;
    }

    public static NodeDL robotUpdateToSheepSlow(NodeDL[][] map, NodeDL robot, NodeDL Target) { // robot agile movement
        Stack<NodeDL> n = ComputePath2(map, robot, Target); // able to move shortest distance and agile
        robot.robot = false; // removing previous bot position
        if (n != null && !n.empty()) {
            robot = n.pop(); // pop current position
            if (!n.empty()) // if there is more next move
                robot = n.pop(); // pop next position
        }
        robot.robot = true; // Adding current bot position
        return robot;
    }

    public static NodeDL leadSheepNode(NodeDL[][] map, NodeDL sheep, NodeDL goal) {
        NodeDL detect = scan(map, sheep, 11, 11); // sheep detect robot
        if (detect == null) // if not detect robot, go to sheep
            return sheep;
        return detect; // if sheep detect bot, stay at that location to let sheep come to bot
    }

    public static double calculateDistance(NodeDL[][] map, NodeDL botNode, NodeDL target) {
        double distance = 0;
        Stack<NodeDL> n = ComputePath2(map, botNode, target); // count distan
        if (n == null)
            return Double.MAX_VALUE;
        if (n.empty())
            return 0;
        n.pop();
        while (!n.empty()) {
            n.pop();
            distance += 1.0;
        }
        return distance;
    }

    public static void activeProb(NodeDL sheep, NodeDL target) {
        int dx = Math.abs(sheep.current[0] - target.current[0]); // compute x distance
        int dy = Math.abs(sheep.current[1] - target.current[1]); // compute y distance
        if (dx > dy) { // choose to move toward the longer axis x
            if (sheep.current[0] > target.current[0]) // calculating active probability of sheep
                if (sheep.L != null) {
                    sheep.L.p = 1;
                } // this part always walk towards the target
                else {
                    sheep.p = 1;
                }
            else if (sheep.R != null) {
                sheep.R.p = 1;
            } else {
                sheep.p = 1;
            }
        } else if (dx < dy) { // choose to move toward the longer axis y
            if (sheep.current[1] > target.current[1]) // calculating active probability of sheep
                if (sheep.U != null) {
                    sheep.U.p = 1;
                } // this part always walk towards the target
                else {
                    sheep.p = 1;
                }
            else if (sheep.D != null) {
                sheep.D.p = 1;
            } else {
                sheep.p = 1;
            }
        } else { // if both axis is the same distance
            NodeDL c1, c2;
            if (sheep.current[0] > target.current[0] && sheep.current[1] > target.current[1]) { // determine 2 possible
                                                                                                // direction
                c1 = sheep.L;
                c2 = sheep.U;
            } else if (sheep.current[0] < target.current[0] && sheep.current[1] < target.current[1]) {
                c1 = sheep.R;
                c2 = sheep.D;
            } else if (sheep.current[0] < target.current[0] && sheep.current[1] > target.current[1]) {
                c1 = sheep.R;
                c2 = sheep.U;
            } else {
                c1 = sheep.L;
                c2 = sheep.D;
            }
            // calculate the probability to move when it is 50-50 in 2 direction
            if (c1 != null) {
                c1.p = 0.5;
            } else {
                sheep.p += 0.5;
            } // if cell is block, sheep has 50% stay in same cell

            if (c2 != null) {
                c2.p = 0.5;
            } else {
                sheep.p += 0.5;
            } // if cell is block, sheep has 50% to 100% stay in same cell
        }
    }

    public static void sheepTargetDist(NodeDL sheep, NodeDL target) {
        int dx = Math.abs(sheep.current[0] - target.current[0]); // compute x distance
        int dy = Math.abs(sheep.current[1] - target.current[1]); // compute y distance
        if (dx > dy) { // choose to move toward the longer axis x
            if (sheep.current[0] > target.current[0]) { // calculating active probability of sheep
                if (sheep.L != null)
                    sheep.L.p += 0.1;
                if (sheep.R != null)
                    sheep.R.p -= 0.1;
            } else {
                if (sheep.R != null)
                    sheep.R.p += 0.1;
                if (sheep.L != null)
                    sheep.L.p -= 0.1;
            }
        } else if (dx < dy) { // choose to move toward the longer axis y
            if (sheep.current[1] > target.current[1]) { // calculating active probability of sheep
                if (sheep.U != null)
                    sheep.U.p += 0.1;
                if (sheep.D != null)
                    sheep.D.p -= 0.1;
            } else {
                if (sheep.U != null)
                    sheep.U.p += 0.1;
                if (sheep.D != null)
                    sheep.D.p -= 0.1;
            }
        } else { // if both axis is the same distance
            if (sheep.current[0] > target.current[0] && sheep.current[1] > target.current[1]) { // determine 2 possible
                                                                                                // direction
                if (sheep.L != null)
                    sheep.L.p += 0.1;
                if (sheep.U != null)
                    sheep.U.p += 0.1;
                if (sheep.R != null)
                    sheep.R.p -= 0.1;
                if (sheep.D != null)
                    sheep.D.p -= 0.1;
            } else if (sheep.current[0] < target.current[0] && sheep.current[1] < target.current[1]) {
                if (sheep.L != null)
                    sheep.L.p -= 0.1;
                if (sheep.U != null)
                    sheep.U.p -= 0.1;
                if (sheep.R != null)
                    sheep.R.p += 0.12;
                if (sheep.D != null)
                    sheep.D.p += 0.1;
            } else if (sheep.current[0] < target.current[0] && sheep.current[1] > target.current[1]) {
                if (sheep.L != null)
                    sheep.L.p -= 0.1;
                if (sheep.U != null)
                    sheep.U.p += 0.1;
                if (sheep.R != null)
                    sheep.R.p += 0.12;
                if (sheep.D != null)
                    sheep.D.p -= 0.1;
            } else {
                if (sheep.L != null)
                    sheep.L.p += 0.12;
                if (sheep.U != null)
                    sheep.U.p -= 0.1;
                if (sheep.R != null)
                    sheep.R.p -= 0.1;
                if (sheep.D != null)
                    sheep.D.p += 0.1;
            }
        }
    }

    public static void probNeighbor(NodeDL sheep) {
        double direction = 0.0;
        if (sheep.U != null) {
            direction++;
        }
        if (sheep.D != null) {
            direction++;
        }
        if (sheep.L != null) {
            direction++;
        }
        if (sheep.R != null) {
            direction++;
        }
        if (sheep != null) {
            direction++;
        }
        if (sheep.U != null) {
            sheep.U.p = 1.0 / direction;
        }
        if (sheep.D != null) {
            sheep.D.p = 1.0 / direction;
        }
        if (sheep.L != null) {
            sheep.L.p = 1.0 / direction;
        }
        if (sheep.R != null) {
            sheep.R.p = 1.0 / direction;
        }
        if (sheep != null) {
            sheep.p = 1.0 / direction;
        }
    }

    public static void clearProbNeighbor(NodeDL sheep) {
        if (sheep.U != null) {
            sheep.U.p = 0.0;
        }
        if (sheep.D != null) {
            sheep.D.p = 0.0;
        }
        if (sheep.L != null) {
            sheep.L.p = 0.0;
        }
        if (sheep.R != null) {
            sheep.R.p = 0.0;
        }
        if (sheep.R != null) {
            sheep.p = 0.0;
        }
    }

    public static NodeDL[][] matrixDL(int x, int y) {
        /* Intitializing map */
        NodeDL[][] grid = new NodeDL[x][y];
        /* Setting up nodes */
        for (int i = 0; i < x; i++)
            for (int j = 0; j < y; j++)
                grid[i][j] = new NodeDL(i, j);

        /* connecting tiles together */
        for (int i = 0; i < x; i++) {
            for (int j = 0; j < y; j++) {

                if (i > 0 && j > 0) {
                    grid[i][j].UL = grid[i - 1][j - 1];
                }
                if (j > 0) {
                    grid[i][j].U = grid[i][j - 1];
                }
                if (i < grid.length - 1 && j > 0) {
                    grid[i][j].UR = grid[i + 1][j - 1];
                }
                if (i < grid.length - 1) {
                    grid[i][j].R = grid[i + 1][j];
                }
                if (i < grid.length - 1 && j < grid.length - 1) {
                    grid[i][j].DR = grid[i + 1][j + 1];
                }
                if (j < grid[i].length - 1) {
                    grid[i][j].D = grid[i][j + 1];
                }
                if (i > 0 && j < grid.length - 1) {
                    grid[i][j].DL = grid[i - 1][j + 1];
                }
                if (i > 0) {
                    grid[i][j].L = grid[i - 1][j];
                }
            }
        }
        return grid;
    }

    public static void setChangeRewards(NodeDL[][] map, NodeDL robot, NodeDL sheep) {
        int gx = (map.length) / 2;
        int gy = (map[0].length) / 2;
        clearChangeRewards(map);
        if (sheep.current[0] > (gx - 2) && sheep.current[0] < (gx + 2) && sheep.current[1] < (gy - 1)) {
            map[gx][gy + 2].rewardChange = 3;
            map[gx + 1][gy + 2].rewardChange = 2;
            map[gx - 1][gy + 2].rewardChange = 2;
            map[gx][gy + 3].rewardChange = 3;
        } else if (sheep.current[0] > (gx - 2) && sheep.current[0] < (gx + 2) && sheep.current[1] > (gy + 1)) {
            map[gx][gy - 2].rewardChange = 3;
            map[gx + 1][gy - 2].rewardChange = 2;
            map[gx - 1][gy - 2].rewardChange = 2;
            map[gx][gy - 3].rewardChange = 3;
        } else if (sheep.current[1] > (gy - 2) && sheep.current[1] < (gy + 2) && sheep.current[0] > (gx + 1)) {
            map[gx - 2][gy].rewardChange = 3;
            map[gx - 2][gy + 1].rewardChange = 2;
            map[gx - 2][gy - 1].rewardChange = 2;
            map[gx - 3][gy].rewardChange = 3;
        } else if (sheep.current[1] > (gy - 2) && sheep.current[1] < (gy + 2) && sheep.current[0] < (gx - 1)) {
            map[gx + 2][gy].rewardChange = 3;
            map[gx + 2][gy + 1].rewardChange = 2;
            map[gx + 2][gy - 1].rewardChange = 2;
            map[gx + 3][gy].rewardChange = 3;
        }
    }

    public static void clearChangeRewards(NodeDL[][] map) {
        int gx = (map.length) / 2;
        int gy = (map[0].length) / 2;

        map[gx][gy + 2].rewardChange = 0;
        map[gx + 1][gy + 2].rewardChange = 0;
        map[gx - 1][gy + 2].rewardChange = 0;
        map[gx][gy + 3].rewardChange = 0;

        map[gx][gy - 2].rewardChange = 0;
        map[gx + 1][gy - 2].rewardChange = 0;
        map[gx - 1][gy - 2].rewardChange = 0;
        map[gx][gy - 3].rewardChange = 0;

        map[gx - 2][gy].rewardChange = 0;
        map[gx - 2][gy + 1].rewardChange = 0;
        map[gx - 2][gy - 1].rewardChange = 0;
        map[gx - 3][gy].rewardChange = 0;

        map[gx + 2][gy].rewardChange = 0;
        map[gx + 2][gy + 1].rewardChange = 0;
        map[gx + 2][gy - 1].rewardChange = 0;
        map[gx + 3][gy].rewardChange = 0;
    }

    public static void setPen(NodeDL[][] map, int x, int y) {
        blockNode(map[x - 1][y - 1]); // set the block gpt pen
        blockNode(map[x - 1][y]);
        blockNode(map[x - 1][y + 1]);
        blockNode(map[x][y + 1]);
        blockNode(map[x + 1][y + 1]);
        blockNode(map[x + 1][y]);
        blockNode(map[x + 1][y - 1]);
        map[x][y].goal = true; // Set the goal
        map[x][y - 1].p = 1; // Set goal.U probability = 1

        map[x][y + 2].rewardSet = -10; // set reward around the pen
        map[x - 1][y + 2].rewardSet = -20; // to prevent trap when bring
        map[x + 1][y + 2].rewardSet = -20; // sheep to goal due to imperfect
        map[x - 2][y + 2].rewardSet = -30; // distance count
        map[x + 2][y + 2].rewardSet = -30;
        map[x - 2][y + 1].rewardSet = -40;
        map[x + 2][y + 1].rewardSet = -40;
        map[x - 2][y].rewardSet = -50;
        map[x + 2][y].rewardSet = -50;
        map[x - 2][y - 1].rewardSet = -60;
        map[x + 2][y - 1].rewardSet = -60;
        map[x - 2][y - 2].rewardSet = -70;
        map[x + 2][y - 2].rewardSet = -70;
        map[x - 1][y - 2].rewardSet = -80;
        map[x + 1][y - 2].rewardSet = -80;
        map[x][y - 2].rewardSet = -90;
        map[x][y - 2].rewardSet = -90;
        map[x][y - 1].rewardSet = -1000;
    }

    public static void blockNode(NodeDL n) {
        if (n == null)
            return;
        if (n.UL != null)
            n.UL.DR = null;
        if (n.U != null)
            n.U.D = null;
        if (n.UR != null)
            n.UR.DL = null;
        if (n.R != null)
            n.R.L = null;
        if (n.DR != null)
            n.DR.UL = null;
        if (n.D != null)
            n.D.U = null;
        if (n.DL != null)
            n.DL.UR = null;
        if (n.L != null)
            n.L.R = null;
        n.block = true;
    }

    public static void printmap(NodeDL[][] map) {
        for (int j = 0; j < map[0].length; j++) {
            String line = "";
            for (int i = 0; i < map.length; i++) {
                line += " | ";
                if (map[i][j].sheep)
                    line += "S";
                else if (map[i][j].robot)
                    line += "R";
                else if (map[i][j].goal)
                    line += "G";
                else if (map[i][j].block)
                    line += "X";
                else if (map[i][j].visited)
                    line += "*";
                else
                    line += " ";
            }
            if (!line.equals(""))
                System.out.println(line + " | ");
        }
    }

    // ---------------------

    /*
     * ---------Ignore below this line. It is A* search in case we need it.
     * -----------------------
     */

    private static Comparator<NodeDL> comparator = new Comparator<NodeDL>() {
        public int compare(NodeDL node1, NodeDL node2) {
            return Double.compare(node1.f, node2.f);
        }
    };

    public static Stack<NodeDL> ComputePath(NodeDL[][] grid, NodeDL start, NodeDL goal) {
        PriorityQueue<NodeDL> openSet = new PriorityQueue<>(comparator); // Priority queue to store nodes to be explored
        Set<NodeDL> closedSet = new HashSet<>(); // Set to store visited nodes
        Map<NodeDL, NodeDL> cameFrom = new HashMap<>(); // Map to store the path reconstruction information
        start.g = 0;
        start.h = heuristic(start, goal); // calculate heuristic
        start.f = start.g * 0.9999 + start.h;
        start.visited = true;
        openSet.add(start); // Add the start node to the open set

        // A* search algorithm
        while (!openSet.isEmpty()) {
            // Get the node with the lowest f-value from the open set
            NodeDL current = openSet.poll();

            if (current == goal) {
                clearVisited(grid);
                return reconstructPath(cameFrom, current); // If the current node is the goal, create the path and
                                                           // return it
            }

            closedSet.add(current);

            for (NodeDL neighbor : getNeighbors(current, grid)) { // Check the neighbors of the current node
                if (closedSet.contains(neighbor)) {
                    continue;
                }

                double tentativeGScore = current.g + 1;

                // If the tentative g-score is better than the previous g-score, update the
                // neighbor
                if (tentativeGScore < neighbor.g) {
                    cameFrom.put(neighbor, current);
                    neighbor.g = tentativeGScore;
                    neighbor.h = heuristic(neighbor, goal);
                    neighbor.f = neighbor.g * 0.9999 + neighbor.h;

                    if (!openSet.contains(neighbor)) { // If the neighbor is not in the open set, add it
                        openSet.add(neighbor);
                    }
                }
            }
        }
        clearVisited(grid);
        return null; // No path found
    }

    public static Stack<NodeDL> ComputePath2(NodeDL[][] grid, NodeDL start, NodeDL goal) {
        PriorityQueue<NodeDL> openSet = new PriorityQueue<>(comparator); // Priority queue to store nodes to be explored
        Set<NodeDL> closedSet = new HashSet<>(); // Set to store visited nodes
        Map<NodeDL, NodeDL> cameFrom = new HashMap<>(); // Map to store the path reconstruction information
        start.g = 0;
        start.h = heuristic2(start, goal); // calculate heuristic
        start.f = start.g * 0.9999 + start.h;
        start.visited = true;
        openSet.add(start); // Add the start node to the open set

        // A* search algorithm
        while (!openSet.isEmpty()) {
            // Get the node with the lowest f-value from the open set
            NodeDL current = openSet.poll();

            if (current == goal) {
                clearVisited(grid);
                return reconstructPath(cameFrom, current); // If the current node is the goal, create the path and
                                                           // return it
            }
            closedSet.add(current);

            for (NodeDL neighbor : getNeighborsWithoutDiag(current, grid)) { // Check the neighbors of the current node
                if (closedSet.contains(neighbor)) {
                    continue;
                }
                double tentativeGScore = current.g + 1;

                // If the tentative g-score is better than the previous g-score, update the
                // neighbor
                if (tentativeGScore < neighbor.g) {
                    cameFrom.put(neighbor, current);
                    neighbor.g = tentativeGScore;
                    neighbor.h = Math.abs(neighbor.current[0] - goal.current[0])
                            + Math.abs(neighbor.current[1] - goal.current[1]);
                    neighbor.f = neighbor.g * 0.9999 + neighbor.h;

                    if (!openSet.contains(neighbor)) { // If the neighbor is not in the open set, add it
                        openSet.add(neighbor);
                    }
                }
            }
        }
        clearVisited(grid);
        return null; // No path found
    }

    public static List<NodeDL> getNeighborsWithoutDiag(NodeDL node, NodeDL[][] grid) {
        List<NodeDL> neighbors = new ArrayList<>();

        if (node.U != null && !node.U.block && !node.U.visited) {
            node.U.visited = true;
            neighbors.add(node.U);
        }
        if (node.R != null && !node.R.block && !node.R.visited) {
            node.R.visited = true;
            neighbors.add(node.R);
        }
        if (node.D != null && !node.D.block && !node.D.visited) {
            node.D.visited = true;
            neighbors.add(node.D);
        }
        if (node.L != null && !node.L.block && !node.L.visited) {
            node.L.visited = true;
            neighbors.add(node.L);
        }

        return neighbors;
    }

    public static List<NodeDL> getNeighbors(NodeDL node, NodeDL[][] grid) {
        List<NodeDL> neighbors = new ArrayList<>();

        if (node.UL != null && !node.UL.block && !node.UL.visited) {
            node.UL.visited = true;
            neighbors.add(node.UL);
        }
        if (node.U != null && !node.U.block && !node.U.visited && !node.U.sheep) {
            node.U.visited = true;
            neighbors.add(node.U);
        }
        if (node.UR != null && !node.UR.block && !node.UR.visited) {
            node.UR.visited = true;
            neighbors.add(node.UR);
        }
        if (node.R != null && !node.R.block && !node.R.visited) {
            node.R.visited = true;
            neighbors.add(node.R);
        }
        if (node.DR != null && !node.DR.block && !node.DR.visited) {
            node.DR.visited = true;
            neighbors.add(node.DR);
        }
        if (node.D != null && !node.D.block && !node.D.visited) {
            node.D.visited = true;
            neighbors.add(node.D);
        }
        if (node.DL != null && !node.DL.block && !node.DL.visited) {
            node.DL.visited = true;
            neighbors.add(node.DL);
        }
        if (node.L != null && !node.L.block && !node.L.visited) {
            node.L.visited = true;
            neighbors.add(node.L);
        }

        return neighbors;
    }

    public static Stack<NodeDL> reconstructPath(Map<NodeDL, NodeDL> cameFrom, NodeDL current) {
        Stack<NodeDL> path = new Stack<NodeDL>();
        while (current != null) {
            path.push(current);
            current = cameFrom.get(current);
        }
        return path;
    }

    public static Stack<NodeDL> createPath(NodeDL ptr) {
        Stack<NodeDL> stack = new Stack<>();
        while (ptr != null) {
            stack.push(ptr);
            ptr.visited = false;
            ptr = ptr.prevLocation;
        }
        return stack;
    }

    public static double heuristic(NodeDL node, NodeDL goal) {
        return Math
                .sqrt(Math.pow(node.current[0] - goal.current[0], 2) + Math.pow(node.current[1] - goal.current[1], 2));
    }

    public static int heuristic2(NodeDL node, NodeDL goal) {// Manhattan distance heuristic
        return Math.abs(node.current[0] - goal.current[0]) + Math.abs(node.current[1] - goal.current[1]);
    }

    public static NodeDL[][] clearVisited(NodeDL[][] grid) {
        for (int i = 0; i < grid.length; i++) {
            for (int j = 0; j < grid[i].length; j++) {
                // if(grid[i][j]!=null)
                grid[i][j].visited = false;
                grid[i][j].f = Double.MAX_VALUE;
                grid[i][j].g = Double.MAX_VALUE;
                grid[i][j].h = 0;
            }
        }
        return grid;
    }

}