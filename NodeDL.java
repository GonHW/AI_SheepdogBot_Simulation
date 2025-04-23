/*
 * Summer 2023
 *  
 * 
 * Hench Wu
 *  
 * Program files: SheepRobot3.java(main program), NodeDL.java(Node class)
 */

public class NodeDL
	{ 
	double eUL, eU, eUR, eR, eDR, eD, eDL, eL, e;
	double pU, pR, pD, pL, pS;
	NodeDL UL, U, UR, R, DR, D, DL, L;
	boolean visited;
	boolean block, robot, sheep, goal;
	NodeDL prevLocation;
	int[] current = new int[2];
	double heuristic;
	double h, g, f;
	double p, rewardSet, rewardChange;
	public NodeDL (int x, int y) {
		heuristic = 0;
		h = 0;
		g = Integer.MAX_VALUE;
		f = Integer.MAX_VALUE;
		visited = false;
		block = false;
		goal = false;
		sheep = false;
		robot = false;
		prevLocation = null;
		current[0] = x;
		current[1] = y;
	    R = null;
        L = null;
        U = null;
        D = null;
		UL = null;
		UR = null;
		DR = null; 
		DL = null; 
		e = 0; eUL = 0; eU = 0; eUR = 0; eR = 0; eDR = 0; eD = 0; eDL = 0; eL = 0;
		pS=0; pU = 0; pR = 0; pD = 0; pL = 0;
		p = 0.0; // Example: Equal probability for all actions
     
        rewardSet = 0.0; // Set this to the appropriate reward for each state
		rewardChange = 0.0;
	}
	
}
