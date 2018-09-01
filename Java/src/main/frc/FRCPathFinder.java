package main.frc;

import main.PathFinder;
import main.aStar.AStarResult;
import main.nodes.Node;

/**
 * FRCPathFinder uses an A* algorithm to find the shortest route from a starting location to a goal location, while avoiding obstacles.
 * This algorithm moves one Node at a time so it is not 100% the fastest route.
 *
 * This Version of PathFinder was features specifically helpful for FRC.
 */
public class FRCPathFinder extends PathFinder {

  /**
   * Calculates a path avoiding obstacles from the start node to the goal node.
   * This is done by stepping in each direction and checking out distance from the start and distance to the goal,
   * and assigning a priority of if this path will likely be fruitful.
   *
   * @param start The starting node.
   * @param goal The goal node.
   * @return an FRCResult object. This object has specific methods that are helpful for FRC.
   */
  @Override
  public FRCResult aStarSearch(Node start, Node goal) {
    AStarResult aStar = super.aStarSearch(start, goal);
    FRCResult out = new FRCResult(aStar.getNodeMap(), aStar.getNodeTrace(), aStar.getCostOfNode(), aStar.getStartNode(), aStar.getGoalNode(), aStar.getLastNode());

    return out;
  }
}
