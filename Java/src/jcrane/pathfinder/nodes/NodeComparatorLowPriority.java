package jcrane.pathfinder.nodes;

import java.util.Comparator;

/**
 * Priority comparator to use PriorityNodes in a priority queue. Lower priority values will be moved to the top of the queue, as to be pulled sooner.
 */
public class NodeComparatorLowPriority implements Comparator<PriorityNode> {
    @Override
    public int compare(PriorityNode node1, PriorityNode node2) {
        return (int) (node1.getPriority() - node2.getPriority());
    }
}