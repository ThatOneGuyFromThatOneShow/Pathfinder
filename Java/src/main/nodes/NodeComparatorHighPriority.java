package main.nodes;

import java.util.Comparator;

/**
 * Priority comparator to use PriorityNodes in a priority queue. Higher priority values will be moved to the top of the queue, as to be pulled sooner.
 */
public class NodeComparatorHighPriority implements Comparator<PriorityNode> {
    @Override
    public int compare(PriorityNode node1, PriorityNode node2) {
        return (int) (node2.getPriority() - node1.getPriority());
    }
}