
import java.util.Arrays;
import java.util.PriorityQueue;

public class Solution {

    private static final int[][] MOVES = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
    private static final int FREE_SPACE = 0;
    private static final int OBTSACLE = 1;
    private static final int NO_PATH_FOUND = -1;

    private int rows;
    private int columns;

    public int shortestPath(int[][] matrix, int quotaForObstaclesElimination) {
        rows = matrix.length;
        columns = matrix[0].length;
        return findShortestPathByAStarSearch(matrix, quotaForObstaclesElimination);
    }

    private int findShortestPathByAStarSearch(int[][] matrix, int quotaForObstaclesElimination) {
        PriorityQueue<Point> minHeap = new PriorityQueue<>((first, second) -> first.estimateDistanceStartGoal - second.estimateDistanceStartGoal);

        int[][] visitedByGreatestQuota = new int[rows][columns];
        for (int r = 0; r < rows; ++r) {
            Arrays.fill(visitedByGreatestQuota[r], Integer.MIN_VALUE);
        }

        Point start = new Point(0, 0);
        start.distanceFromStart = 0;
        start.estimateDistanceStartGoal = manhattanDistanceToGoal(start);
        start.quotaForObstacles = quotaForObstaclesElimination;

        minHeap.add(start);
        visitedByGreatestQuota[start.row][start.column] = start.quotaForObstacles;

        while (!minHeap.isEmpty()) {
            Point current = minHeap.poll();

            if (current.row == rows - 1 && current.column == columns - 1) {
                return current.distanceFromStart;
            }
            if (current.quotaForObstacles >= manhattanDistanceToGoal(current)) {
                return current.distanceFromStart + manhattanDistanceToGoal(current);
            }

            for (int[] move : MOVES) {
                int nextRow = current.row + move[0];
                int nextColumn = current.column + move[1];

                if (isInMatrix(nextRow, nextColumn) && visitedByGreatestQuota[nextRow][nextColumn] < current.quotaForObstacles) {
                    if (matrix[nextRow][nextColumn] == OBTSACLE && current.quotaForObstacles == 0) {
                        continue;
                    }
                    Point next = new Point(nextRow, nextColumn);
                    next.distanceFromStart = current.distanceFromStart + 1;
                    next.estimateDistanceStartGoal = next.distanceFromStart + manhattanDistanceToGoal(next);
                    next.quotaForObstacles = current.quotaForObstacles - matrix[nextRow][nextColumn];

                    visitedByGreatestQuota[next.row][next.column] = next.quotaForObstacles;
                    minHeap.add(next);
                }
            }
        }
        return NO_PATH_FOUND;
    }

    private boolean isInMatrix(int row, int column) {
        return row >= 0 && row < rows && column >= 0 && column < columns;
    }

    private int manhattanDistanceToGoal(Point point) {
        return (rows - point.row - 1) + (columns - point.column - 1);
    }
}

class Point {

    int row;
    int column;
    int distanceFromStart;
    int estimateDistanceStartGoal;
    int quotaForObstacles;

    Point(int row, int column) {
        this.row = row;
        this.column = column;
    }
}
