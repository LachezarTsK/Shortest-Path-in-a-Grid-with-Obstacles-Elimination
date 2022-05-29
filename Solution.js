
/**
 * @param {number[][]} matrix
 * @param {number} quotaForObstaclesElimination
 * @return {number}
 */
var shortestPath = function (matrix, quotaForObstaclesElimination) {
    this.MOVES = [[-1, 0], [1, 0], [0, -1], [0, 1]];
    this.FREE_SPACE = 0;
    this.OBTSACLE = 1;
    this.NO_PATH_FOUND = -1;

    this.rows = matrix.length;
    this.columns = matrix[0].length;
    return findShortestPathByAStarSearch(matrix, quotaForObstaclesElimination);
};

/**
 * @param {number[][]} matrix
 * @param {number} quotaForObstaclesElimination
 * @return {number}
 */
function findShortestPathByAStarSearch(matrix, quotaForObstaclesElimination) {
    const {PriorityQueue} = require('@datastructures-js/priority-queue');
    const minHeap = new MinPriorityQueue({compare: (first, second) => first.estimateDistanceStartGoal - second.estimateDistanceStartGoal});

    const visitedByGreatestQuota = Array.from(new Array(rows), () => new Array(columns).fill(Number.MIN_SAFE_INTEGER));
    const start = new Point(0, 0);

    start.distanceFromStart = 0;
    start.estimateDistanceStartGoal = manhattanDistanceToGoal(start);
    start.quotaForObstacles = quotaForObstaclesElimination;

    minHeap.enqueue(start);
    visitedByGreatestQuota[start.row][start.column] = start.quotaForObstacles;

    while (!minHeap.isEmpty()) {
        const current = minHeap.dequeue();

        if (current.row === this.rows - 1 && current.column === this.columns - 1) {
            return current.distanceFromStart;
        }
        if (current.quotaForObstacles >= manhattanDistanceToGoal(current)) {
            return current.distanceFromStart + manhattanDistanceToGoal(current);
        }

        for (let move of this.MOVES) {
            let nextRow = current.row + move[0];
            let nextColumn = current.column + move[1];

            if (isInMatrix(nextRow, nextColumn) && visitedByGreatestQuota[nextRow][nextColumn] < current.quotaForObstacles) {
                if (matrix[nextRow][nextColumn] === this.OBTSACLE && current.quotaForObstacles === 0) {
                    continue;
                }
                const next = new Point(nextRow, nextColumn);
                next.distanceFromStart = current.distanceFromStart + 1;
                next.estimateDistanceStartGoal = next.distanceFromStart + manhattanDistanceToGoal(next);
                next.quotaForObstacles = current.quotaForObstacles - matrix[nextRow][nextColumn];

                visitedByGreatestQuota[next.row][next.column] = next.quotaForObstacles;
                minHeap.enqueue(next);
            }
        }
    }
    return this.NO_PATH_FOUND;
}

/**
 * @param {number} row
 * @param {number} column
 * @return {boolean}
 */
function isInMatrix(row, column) {
    return row >= 0 && row < this.rows && column >= 0 && column < this.columns;
}

/**
 * @param {Point} point
 * @return {number}
 */
function manhattanDistanceToGoal(point) {
    return (this.rows - point.row - 1) + (this.columns - point.column - 1);
}

/**
 * @param {number} row
 * @param {number} column
 */
function Point(row, column) {
    this.row = row;
    this.column = column;
    this.distanceFromStart = 0;
    this.estimateDistanceStartGoal = 0;
    this.quotaForObstacles = 0;
}
