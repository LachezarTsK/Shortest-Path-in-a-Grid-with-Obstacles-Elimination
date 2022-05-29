
#include <array>
#include <queue>
#include <vector>
using namespace std;

struct Point {
    
    int row{};
    int column{};
    int distanceFromStart{};
    int estimateDistanceStartGoal{};
    int quotaForObstacles{};

    Point(int row, int column) : row {row}, column {column}{}

    Point() = default;
    virtual ~Point() = default; //virtual: no derived structs but for the sake of good practice.

    Point(const Point& p) = default;
    Point& operator=(const Point& p) = default;

    Point(Point&& p) noexcept = default;
    Point& operator=(Point&& p) noexcept = default;
};

class Solution {
    
    inline static const array<array<int, 2>, 4> MOVES{ {{-1, 0}, {1, 0}, {0, -1}, {0, 1}} };
    inline static const int FREE_SPACE = 0;
    inline static const int OBTSACLE = 1;
    inline static const int NO_PATH_FOUND = -1;

    size_t rows;
    size_t columns;

    struct Comparator {
        bool operator()(const Point& first, const Point& second) {
            return first.estimateDistanceStartGoal > second.estimateDistanceStartGoal;
        }
    };

public:
    int shortestPath(vector<vector<int>>& matrix, int quotaForObstaclesElimination) {
        rows = matrix.size();
        columns = matrix[0].size();
        return findShortestPathByAStarSearch(matrix, quotaForObstaclesElimination);
    }

private:
    int findShortestPathByAStarSearch(const vector<vector<int>>& matrix, int quotaForObstaclesElimination) {
        priority_queue<Point, vector<Point>, Comparator> minHeap;
        vector<vector<int>> visitedByGreatestQuota(rows, vector(columns, INT_MIN));

        Point start(0, 0);
        start.distanceFromStart = 0;
        start.estimateDistanceStartGoal = manhattanDistanceToGoal(start);
        start.quotaForObstacles = quotaForObstaclesElimination;

        minHeap.push(start);
        visitedByGreatestQuota[start.row][start.column] = start.quotaForObstacles;

        while (!minHeap.empty()) {
            Point current = minHeap.top();
            minHeap.pop();

            if (current.row == rows - 1 && current.column == columns - 1) {
                return current.distanceFromStart;
            }
            if (current.quotaForObstacles >= manhattanDistanceToGoal(current)) {
                return current.distanceFromStart + manhattanDistanceToGoal(current);
            }

            for (const auto& move : MOVES) {
                int nextRow = current.row + move[0];
                int nextColumn = current.column + move[1];

                if (isInMatrix(nextRow, nextColumn) && visitedByGreatestQuota[nextRow][nextColumn] < current.quotaForObstacles) {
                    if (matrix[nextRow][nextColumn] == OBTSACLE && current.quotaForObstacles == 0) {
                        continue;
                    }
                    Point next(nextRow, nextColumn);
                    next.distanceFromStart = current.distanceFromStart + 1;
                    next.estimateDistanceStartGoal = next.distanceFromStart + manhattanDistanceToGoal(next);
                    next.quotaForObstacles = current.quotaForObstacles - matrix[nextRow][nextColumn];

                    visitedByGreatestQuota[next.row][next.column] = next.quotaForObstacles;
                    minHeap.push(next);
                }
            }
        }
        return NO_PATH_FOUND;
    }

    bool isInMatrix(int row, int column) {
        return row >= 0 && row < rows && column >= 0 && column < columns;
    }

    int manhattanDistanceToGoal(const Point& point) {
        return (rows - point.row - 1) + (columns - point.column - 1);
    }
};
