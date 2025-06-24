#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>
#include <utility>

constexpr int MAP_SIZE = 128;

struct Node {
    int x, y;
    double g, h;
    Node* parent;
    Node(int x, int y, double g, double h, Node* parent = nullptr)
        : x(x), y(y), g(g), h(h), parent(parent) {}
    double f() const { return g + h; }
};

struct CompareNode {
    bool operator()(const Node* a, const Node* b) const {
        return a->f() > b->f();
    }
};

using Map = std::vector<std::vector<int>>;

std::vector<std::pair<int, int>> reconstruct_path(Node* node) {
    std::vector<std::pair<int, int>> path;
    while (node) {
        path.emplace_back(node->x, node->y);
        node = node->parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

bool is_valid(int x, int y, const Map& map) {
    return x >= 0 && x < MAP_SIZE && y >= 0 && y < MAP_SIZE && map[y][x] == 0;
}

double heuristic(int x1, int y1, int x2, int y2) {
    // Euclidean distance
    return std::hypot(x1 - x2, y1 - y2);
}

std::vector<std::pair<int, int>> a_star(const Map& map, int sx, int sy, int gx, int gy) {
    std::priority_queue<Node*, std::vector<Node*>, CompareNode> open;
    std::vector<std::vector<bool>> closed(MAP_SIZE, std::vector<bool>(MAP_SIZE, false));
    open.push(new Node(sx, sy, 0.0, heuristic(sx, sy, gx, gy)));

    std::vector<std::pair<int, int>> directions = {
        {1,0}, {-1,0}, {0,1}, {0,-1},
        {1,1}, {1,-1}, {-1,1}, {-1,-1}
    };

    while (!open.empty()) {
        Node* current = open.top();
        open.pop();

        if (current->x == gx && current->y == gy) {
            auto path = reconstruct_path(current);
            // Free memory
            delete current;
            while (!open.empty()) { delete open.top(); open.pop(); }
            return path;
        }

        if (closed[current->y][current->x]) {
            delete current;
            continue;
        }
        closed[current->y][current->x] = true;

        for (const auto& d : directions) {
            int nx = current->x + d.first;
            int ny = current->y + d.second;
            if (is_valid(nx, ny, map) && !closed[ny][nx]) {
                double cost = current->g + ((d.first == 0 || d.second == 0) ? 1.0 : std::sqrt(2));
                open.push(new Node(nx, ny, cost, heuristic(nx, ny, gx, gy), current));
            }
        }
    }
    // No path found
    return {};
}