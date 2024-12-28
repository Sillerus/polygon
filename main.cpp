#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <unordered_map>
#include <stack>
#include <queue>
#include <limits>
#include <cmath>
#include <algorithm>
#include <set>
#include <chrono>

using namespace std;
using namespace chrono;

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <unordered_map>
#include <stack>
#include <queue>
#include <limits>
#include <cmath>
#include <algorithm>
#include <set>

using namespace std;

// Структура узла графа с координатами и ребрами
struct Node {
    double lon; // Долгота узла
    double lat; // Широта узла
    vector<pair<Node*, double>> edges; // Ребра к соседним узлам с весами

    Node(double lon, double lat) : lon(lon), lat(lat) {}
};

// Хэш-функция для пары координат
struct PairHash {
    template <class T1, class T2>
    size_t operator()(const pair<T1, T2>& p) const {
        auto h1 = hash<T1>{}(p.first);
        auto h2 = hash<T2>{}(p.second);
        return h1 ^ (h2 << 1);
    }
};

// Класс графа, инкапсулирующий операции и структуру графа
class Graph {
private:
    unordered_map<pair<double, double>, Node*, PairHash> nodes; // Словарь узлов с ключами и указателями

public:
    ~Graph() {
        // Деструктор для очистки динамически выделенных узлов
        for (auto& [key, node] : nodes) {
            delete node;
        }
    }

    // Добавляет узел в граф или возвращает существующий
    Node* addNode(double lon, double lat) {
        pair<double, double> key = {lon, lat};
        if (nodes.find(key) == nodes.end()) {
            nodes[key] = new Node(lon, lat);
        }
        return nodes[key];
    }

    // Добавляет ребро между двумя узлами, создавая их при необходимости
    void addEdge(double lon1, double lat1, double lon2, double lat2, double weight) {
        Node* node1 = addNode(lon1, lat1);
        Node* node2 = addNode(lon2, lat2);
        node1->edges.emplace_back(node2, weight);
    }

    // Получает узел по его координатам
    Node* getNode(double lon, double lat) {
        pair<double, double> key = {lon, lat};
        if (nodes.find(key) != nodes.end()) {
            return nodes[key];
        }
        return nullptr;
    }
};

// Читает граф из файла с заданным форматом
Graph readGraphFromFile(const string& filename) {
    Graph graph;
    ifstream file(filename);
    if (!file.is_open()) {
        cerr << "Ошибка: не удается открыть файл " << filename << endl;
        return graph;
    }

    string line;
    while (getline(file, line)) {
        stringstream ss(line);
        string nodeData, neighbors;
        getline(ss, nodeData, ':');

        double lon1, lat1;
        char comma;
        stringstream nodeStream(nodeData);
        nodeStream >> lon1 >> comma >> lat1;

        while (getline(ss, neighbors, ';')) {
            double lon2, lat2, weight;
            stringstream neighborStream(neighbors);
            neighborStream >> lon2 >> comma >> lat2 >> comma >> weight;

            graph.addEdge(lon1, lat1, lon2, lat2, weight);
        }
    }

    file.close();
    return graph;
}

// Поиск кратчайшего пути с использованием поиска в глубину (DFS)
pair<vector<Node*>, double> dfsShortestPath(Node* start, Node* goal) {
    if (!start || !goal) {
        return {{}, 0.0};
    }

    unordered_map<Node*, bool> visited;
    unordered_map<Node*, Node*> cameFrom;
    unordered_map<Node*, double> distance;
    stack<pair<Node*, double>> stack;

    stack.push({start, 0.0});
    distance[start] = 0.0;

    while (!stack.empty()) {
        auto [current, currDist] = stack.top();
        stack.pop();

        if (visited[current]) {
            continue;
        }

        visited[current] = true;

        if (current == goal) {
            break;
        }

        for (const auto& edge : current->edges) {
            Node* neighbor = edge.first;
            double edgeWeight = edge.second;

            double newDist = currDist + edgeWeight;

            if (!visited[neighbor] && (distance.find(neighbor) == distance.end() || newDist < distance[neighbor])) {
                distance[neighbor] = newDist;
                cameFrom[neighbor] = current;
                stack.push({neighbor, newDist});
            }
        }
    }

    if (!visited[goal]) {
        return {{}, 0.0};
    }

    vector<Node*> path;
    double totalWeight = 0.0;
    for (Node* at = goal; at != nullptr; at = cameFrom[at]) {
        if (cameFrom[at]) {
            for (const auto& edge : cameFrom[at]->edges) {
                if (edge.first == at) {
                    totalWeight += edge.second;
                    break;
                }
            }
        }
        path.push_back(at);
    }
    reverse(path.begin(), path.end());
    return {path, totalWeight};
}

// Реализация BFS для поиска кратчайшего пути
pair<vector<Node*>, double> bfsShortestPath(Node* start, Node* goal) {
    if (!start || !goal) {
        return {{}, 0.0};
    }

    unordered_map<Node*, bool> visited;
    unordered_map<Node*, Node*> cameFrom;
    unordered_map<Node*, double> distance;
    queue<pair<Node*, double>> queue;

    queue.push({start, 0.0});
    distance[start] = 0.0;

    while (!queue.empty()) {
        auto [current, currDist] = queue.front();
        queue.pop();

        if (visited[current]) {
            continue;
        }

        visited[current] = true;

        if (current == goal) {
            break;
        }

        for (const auto& edge : current->edges) {
            Node* neighbor = edge.first;
            double edgeWeight = edge.second;

            double newDist = currDist + edgeWeight;

            if (!visited[neighbor] && (distance.find(neighbor) == distance.end() || newDist < distance[neighbor])) {
                distance[neighbor] = newDist;
                cameFrom[neighbor] = current;
                queue.push({neighbor, newDist});
            }
        }
    }

    if (!visited[goal]) {
        return {{}, 0.0};
    }

    vector<Node*> path;
    double totalWeight = 0.0;
    for (Node* at = goal; at != nullptr; at = cameFrom[at]) {
        if (cameFrom[at]) {
            for (const auto& edge : cameFrom[at]->edges) {
                if (edge.first == at) {
                    totalWeight += edge.second;
                    break;
                }
            }
        }
        path.push_back(at);
    }
    reverse(path.begin(), path.end());
    return {path, totalWeight};
}

// Реализация алгоритма Дейкстры для поиска кратчайшего пути
pair<vector<Node*>, double> dijkstraShortestPath(Node* start, Node* goal) {
    if (!start || !goal) {
        return {{}, 0.0};
    }

    unordered_map<Node*, double> distance;
    unordered_map<Node*, Node*> cameFrom;
    set<pair<double, Node*>> priorityQueue;

    distance[start] = 0.0;
    priorityQueue.insert({0.0, start});

    while (!priorityQueue.empty()) {
        auto [currDist, current] = *priorityQueue.begin();
        priorityQueue.erase(priorityQueue.begin());

        if (current == goal) {
            break;
        }

        for (const auto& edge : current->edges) {
            Node* neighbor = edge.first;
            double edgeWeight = edge.second;

            double newDist = currDist + edgeWeight;

            if (distance.find(neighbor) == distance.end() || newDist < distance[neighbor]) {
                priorityQueue.erase({distance[neighbor], neighbor});
                distance[neighbor] = newDist;
                cameFrom[neighbor] = current;
                priorityQueue.insert({newDist, neighbor});
            }
        }
    }

    if (distance.find(goal) == distance.end()) {
        return {{}, 0.0};
    }

    vector<Node*> path;
    double totalWeight = distance[goal];
    for (Node* at = goal; at != nullptr; at = cameFrom[at]) {
        path.push_back(at);
    }
    reverse(path.begin(), path.end());
    return {path, totalWeight};
}

// Реализация алгоритма A* для поиска кратчайшего пути
pair<vector<Node*>, double> aStarShortestPath(Node* start, Node* goal) {
    if (!start || !goal) {
        return {{}, 0.0};
    }

    auto heuristic = [](Node* a, Node* b) {
        return sqrt(pow(a->lon - b->lon, 2) + pow(a->lat - b->lat, 2));
    };

    unordered_map<Node*, double> gScore;
    unordered_map<Node*, double> fScore;
    unordered_map<Node*, Node*> cameFrom;
    set<pair<double, Node*>> openSet;

    gScore[start] = 0.0;
    fScore[start] = heuristic(start, goal);
    openSet.insert({fScore[start], start});

    while (!openSet.empty()) {
        auto [currentF, current] = *openSet.begin();
        openSet.erase(openSet.begin());

        if (current == goal) {
            break;
        }

        for (const auto& edge : current->edges) {
            Node* neighbor = edge.first;
            double edgeWeight = edge.second;

            double tentativeGScore = gScore[current] + edgeWeight;

            if (gScore.find(neighbor) == gScore.end() || tentativeGScore < gScore[neighbor]) {
                gScore[neighbor] = tentativeGScore;
                fScore[neighbor] = tentativeGScore + heuristic(neighbor, goal);
                cameFrom[neighbor] = current;
                openSet.insert({fScore[neighbor], neighbor});
            }
        }
    }

    if (gScore.find(goal) == gScore.end()) {
        return {{}, 0.0};
    }

    vector<Node*> path;
    double totalWeight = gScore[goal];
    for (Node* at = goal; at != nullptr; at = cameFrom[at]) {
        path.push_back(at);
    }
    reverse(path.begin(), path.end());
    return {path, totalWeight};
}

int main() {
    const string filename = "C:/Users/mosko/CLionProjects/AlgosiItmo/polygon/data/spb_graph.txt";
    Graph graph = readGraphFromFile(filename);

    double startLon = 30.4141326, startLat = 59.9470649;
    double goalLon = 30.3132204, goalLat = 59.9574788;

    Node* start = graph.getNode(startLon, startLat);
    Node* goal = graph.getNode(goalLon, goalLat);

    if (!start || !goal) {
        cerr << "Начало или конец не найдены" << endl;
        return 1;
    }

    auto starttimedfs = high_resolution_clock::now();

    auto [dfsPath, dfsWeight] = dfsShortestPath(start, goal);

    auto endtimedfs = high_resolution_clock::now();

    auto dfsDuration = duration_cast<microseconds>(endtimedfs - starttimedfs).count();

    if (dfsPath.empty()) {
        cout << "путь для DFS не найден" << endl;
    } else {
        cout << "DFS вес пути: " << dfsWeight << endl;
        cout << "DFS" << " завершен за: " << dfsDuration << " микросекунд" << endl;
    }

    auto starttimebfs = high_resolution_clock::now();

    auto [bfsPath, bfsWeight] = bfsShortestPath(start, goal);

    auto endtimebfs = high_resolution_clock::now();

    auto bfsDuration = duration_cast<microseconds>(endtimebfs - starttimebfs).count();

    if (bfsPath.empty()) {
        cout << "путь для BFS не найден" << endl;
    } else {
        cout << "BFS вес пути: " << bfsWeight << endl;
        cout << "BFS" << " завершен за: " << bfsDuration << " микросекунд" << endl;
    }

    auto starttimedijk = high_resolution_clock::now();

    auto [dijkstraPath, dijkstraWeight] = dijkstraShortestPath(start, goal);

    auto endtimedijk = high_resolution_clock::now();

    auto dijkDuration = duration_cast<microseconds>(endtimedijk - starttimedijk).count();

    if (dijkstraPath.empty()) {
        cout << "путь для Dijkstra не найден" << endl;
    } else {
        cout << "Dijkstra вес пути: " << dijkstraWeight << endl;
        cout << "Dijkstra" << " завершен за: " << dijkDuration << " микросекунд" << endl;
    }

    auto starttimea = high_resolution_clock::now();

    auto [aStarPath, aStarWeight] = aStarShortestPath(start, goal);

    auto endtimea = high_resolution_clock::now();

    auto aDuration = duration_cast<microseconds>(endtimea - starttimea).count();

    if (aStarPath.empty()) {
        cout << "путь для A* не найден" << endl;
    } else {
        cout << "A* вес пути: " << aStarWeight << endl;
        cout << "A*" << " завершен за: " << aDuration << " микросекунд" << endl;
    }
    return 0;
}
