#include <iostream>
#include <vector>

using namespace std;

class Solution {
public:
    // O(n) время
    int minNumberOperations(vector<int>& target) {
        int operations = 0; // хранение кол-во операций
        int prevWeight = 0; // вес предыдущего элемента

        for (int& weight : target) {
            if (weight > prevWeight) { // если вес больше предыдущего, то считаем, если нет -> не выполняем(оптимизация)
                operations += (weight - prevWeight);
            }
            prevWeight = weight;
        }
        return operations;
    }
};