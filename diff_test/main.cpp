#include <iostream>
#include <vector>


std::vector<double> diff(std::vector<double> a) {
    std::vector<double> a_diff;
    for (int i = 0; i < a.size() - 1; i++) {
        a_diff.push_back(a[i + 1] - a[i]);
    }

    return a_diff;
}

int main() {
    std::vector<double> a;
    std::vector<double> d;
    for (int i = 0; i < 10; ++i) {
        a.push_back(i * 0.2 * i);
        std::cout << a[i] << std::endl;
    }
    d = diff(a);
    std::cout << "d = " << std::endl;
    for (int j = 0; j < d.size(); ++j) {
        std::cout << d[j] << std::endl;
    }
    return 0;
}