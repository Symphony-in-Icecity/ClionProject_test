// any_of example
#include <iostream>     // std::cout
#include <algorithm>    // std::any_of
#include <array>        // std::array

double MAX_SPEED = 50.0 / 3.6;

int main () {
    std::vector<double> foo = {0,1,-1,3,-3,5,-5.0,50.0};

    if ( std::any_of(foo.begin(), foo.end(), [](double v){return v > MAX_SPEED;}) )
        std::cout << "There are negative elements in the range.\n";

    return 0;
}
