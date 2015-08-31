// test up to what log can w
#include <stdexcept>
#include <exception>
#include <math.h>
#include <iostream>

int main(int argv, char** argc)
{
    try{
        int x = -746;
        double y = exp(x);
        std::cout.precision(10);
        std::cout << std::scientific << y << "\n";
    }
    catch(std::underflow_error& e){
        std::cout << "exception caught: " << e.what() << std::endl;
    }
    return 0;
}