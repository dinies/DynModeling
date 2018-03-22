// Demo of sending data via temporary files.  The default is to send data to gnuplot directly
// through stdin.
//
// Compile it with:
//   g++ -o example-tmpfile example-tmpfile.cc -lboost_iostreams -lboost_system -lboost_filesystem

#include <map>
#include <vector>
#include <cmath>
#include "GNUPlot.hpp"
#include <iostream>


int main() {
    std::cout << "try" ;
    std::vector<std::string> script;
    script.push_back("set terminal qt");
    script.push_back("reset");
    script.push_back("plot sin(x)");
    
    std::cout << "try" ;
    GNUPlot plotter;
    plotter.open();
    plotter.execute(script);
    
    getchar(); // prevent graph to close
    
    plotter.write("exit");
    plotter.flush();
    plotter.close();
}
