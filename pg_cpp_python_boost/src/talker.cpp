#include <boost/python.hpp>
#include <iostream>

using namespace std;

void hello() {
    std::cout << "Hello World" << '\n';
}

using namespace boost::python;
BOOST_PYTHON_MODULE(hi) {
    def("hello", hello);
}
