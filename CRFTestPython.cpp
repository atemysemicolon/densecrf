//
// Created by prasscvc on 22/06/15.
//

#ifndef CRFNEW_DUMMY_H
#define CRFNEW_DUMMY_H

#include <iostream>
#include <boost/python.hpp>
class abc
{
public:
    void dispHello()
    {
        std::cout<<"Hello";
    }

    int retInt(int a,int b)
    {
        return (a+b);
    }

};


BOOST_PYTHON_MODULE(libyay)
{
        // An established convention for using boost.python.
        using namespace boost::python;

        // Expose the class Animal.
        class_<abc>("ABC")
        .def("dispHello", &abc::dispHello)
        .def("retInt", &abc::retInt)
        ;
}

#endif //CRFNEW_DUMMY_H
