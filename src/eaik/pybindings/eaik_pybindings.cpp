#include "IKS.h"
#include "EAIK.h"

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h> // Convert numpy <-> Eigen
#include <pybind11/stl.h>   // Convert python list <-> std::list
#include <pybind11/numpy.h>

namespace py = pybind11;


PYBIND11_MODULE(canonical_subproblems, m)
{
    m.doc() = R"pbdoc(
    Pybind11 of canonical subproblems for inverse kinematics
    by
    )pbdoc";

    /*input:
    kin: kinematics struct with form:
        root
            -> H          : [h_1 h_2 ... h_n]
                            (3 x n) actuator axes
            -> P          : [p_{O,1} p_{1,2} ... p_{n,T}]
                            (3 x n + 1) actuator displacements
            -> joint_type : n-vector of joint types
                            0 - rotational
                            1 - prismatic
                            2 - mobile orientation
                            3 - mobile translation
    */

    py::class_<IKS::IK_Solution>(m, "IKSolution")
        .def(py::init<>())
        .def_readwrite("Q", &IKS::IK_Solution::Q)
        .def_readwrite("is_LS", &IKS::IK_Solution::is_LS_vec);

    py::class_<EAIK::Robot>(m, "Robot")
        .def(py::init<const Eigen::MatrixXd &, const Eigen::MatrixXd &, bool>())
        .def("calculate_IK", &EAIK::Robot::calculate_IK, R"pbdoc(
            Run inverse kinematics.

            :param pose:  4x4 Homogeneous transformation matrix
            :return:      IK-Solution class
        )pbdoc",
             py::arg("pose"))
        .def("fwdkin", &EAIK::Robot::fwdkin, R"pbdoc(
            Run forward kinematics.

            :param Q:  Array of six joint angles
            :return:   4x4 Homogeneous transformation matrix
        )pbdoc",
             py::arg("Q"))
        .def("is_spherical", &EAIK::Robot::is_spherical, R"pbdoc(
            Returns if robot has spherical wrist.
            :return:   bool
        )pbdoc");
}
