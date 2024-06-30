#include "IKS.h"
#include "EAIK.h"

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h> // Convert numpy <-> Eigen
#include <pybind11/stl.h>   // Convert python list <-> std::list

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
    */

    py::class_<IKS::IK_Eigen_Solution>(m, "IKSolution")
        .def(py::init<>())
        .def_readwrite("Q", &IKS::IK_Eigen_Solution::Q)
        .def_readwrite("is_LS", &IKS::IK_Eigen_Solution::is_LS_vec)
        .def("num_solutions", &IKS::IK_Eigen_Solution::num_solutions, R"pbdoc(
            Returns number of total IK solutions (including Least-Squares)
            :return:   int
        )pbdoc")
        .def("__bool__", [](const IKS::IK_Eigen_Solution &self) {
        return self.num_solutions() > 0;
        });

    py::class_<EAIK::Robot>(m, "Robot")
        .def(py::init<const Eigen::MatrixXd &, const Eigen::MatrixXd &, bool>())
        .def("calculate_IK", &EAIK::Robot::calculate_Eigen_IK, R"pbdoc(
            Run inverse kinematics.

            :param pose:  4x4 Homogeneous transformation matrix
            :return:      IK-Solution class
        )pbdoc",
             py::arg("pose"))
        .def("calculate_IK_batched", &EAIK::Robot::calculate_Eigen_IK_batched, R"pbdoc(
            Run inverse kinematics for a batch of EE poses.

            :param pose:  Batch of 4x4 Homogeneous transformation matrix
            :param num_worker_threads: Number of total worker threads to assign
            :return:      Batch of IK-Solution classes
        )pbdoc",
             py::arg("pose_batch"), py::arg("num_worker_threads"))
        .def("fwdkin", &EAIK::Robot::fwdkin_Eigen, R"pbdoc(
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
