#!python
from pybind11.setup_helpers import Pybind11Extension
from setuptools import setup

ext_modules = [
    Pybind11Extension(
        "eaik.cpp.canonical_subproblems",
        sorted(("src/eaik/pybindings/eaik_pybindings.cpp",
                "CPP/src/IK/General_IK.cpp",
                "CPP/src/IK/Spherical_IK.cpp",
                "CPP/src/IK/3R_IK.cpp",
                "CPP/src/EAIK.cpp",
                "CPP/src/utils/kinematic_remodelling.cpp",
                "CPP/external/ik-geo/cpp/subproblems/sp.cpp")),
        include_dirs=['CPP/external/ik-geo/cpp/subproblems','CPP/src/IK','CPP/src','CPP/src/utils','/usr/include/eigen3', '/usr/local/include/eigen3']
    )
]


if __name__ == '__main__':
    setup(ext_modules=ext_modules)
