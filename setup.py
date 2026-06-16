#!python
from pybind11.setup_helpers import Pybind11Extension
from setuptools import setup
import platform

include_dirs = [
    "CPP/external/eigen",
    "CPP/external/ik-geo/cpp/subproblems",
    "CPP/src/IK",
    "CPP/src/IK/utils",
    "CPP/src",
    "CPP/src/utils",
]

ext_modules = [
    Pybind11Extension(
        "eaik.pybindings.EAIK",
        sorted((
            "src/eaik/pybindings/eaik_pybindings.cpp",
            "CPP/src/IK/utils/kinematic_utils.cpp",
            "CPP/src/IK/1R_IK.cpp",
            "CPP/src/IK/2R_IK.cpp",
            "CPP/src/IK/3R_IK.cpp",
            "CPP/src/IK/4R_IK.cpp",
            "CPP/src/IK/5R_IK.cpp",
            "CPP/src/IK/6R_IK.cpp",
            "CPP/src/EAIK.cpp",
            "CPP/src/utils/kinematic_remodeling.cpp",
            "CPP/external/ik-geo/cpp/subproblems/sp.cpp",
        )),
        include_dirs=[str(p) for p in include_dirs],
        extra_compile_args=["/std:c++17"] if platform.system()=="Windows" else ["-std=c++17"],
    )
]

if __name__ == "__main__":
    setup(ext_modules=ext_modules)
