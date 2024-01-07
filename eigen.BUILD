package(
    default_visibility = ["//visibility:public"],
)

cc_library(
    name = "eigen",
    hdrs = glob(
        ["Eigen/**"],
        exclude = [
            # "Eigen/src/OrderingMethods/Amd.h",
            # "Eigen/src/SparseCholesky/**",
            # "Eigen/Eigen",
            # "Eigen/IterativeLinearSolvers",
            # "Eigen/MetisSupport",
            # "Eigen/Sparse",
            # "Eigen/SparseCholesky",
            # "Eigen/SparseLU",
        ],
    ),
    defines = [
        # "EIGEN_MPL_ONLY",
        # "EIGEN_NO_DEBUG",
    ],
    includes = ["."],
)