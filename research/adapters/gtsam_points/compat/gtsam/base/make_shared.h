#pragma once
// The installed GTSAM 4.3 development revision already uses std::shared_ptr
// but predates this convenience header. Prefer a real upstream header when
// available; otherwise supply only the equivalent alias, without changing ABI.
#if __has_include_next(<gtsam/base/make_shared.h>)
#include_next <gtsam/base/make_shared.h>
#else
#include <gtsam/config.h>
#include <memory>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#if GTSAM_VERSION_NUMERIC < 40300
#error "GTSAM 4.2 requires its upstream make_shared header"
#endif
namespace gtsam { using std::make_shared; }
#define GTSAM_MAKE_ALIGNED_OPERATOR_NEW EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif
