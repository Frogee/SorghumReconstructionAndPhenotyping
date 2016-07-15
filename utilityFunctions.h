
#ifndef UTILITYFUNCTIONS_H
#define UTILITYFUNCTIONS_H

#include <Eigen/Core>

Eigen::Matrix4f returnRotationMatrixToTranslateFirstNormalToSecondNormal(Eigen::Vector3f inputFirstNormal, Eigen::Vector3f inputSecondNormal);

#endif
