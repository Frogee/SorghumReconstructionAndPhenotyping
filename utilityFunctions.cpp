
#include "utilityFunctions.h"

Eigen::Matrix4f returnRotationMatrixToTranslateFirstNormalToSecondNormal(Eigen::Vector3f inputFirstNormal, Eigen::Vector3f inputSecondNormal) {
    Eigen::Matrix4f rotationMatrixToReturn = Eigen::Matrix4f::Identity();

    Eigen::Vector3f vector_A = inputFirstNormal;
    Eigen::Vector3f vector_B = inputSecondNormal;

    // This is an implementation of Rodrigues' Rotation Formula
    // https://en.wikipedia.org/wiki/Rodrigues'_rotation_formula
    // Following http://math.stackexchange.com/questions/293116/rotating-one-3-vector-to-another?rq=1
    // Problem: Given two 3-vectors, A and B, find the rotation of A so that its orientation matches B.
    // There are some edge cases where this implementation will fail, notably if the norm of the cross product = 0.

    // Step 1: Find axis (X)
    Eigen::Vector3f crossProduct = vector_A.cross(vector_B);
    float crossProductNorm = crossProduct.norm();
    Eigen::Vector3f vector_X = (crossProduct / crossProductNorm);

    // Step 2: Find angle (theta)
    float dotProduct = vector_A.dot(vector_B);
    float norm_A = vector_A.norm();
    float norm_B = vector_B.norm();
    float dotProductOfNorms = norm_A * norm_B;
    float dotProductDividedByDotProductOfNorms = (dotProduct / dotProductOfNorms);
    float thetaAngleRad = acos(dotProductDividedByDotProductOfNorms);

    // Step 3: Construct A, the skew-symmetric matrix corresponding to X
    Eigen::Matrix3f matrix_A = Eigen::Matrix3f::Identity();

    matrix_A(0,0) = 0.0;
    matrix_A(0,1) = -1.0 * (vector_X(2));
    matrix_A(0,2) = vector_X(1);
    matrix_A(1,0) = vector_X(2);
    matrix_A(1,1) = 0.0;
    matrix_A(1,2) = -1.0 * (vector_X(0));
    matrix_A(2,0) = -1.0 * (vector_X(1));
    matrix_A(2,1) = vector_X(0);
    matrix_A(2,2) = 0.0;

    // Step 4: Plug and chug.
    Eigen::Matrix3f IdentityMat = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f firstTerm = sin(thetaAngleRad) * matrix_A;
    Eigen::Matrix3f secondTerm = (1.0 - cos(thetaAngleRad)) * matrix_A * matrix_A;

    Eigen::Matrix3f matrix_R = IdentityMat + firstTerm + secondTerm;

    // We copy the rotation matrix into the matrix that will be used for the transformation.
    Eigen::Matrix4f Transform = Eigen::Matrix4f::Identity();
    Transform(0,0) = matrix_R(0,0);
    Transform(0,1) = matrix_R(0,1);
    Transform(0,2) = matrix_R(0,2);
    Transform(1,0) = matrix_R(1,0);
    Transform(1,1) = matrix_R(1,1);
    Transform(1,2) = matrix_R(1,2);
    Transform(2,0) = matrix_R(2,0);
    Transform(2,1) = matrix_R(2,1);
    Transform(2,2) = matrix_R(2,2);

    // Add the translation to the matrix as 0.
    Transform(0,3) = 0.0;
    Transform(1,3) = 0.0;
    Transform(2,3) = 0.0;

    rotationMatrixToReturn = Transform;

    return(rotationMatrixToReturn);
}
