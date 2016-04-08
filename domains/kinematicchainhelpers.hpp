#pragma once

#include <ompl/base/spaces/SE3StateSpace.h>

namespace KinematicChainHelpers {
	ompl::base::SO3StateSpace SO3;

	struct TransformPair {
		TransformPair(const std::vector<double> &translation, const std::vector<double> &rotation) :
			translation(translation), rotation(rotation) {}
		TransformPair(const TransformPair &tp) : translation(tp.translation), rotation(tp.rotation) {}
		std::vector<double> translation, rotation;
	};

	void multiply(const std::vector<double> &m1, const std::vector<double> &m2, std::vector<double> &out) {
		std::vector<double> temp(16);
		for(unsigned int row = 0; row < 4; ++row) {
			for(unsigned int col = 0; col < 4; ++col) {
				double sum = 0;
				for(unsigned int i = 0; i < 4; i++) {
					double elem1 = m1[row * 4 + i];
					double elem2 = m2[col + 4 * i];
					sum += elem1 * elem2;
				}
				temp[row * 4 + col] = sum;
			}
		}
		for(unsigned int i = 0; i < 16; i++) out[i] = temp[i];
	}

	void quaternionToMatrix3x3(const ompl::base::SO3StateSpace::StateType &rot, std::vector<double> &matrix) {
		matrix[0] = 1 - 2 * rot.y * rot.y - 2 * rot.z * rot.z;
		matrix[1] = 2 * rot.x * rot.y - 2 * rot.z * rot.w;
		matrix[2] = 2 * rot.x * rot.z + 2 * rot.y * rot.w;
		matrix[3] = 2 * rot.x * rot.y + 2 * rot.z * rot.w;
		matrix[4] = 1 - 2 * rot.x * rot.x - 2 * rot.z * rot.z;
		matrix[5] = 2 * rot.y * rot.z - 2 * rot.x * rot.w;
		matrix[6] = 2 * rot.x * rot.z - 2 * rot.y * rot.w;
		matrix[7] = 2 * rot.y * rot.z + 2 * rot.x * rot.w;
		matrix[8] = 1 - 2 * rot.x * rot.x - 2 * rot.y * rot.y;
	}

	void quaternionToMatrix4x4(const ompl::base::SO3StateSpace::StateType &rot, std::vector<double> &matrix) {
		matrix[0] = 1 - 2 * rot.y * rot.y - 2 * rot.z * rot.z;
		matrix[1] = 2 * rot.x * rot.y - 2 * rot.z * rot.w;
		matrix[2] = 2 * rot.x * rot.z + 2 * rot.y * rot.w;
		matrix[3] = 0;
		matrix[4] = 2 * rot.x * rot.y + 2 * rot.z * rot.w;
		matrix[5] = 1 - 2 * rot.x * rot.x - 2 * rot.z * rot.z;
		matrix[6] = 2 * rot.y * rot.z - 2 * rot.x * rot.w;
		matrix[7] = 0;
		matrix[8] = 2 * rot.x * rot.z - 2 * rot.y * rot.w;
		matrix[9] = 2 * rot.y * rot.z + 2 * rot.x * rot.w;
		matrix[10] = 1 - 2 * rot.x * rot.x - 2 * rot.y * rot.y;
		matrix[11] = 0;
		matrix[12] = 0;
		matrix[13] = 0;
		matrix[14] = 0;
		matrix[15] = 1;
	}

	double getRotationAngle(const std::vector<double> &a, const std::vector<double> &b, unsigned int axis) {
		double w0 = sqrt(1 + a[0] + a[5] + a[10]) / 2;
		double x0 = (a[9] - a[6])/( 4 * w0);
		double y0 = (a[2] - a[8])/( 4 * w0);
		double z0 = (a[4] - a[1])/( 4 * w0);

		double w1 = sqrt(1 + b[0] + b[5] + b[10]) / 2;
		double x1 = (b[9] - b[6])/( 4 * w1);
		double y1 = (b[2] - b[8])/( 4 * w1);
		double z1 = (b[4] - b[1])/( 4 * w1);

		double innerProduct = w0 * w1 + x0 * x1 + y0 * y1 + z0 * z1;
		double angle = acos(2 * innerProduct * innerProduct - 1);

		std::vector<double> innerRotation(16, 0);
		innerRotation[0] = innerRotation[5] = innerRotation[10] = innerRotation[15] = 1;
		if(axis == 0) { // rotate around x
			innerRotation[5] = cos(angle);
			innerRotation[6] = -sin(angle);
			innerRotation[9] = sin(angle);
			innerRotation[10] = cos(angle);
		} else if(axis == 1) { //rotate around y
			innerRotation[0] = cos(angle);
			innerRotation[2] = sin(angle);
			innerRotation[8] = -sin(angle);
			innerRotation[10] = cos(angle);
		} else if(axis == 2) { //rotate around z
			innerRotation[0] = cos(angle);
			innerRotation[1] = -sin(angle);
			innerRotation[4] = sin(angle);
			innerRotation[5] = cos(angle);
		}

		multiply(a, innerRotation, innerRotation);


		for(unsigned int i = 0; i < 16; i++) {
			if(fabs(b[i] - innerRotation[i]) >= 0.0000001) {
				angle *= -1;
				break;
			}
		}

		return angle;
	}

	void rotatePoint(const ompl::base::SO3StateSpace::StateType &quat, const std::vector<double> &vec, std::vector<double> &result) {
		float num = quat.x * 2;
		float num2 = quat.y * 2;
		float num3 = quat.z * 2;
		float num4 = quat.x * num;
		float num5 = quat.y * num2;
		float num6 = quat.z * num3;
		float num7 = quat.x * num2;
		float num8 = quat.x * num3;
		float num9 = quat.y * num3;
		float num10 = quat.w * num;
		float num11 = quat.w * num2;
		float num12 = quat.w * num3;
		
		double x = (1 - (num5 + num6)) * vec[0] + (num7 - num12) * vec[1] + (num8 + num11) * vec[2];
		double y = (num7 + num12) * vec[0] + (1 - (num4 + num6)) * vec[1] + (num9 - num10) * vec[2];
		double z = (num8 - num11) * vec[0] + (num9 + num10) * vec[1] + (1 - (num4 + num5)) * vec[2];

		result[0] = x;
		result[1] = y;
		result[2] = z;
	}

	TransformPair getTransformPair(const std::vector<double> &transform) {
		std::vector<double> translation(16, 0);
		translation[0] = translation[5] = translation[10] = translation[15] = 1;
		translation[3] = transform[0];
		translation[7] = transform[1];
		translation[11] = transform[2];

		std::vector<double> rotation(16, 0);
		rotation[0] = rotation[5] = rotation[10] = rotation[15] = 1;
		for(unsigned int j = 0; j < 3; j++) {
			double rotValue = transform[3 + j];

			if(rotValue == 0) continue;

			std::vector<double> innerRotation(16, 0);
			innerRotation[0] = innerRotation[5] = innerRotation[10] = innerRotation[15] = 1;
			if(j == 0) { // rotate around x
				innerRotation[5] = cos(rotValue);
				innerRotation[6] = -sin(rotValue);
				innerRotation[9] = sin(rotValue);
				innerRotation[10] = cos(rotValue);
			} else if(j == 1) { //rotate around y
				innerRotation[0] = cos(rotValue);
				innerRotation[2] = sin(rotValue);
				innerRotation[8] = -sin(rotValue);
				innerRotation[10] = cos(rotValue);
			} else if(j == 2) { //rotate around z
				innerRotation[0] = cos(rotValue);
				innerRotation[1] = -sin(rotValue);
				innerRotation[4] = sin(rotValue);
				innerRotation[5] = cos(rotValue);
			}
			multiply(innerRotation, rotation, rotation);
		}

		return TransformPair(translation, rotation);
	}

	void getJointAngles(const ompl::base::State *state, const std::vector<TransformPair> &transformPairs,
		const std::vector<std::vector<double>> &jointAxes, std::vector<double> &jointAngles) {

		auto compoundState = state->as<ompl::base::CompoundStateSpace::StateType>();

		std::vector<double> transform(16);
		transform[0] = transform[5] = transform[10] = transform[15] = 1;

		unsigned int transformPairIndex = 0; 
		for(unsigned int linkIndex = 0; linkIndex < 10; linkIndex++) {

			std::vector<double> rotation(16);
			rotation[0] = rotation[5] = rotation[10] = rotation[15] = 1;
			for(unsigned int i = 0; i < 2; i++) {
				const auto &matrixTransform = transformPairs[transformPairIndex++];
				multiply(transform, matrixTransform.translation, transform);
				multiply(matrixTransform.rotation, rotation, rotation);

				if(transformPairIndex % 2 == 1) {
					unsigned int which = transformPairIndex/2;
					unsigned int axis = 0;					

					if(std::fabs(jointAxes[which][0]) > 0) {
						axis = 0;
					} else if(std::fabs(jointAxes[which][1]) > 0) {
						axis = 1;
					} else if(std::fabs(jointAxes[which][2]) > 0) {
						axis = 2;
					} else {
						assert(false);
					}

					std::vector<double> stateTransform(16);
					quaternionToMatrix4x4(compoundState->as<ompl::base::SE3StateSpace::StateType>(linkIndex)->rotation(), stateTransform);

					double rotValue = getRotationAngle(transform, stateTransform, axis);
					jointAngles[which] = rotValue;

					std::vector<double> innerRotation(16, 0);
					innerRotation[0] = innerRotation[5] = innerRotation[10] = innerRotation[15] = 1;
					if(axis == 0) { // rotate around x
						innerRotation[5] = cos(rotValue);
						innerRotation[6] = -sin(rotValue);
						innerRotation[9] = sin(rotValue);
						innerRotation[10] = cos(rotValue);
					} else if(axis == 1) { //rotate around y
						innerRotation[0] = cos(rotValue);
						innerRotation[2] = sin(rotValue);
						innerRotation[8] = -sin(rotValue);
						innerRotation[10] = cos(rotValue);
					} else if(axis == 2) { //rotate around z
						innerRotation[0] = cos(rotValue);
						innerRotation[1] = -sin(rotValue);
						innerRotation[4] = sin(rotValue);
						innerRotation[5] = cos(rotValue);
					}

					multiply(transform, innerRotation, transform);
				}
			}
		}
	}

	void printDrawableState(const ompl::base::State *state) {
		fprintf(stderr, "\n");
		for(unsigned int i = 0; i < 10; i++) {
			const auto s = state->as<ompl::base::CompoundStateSpace::StateType>()->as<ompl::base::SE3StateSpace::StateType>(i);
			const auto &rot = s->rotation();

			fprintf(stderr, "{%g, %g, %g, %g, %g, %g, %g},\n", s->getX(), s->getY(), s->getZ(), rot.w, rot.x, rot.y, rot.z);
		}
		fprintf(stderr, "\n");
	}

	void translateIntoState(const std::vector<double> &transform, ompl::base::SE3StateSpace::StateType *stateSE3) {
		stateSE3->setXYZ(transform[3], transform[7], transform[11]);
		auto &so3 = stateSE3->rotation();
		so3.w = sqrt(1 + transform[0] + transform[5] + transform[10]) / 2;
		so3.x = (transform[9] - transform[6])/( 4 * so3.w);
		so3.y = (transform[2] - transform[8])/( 4 * so3.w);
		so3.z = (transform[4] - transform[1])/( 4 * so3.w);

		SO3.enforceBounds(&so3);
	}

	void buildState(const std::vector<double> &jointAngles, const std::vector<TransformPair> &transformPairs,
		const std::vector<std::vector<double>> &jointAxes, ompl::base::State *result) {

		auto compoundState = result->as<ompl::base::CompoundStateSpace::StateType>();

		std::vector<double> transform(16);
		transform[0] = transform[5] = transform[10] = transform[15] = 1;

		unsigned int transformPairIndex = 0; 
		for(unsigned int linkIndex = 0; linkIndex < 10; linkIndex++) {

			std::vector<double> rotation(16);
			rotation[0] = rotation[5] = rotation[10] = rotation[15] = 1;
			for(unsigned int i = 0; i < 2; i++) {
				const auto &matrixTransform = transformPairs[transformPairIndex++];
				multiply(transform, matrixTransform.translation, transform);
				multiply(matrixTransform.rotation, rotation, rotation);

				if(transformPairIndex % 2 == 1) {
					unsigned int which = transformPairIndex/2;
					unsigned int axis = 0;
					double rotValue = jointAngles[which];

					if(std::fabs(jointAxes[which][0]) > 0) {
						axis = 0;
					} else if(std::fabs(jointAxes[which][1]) > 0) {
						axis = 1;
					} else if(std::fabs(jointAxes[which][2]) > 0) {
						axis = 2;
					} else {
						assert(false);
					}

					if(std::fabs(jointAxes[which][axis]) < 0) {
						rotValue *= -1;
					}

					std::vector<double> innerRotation(16, 0);
					innerRotation[0] = innerRotation[5] = innerRotation[10] = innerRotation[15] = 1;
					if(axis == 0) { // rotate around x
						innerRotation[5] = cos(rotValue);
						innerRotation[6] = -sin(rotValue);
						innerRotation[9] = sin(rotValue);
						innerRotation[10] = cos(rotValue);
					} else if(axis == 1) { //rotate around y
						innerRotation[0] = cos(rotValue);
						innerRotation[2] = sin(rotValue);
						innerRotation[8] = -sin(rotValue);
						innerRotation[10] = cos(rotValue);
					} else if(axis == 2) { //rotate around z
						innerRotation[0] = cos(rotValue);
						innerRotation[1] = -sin(rotValue);
						innerRotation[4] = sin(rotValue);
						innerRotation[5] = cos(rotValue);
					}

					multiply(transform, innerRotation, transform);
				}

			}

			std::vector<double> transform2(16);
			transform2[0] = transform2[5] = transform2[10] = transform2[15] = 1;
			
			multiply(transform, rotation, transform2);

			translateIntoState(transform2, compoundState->as<ompl::base::SE3StateSpace::StateType>(linkIndex));
		}
	}
};



