#pragma once
#include <gtest/gtest.h>
#include "foundamental.h"

TEST(SampsonDistance, normalOutput)
{
	Eigen::Matrix<float, 3, 3> vFoundamentalMatrix;
	vFoundamentalMatrix << 0.29729533220101972, 0.11398894163601332, -0.21587164759595029
		, -0.50272650233060057, 0.74764969515802793, 0.030327555769636270
		, 0.15937047976956367, -0.071195115517615426, -0.068632958518681272;
	std::pair<glm::vec3, glm::vec3> matchResult;
	matchResult.first[0] = 0.21810913085937500;
	matchResult.first[1] = -0.21665555238723755;
	matchResult.first[2] = 0;

	matchResult.second[0] = 0.14227429032325745;
	matchResult.second[1] = -0.17698724567890167;
	matchResult.second[2] = 0;

	double expectedError = 2.6342750315477530e-06;
	double actualError = sampsonDistance(vFoundamentalMatrix, matchResult);
	EXPECT_LE(abs(actualError - expectedError), 1e-7);
}

TEST(Estimate8Point, normalOutput)
{
	PairWiseCamera pairCameras;
	pairCameras.matchResult.push_back(std::make_pair(
		glm::vec3(-0.22220300137996674, -0.19599626958370209,1)
		, glm::vec3(-0.17978143692016602, -0.17929978668689728,1)));
	pairCameras.matchResult.push_back(std::make_pair(
		glm::vec3(-0.17707590758800507, -0.10022066533565521,1)
		, glm::vec3(-0.17288565635681152, -0.11641129851341248,1)));
	pairCameras.matchResult.push_back(std::make_pair(
		glm::vec3(0.064062662422657013, 0.22408773005008698,1)
		, glm::vec3(-0.072953775525093079, 0.10470202565193176,1)));
	pairCameras.matchResult.push_back(std::make_pair(
		glm::vec3(0.14967988431453705, -0.023013081401586533,1)
		, glm::vec3(0.13178914785385132, -0.17159436643123627,1)));
	pairCameras.matchResult.push_back(std::make_pair(
		glm::vec3(0.25382316112518311, -0.24619381129741669,1)
		, glm::vec3(0.16796284914016724, -0.17208206653594971,1)));
	pairCameras.matchResult.push_back(std::make_pair(
		glm::vec3(-0.36967739462852478, -0.010102783329784870,1)
		, glm::vec3(-0.30407378077507019, -0.079490944743156433,1)));
	pairCameras.matchResult.push_back(std::make_pair(
		glm::vec3(-0.43051734566688538, -0.21434997022151947,1)
		, glm::vec3(-0.28794512152671814, -0.21774385869503021,1)));
	pairCameras.matchResult.push_back(std::make_pair(
		glm::vec3(-0.39046955108642578, 0.047898761928081512,1)
		, glm::vec3(-0.32208716869354248, -0.045836668461561203,1)));

	Eigen::VectorXf expectedError(9);
	expectedError << -0.074253865292759721, -0.63010132601381974, -0.33769278517261797
		, 0.51352491486049878, -0.034520716463675029, -0.37163590769382021
		, 0.25443429905947856, 0.11802224150632384, -0.041754488668258122;

	Eigen::VectorXf actualDoundamental = estimate8Points(pairCameras);
	EXPECT_EQ(true, expectedError.isApprox(-actualDoundamental, 1e-3));
	//cout << actualDoundamental << endl;
	//cout << expectedError << endl;
}

TEST(EnforceConstrain, normalOutput)
{
	Eigen::VectorXf vSolution(9);
	vSolution << 0.28837330617455809, 0.10860346282308511, -0.23100268382932154
		, -0.49851682974262967, 0.75019072142993459, 0.037466820805436014
		, 0.18929315223968329, -0.053133307791873363, -0.017886520433671036;
	Eigen::Matrix<float, 3, 3> expectedError;
	expectedError << 0.29729533220101972, 0.11398894163601332, -0.21587164759595029
		, -0.50272650233060057, 0.74764969515802793, 0.030327555769636270
		, 0.15937047976956367, -0.071195115517615426, -0.068632958518681272;
	Eigen::Matrix<float, 3, 3> actualDoundamental = enforceConstrains(vSolution);
	EXPECT_EQ(true, (actualDoundamental.isApprox(expectedError, 1e-6)));
	//cout << actualDoundamental << endl;
	//cout << expectedError << endl;
}

TEST(SymmetricError, normalOutput)
{
	Eigen::Matrix<float, 3, 3> vHomographyMatrix;
	vHomographyMatrix << 1.3582021842831014, -0.043742832197414878, 0.12184747535436576
		, -0.078472327741160336, 1.1461194005242523, 0.048736410468195379
		, -0.13375965567861531, 0.68708451571155349, 1.0000000000000000;
	glm::mat homo = glmFromEigen<3,3>(vHomographyMatrix);
	std::pair<glm::vec3, glm::vec3> matchResult;
	matchResult.first[0] = -0.30842339992523193;
	matchResult.first[1] = -0.15599510073661804;
	matchResult.first[2] = 0;

	matchResult.second[0] = -0.31903117895126343;
	matchResult.second[1] = -0.11188735067844391;
	matchResult.second[2] = 0;

	double expectedError = 5.3751678625243315e-05;
	double actualError = symmetricTransferError(homo, matchResult);
	EXPECT_LE(abs(actualError - expectedError), 1e-7);
}

TEST(TriangulateTrackError, normalOutput)
{
	std::vector<glm::vec2> vPostion;
	std::vector<CameraPose> vPoses;
	vPostion.push_back(glm::vec2(-0.291181982f, -0.192279682f));
	vPostion.push_back(glm::vec2(-0.205872044f, -0.0936535671f));

	CameraPose pose1, pose2;
	pose2.R[0][0] = 0.99941037176556757;
	pose2.R[0][1] = 0.034266463962079718;
	pose2.R[0][2] = -0.0021721544479905153;
	pose2.R[1][0] = -0.033268140623173625;
	pose2.R[1][1] = 0.98205948907569274;
	pose2.R[1][2] = 0.18561355213417374;
	pose2.R[2][0] = 0.0084935049824663711;
	pose2.R[2][1] = -0.18543184560351150;
	pose2.R[2][2] = 0.98262042061479071;
	pose2.T[0] = 0.89617069378421477;
	pose2.T[1] = -0.15643556371642153;
	pose2.T[2] = 0.41521801744028974;
	vPoses.push_back(pose1);
	vPoses.push_back(pose2);

	glm::vec3 expectedPos = glm::vec3(-2.8591628206205266, -2.2887103047447170, 9.6374448796934935);
	glm::vec3 actualPos = triangulateTrack(vPostion, vPoses);
	EXPECT_LE(abs(glm::distance(expectedPos , actualPos)), 1e-5);
}

TEST(PoseFromEssensial, normalOutput)
{
	Eigen::Matrix3f e;
	e << 9.0270081951060455e-07, 0.26131201394831827, 0.13855922416246347
		, -0.29535328927602827, -0.18397842245636264, 0.61092372228775027
		, -0.11127784762870724, -0.63330810807276505, -0.068885544371089680;
	glm::mat3 vEssensialMatrix = glmFromEigen<3, 3>(e);
	std::vector<CameraPose> vPoses;
	poseFromEssential(vEssensialMatrix, vPoses);

	glm::mat3 finalR;
	glm::vec3 finalT;
	CameraPose pose;
	glm::mat3 K = glm::identity<glm::mat3>();
	for (std::size_t i = 0; i < vPoses.size(); ++i) {
		std::pair<glm::vec2, glm::vec2> matchPos = std::make_pair<glm::vec2, glm::vec2>(
			glm::vec2(-0.29118198156356812, -0.19227968156337738)
			, glm::vec2(-0.20587204396724701, -0.093653567135334015));
		glm::vec3 x = triangulateMatch(matchPos, pose, vPoses[i]);
		glm::vec3 x1 = pose.R * x + pose.T;
		glm::vec3 x2 = vPoses[i].R * x + vPoses[i].T;

		if (x1[2] > 0.0f && x2[2] > 0.0f) {
			finalR= vPoses[i].R;
			finalT = vPoses[i].T;
			break;
		}
	}

	Eigen::Matrix3f expectR;
	expectR << 0.99941037176556757, 0.034266463962079718, -0.0021721544479905153
		, -0.033268140623173625, 0.98205948907569274, 0.18561355213417374
		, 0.0084935049824663711, -0.18543184560351150, 0.98262042061479071;

	glm::vec3 expectedT = glm::vec3(0.89617069378421477, -0.15643556371642153, 0.41521801744028974);
	EXPECT_LE(glm::distance(expectedT,finalT), 1e-5);
	EXPECT_EQ(expectR.isApprox(eigenFromGLM<3, 3>(finalR), 1e-5), true);
}


void unitTest(int argc, char* argv[]) {
	testing::InitGoogleTest(&argc, argv);
	RUN_ALL_TESTS();
}