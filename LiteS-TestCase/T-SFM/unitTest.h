#pragma once
#include <gtest/gtest.h>
#include "foundamental.h"

TEST(SampsonDistance, normalOutput)
{
	Eigen::Matrix<float, 3, 3> vFoundamentalMatrix;
	vFoundamentalMatrix << 0.29729533220101972f, 0.11398894163601332f, -0.21587164759595029f
		, -0.50272650233060057f, 0.74764969515802793f, 0.030327555769636270f
		, 0.15937047976956367f, -0.071195115517615426f, -0.068632958518681272f;
	std::pair<Eigen::Vector3f, Eigen::Vector3f> matchResult;
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
		Eigen::Vector3f(-0.22220300137996674, -0.19599626958370209,1)
		, Eigen::Vector3f(-0.17978143692016602, -0.17929978668689728,1)));
	pairCameras.matchResult.push_back(std::make_pair(
		Eigen::Vector3f(-0.17707590758800507, -0.10022066533565521,1)
		, Eigen::Vector3f(-0.17288565635681152, -0.11641129851341248,1)));
	pairCameras.matchResult.push_back(std::make_pair(
		Eigen::Vector3f(0.064062662422657013, 0.22408773005008698,1)
		, Eigen::Vector3f(-0.072953775525093079, 0.10470202565193176,1)));
	pairCameras.matchResult.push_back(std::make_pair(
		Eigen::Vector3f(0.14967988431453705, -0.023013081401586533,1)
		, Eigen::Vector3f(0.13178914785385132, -0.17159436643123627,1)));
	pairCameras.matchResult.push_back(std::make_pair(
		Eigen::Vector3f(0.25382316112518311, -0.24619381129741669,1)
		, Eigen::Vector3f(0.16796284914016724, -0.17208206653594971,1)));
	pairCameras.matchResult.push_back(std::make_pair(
		Eigen::Vector3f(-0.36967739462852478, -0.010102783329784870,1)
		, Eigen::Vector3f(-0.30407378077507019, -0.079490944743156433,1)));
	pairCameras.matchResult.push_back(std::make_pair(
		Eigen::Vector3f(-0.43051734566688538, -0.21434997022151947,1)
		, Eigen::Vector3f(-0.28794512152671814, -0.21774385869503021,1)));
	pairCameras.matchResult.push_back(std::make_pair(
		Eigen::Vector3f(-0.39046955108642578, 0.047898761928081512,1)
		, Eigen::Vector3f(-0.32208716869354248, -0.045836668461561203,1)));

	Eigen::VectorXf expectedError(9);
	expectedError << -0.074253865292759721f, -0.63010132601381974f, -0.33769278517261797f
		, 0.51352491486049878f, -0.034520716463675029f, -0.37163590769382021f
		, 0.25443429905947856f, 0.11802224150632384f, -0.041754488668258122f;

	Eigen::VectorXf actualDoundamental = estimate8Points(pairCameras);
	EXPECT_EQ(true, expectedError.isApprox(-actualDoundamental, 1e-3f));
	//cout << actualDoundamental << endl;
	//cout << expectedError << endl;
}

TEST(EnforceConstrain, normalOutput)
{
	Eigen::VectorXf vSolution(9);
	vSolution << 0.28837330617455809f, 0.10860346282308511f, -0.23100268382932154f
		, -0.49851682974262967f, 0.75019072142993459f, 0.037466820805436014f
		, 0.18929315223968329f, -0.053133307791873363f, -0.017886520433671036f;
	Eigen::Matrix<float, 3, 3> expectedError;
	expectedError << 0.29729533220101972f, 0.11398894163601332f, -0.21587164759595029f
		, -0.50272650233060057f, 0.74764969515802793f, 0.030327555769636270f
		, 0.15937047976956367f, -0.071195115517615426f, -0.068632958518681272f;
	Eigen::Matrix<float, 3, 3> actualDoundamental = enforceConstrains(vSolution);
	EXPECT_EQ(true, (actualDoundamental.isApprox(expectedError, 1e-6f)));
	//cout << actualDoundamental << endl;
	//cout << expectedError << endl;
}

TEST(SymmetricError, normalOutput)
{
	Eigen::Matrix<float, 3, 3> vHomographyMatrix;
	vHomographyMatrix << 1.3582021842831014f, -0.043742832197414878f, 0.12184747535436576f
		, -0.078472327741160336f, 1.1461194005242523f, 0.048736410468195379f
		, -0.13375965567861531f, 0.68708451571155349f, 1.0000000000000000f;
	std::pair<Eigen::Vector3f, Eigen::Vector3f> matchResult;
	matchResult.first[0] = -0.30842339992523193f;
	matchResult.first[1] = -0.15599510073661804f;
	matchResult.first[2] = 0.f;

	matchResult.second[0] = -0.31903117895126343f;
	matchResult.second[1] = -0.11188735067844391f;
	matchResult.second[2] = 0.f;

	double expectedError = 5.3751678625243315e-05f;
	double actualError = symmetricTransferError(vHomographyMatrix, matchResult);
	EXPECT_LE(abs(actualError - expectedError), 1e-7f);
}

TEST(TriangulateTrackError, normalOutput)
{
	std::vector<Eigen::Vector2f> vPostion;
	std::vector<CameraPose> vPoses;
	vPostion.push_back(Eigen::Vector2f(-0.291181982f, -0.192279682f));
	vPostion.push_back(Eigen::Vector2f(-0.205872044f, -0.0936535671f));

	CameraPose pose1, pose2;
	pose2.R << 0.99941037176556757f, 0.034266463962079718f, -0.0021721544479905153f
		, -0.033268140623173625f, 0.98205948907569274f, 0.18561355213417374f
		, 0.0084935049824663711f, -0.18543184560351150f, 0.98262042061479071f;
	pose2.T[0] = 0.89617069378421477f;
	pose2.T[1] = -0.15643556371642153f;
	pose2.T[2] = 0.41521801744028974f;
	vPoses.push_back(pose1);
	vPoses.push_back(pose2);

	Eigen::Vector3f expectedPos = Eigen::Vector3f(-2.8591628206205266f, -2.2887103047447170f, 9.6374448796934935f);
	Eigen::Vector3f actualPos = triangulateTrack(vPostion, vPoses);
	EXPECT_LE(abs((expectedPos-actualPos).norm()), 1e-5f);
}

TEST(PoseFromEssensial, normalOutput)
{
	cv::Matx33f essentialOpenCV(9.0270081951060455e-07f, 0.26131201394831827f, 0.13855922416246347f
		, -0.29535328927602827f, -0.18397842245636264f, 0.61092372228775027f
		, -0.11127784762870724f, -0.63330810807276505f, -0.068885544371089680f);
	cv::Matx33f rotationOpenCV1, rotationOpenCV2;
	cv::Matx31f traslationOpenCV1;
	cv::decomposeEssentialMat(essentialOpenCV, rotationOpenCV1, rotationOpenCV2, traslationOpenCV1);
	//cout << endl << "From OpenCV" << endl;
	//cout << rotationOpenCV1 << endl;
	//cout << rotationOpenCV2 << endl;
	//cout << traslationOpenCV1 << endl;

	Eigen::Matrix3f e;
	e << 9.0270081951060455e-07f, 0.26131201394831827f, 0.13855922416246347f
		, -0.29535328927602827f, -0.18397842245636264f, 0.61092372228775027f
		, -0.11127784762870724f, -0.63330810807276505f, -0.068885544371089680f;
	std::vector<CameraPose> vPoses;
	poseFromEssential(e, vPoses);

	Eigen::Matrix3f finalR;
	Eigen::Vector3f finalT;
	CameraPose pose;
	Eigen::Matrix3f K = Eigen::Matrix3f::Identity();
	for (std::size_t i = 0; i < vPoses.size(); ++i) {
		std::pair<Eigen::Vector2f, Eigen::Vector2f> matchPos 
			= std::make_pair<Eigen::Vector2f, Eigen::Vector2f>(
				Eigen::Vector2f(-0.29118198156356812f, -0.19227968156337738f)
			, Eigen::Vector2f(-0.20587204396724701f, -0.093653567135334015f));
		Eigen::Vector3f x = triangulateMatch(matchPos, pose, vPoses[i]);
		Eigen::Vector3f x1 = pose.R * x + pose.T;
		Eigen::Vector3f x2 = vPoses[i].R * x + vPoses[i].T;

		if (x1[2] > 0.0f && x2[2] > 0.0f) {
			finalR= vPoses[i].R;
			finalT = vPoses[i].T;
			break;
		}
	}

	Eigen::Matrix3f expectR;
	expectR << 0.99941037176556757f, 0.034266463962079718f, -0.0021721544479905153f
		, -0.033268140623173625f, 0.98205948907569274f, 0.18561355213417374f
		, 0.0084935049824663711f, -0.18543184560351150f, 0.98262042061479071f;

	Eigen::Vector3f expectedT = Eigen::Vector3f(0.89617069378421477f, -0.15643556371642153f, 0.41521801744028974f);
	EXPECT_LE((expectedT-finalT).norm(), 1e-5f);
	EXPECT_EQ(expectR.isApprox(finalR, 1e-5f), true);
}


void unitTest(int argc, char* argv[]) {
	testing::InitGoogleTest(&argc, argv);
	RUN_ALL_TESTS();
}