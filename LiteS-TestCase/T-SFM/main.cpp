#include "CSIFT.h"
#include <glm/matrix.hpp>
#include <Eigen/Core>
#include <Eigen/SVD>  
#include <Eigen/Dense>  
#include <iostream>
#include <random>
#include <utility>
#include "CImage.h"
#include "util.h"
#include "ceres/ceres.h"
#include "ceres/rotation.h"

const float RANSAC_INLIER_THRESHOLD = 0.0015f;


struct CameraPose {
	glm::mat3 K = glm::mat3(1.0f, 0.f, 0.f, 0.f, 1.0f, 0.f, 0.f, 0.f, 1.f);
	glm::vec3 T = glm::vec3(0.0f, 0.f, 0.f);
	glm::mat3 R = glm::mat3(1.0f, 0.f, 0.f, 0.f, 1.0f, 0.f, 0.f, 0.f, 1.f);
	bool valied = true;
};

struct Viewport {
	size_t ID;

	std::vector<cv::KeyPoint>* keyPoints;
	cv::Mat* descriptor;

	std::vector<int> trackIDs;

	CameraPose pose;
};

struct PairWiseCamera {
	size_t ID1;
	size_t ID2;
	
	std::vector<std::pair<glm::vec3, glm::vec3>> matchResult;
	glm::mat3 foundamentalMatrix;
};

struct Track {
	bool valied = true;
	glm::vec3 pos;
	std::vector<std::pair<size_t, size_t>> tracks;
};

struct Observation {
	glm::vec2 pos;
	int camera_id;
	int point_id;
};

void getViewportFeatures(const std::vector<std::string> &vImageLists
	,std::vector<Viewport> &vOutViewports) {
	for (size_t i = 0; i < vImageLists.size();++i) {
		vOutViewports.push_back(Viewport());
		vOutViewports.back().ID = i;
		vOutViewports.back().keyPoints = new std::vector<cv::KeyPoint>;
		vOutViewports.back().descriptor = new cv::Mat;
		vOutViewports.back().keyPoints->clear();
		computeFeaturesOpenCV(vImageLists[i], vOutViewports.back().keyPoints
			, vOutViewports.back().descriptor);
	}
}

void matchViewportFeature(const std::vector<Viewport> &vViewports
	, std::vector<PairWiseCamera> &vOutPairs) {
	for (size_t view1 = 0; view1 < vViewports.size(); view1++){
		for (size_t view2 = 0; view2 < view1; view2++) {
			if(view1==view2)
				continue;
			std::vector<cv::DMatch> goodMatches;
			matchFeatureOpenCV(vViewports[view1].descriptor, vViewports[view2].descriptor, goodMatches);
			vOutPairs.push_back(PairWiseCamera());
			PairWiseCamera& p = vOutPairs.back();
			p.ID1 = view1;
			p.ID2 = view2;
			for (auto &matchItem : goodMatches) {
				float x1 = vViewports[view1].keyPoints->at(matchItem.queryIdx).pt.x;
				float y1 = vViewports[view1].keyPoints->at(matchItem.queryIdx).pt.y;
				float x2 = vViewports[view2].keyPoints->at(matchItem.trainIdx).pt.x;
				float y2 = vViewports[view2].keyPoints->at(matchItem.trainIdx).pt.y;
			
				p.matchResult.push_back(std::make_pair < glm::vec3, glm::vec3 >(
					glm::vec3(x1, y1, matchItem.queryIdx), glm::vec3(x2, y2, matchItem.trainIdx)));
			}
		}
	}
}

void normalizeImageCoordinate(std::vector<Viewport> &vViewports,size_t vWidth, size_t vHeight) {
	for (auto& viewport : vViewports) {
		for (size_t i = 0; i < viewport.keyPoints->size();++i) {
			viewport.keyPoints->at(i).pt.x /= static_cast<float>(vWidth);
			viewport.keyPoints->at(i).pt.y /= static_cast<float>(vHeight);
		}
	}
}

Eigen::VectorXf estimate8Points(PairWiseCamera& pairCameras) {
	if (pairCameras.matchResult.size() < 8) {
		std::cout << "Not have enough point to estimate 8 point algorithm" << std::endl;
		return Eigen::VectorXf();
	}

	std::uniform_int_distribution<size_t> gen(0, pairCameras.matchResult.size()-1);
	std::mt19937 myRand(time(0));

	std::set<size_t> result;
	while (result.size() < 8)
		result.insert(gen(myRand));

	Eigen::Matrix<float, 3, 8> pset1, pset2;
	std::set<size_t>::const_iterator iter = result.begin();
	for (int i = 0; i < 8; ++i, iter++) {
		pset1(0, i) = pairCameras.matchResult[*iter].first.x;
		pset1(1, i) = pairCameras.matchResult[*iter].first.y;
		pset1(2, i) = 1.f;
		pset2(0, i) = pairCameras.matchResult[*iter].second.x;
		pset2(1, i) = pairCameras.matchResult[*iter].second.y;
		pset2(2, i) = 1.f;
	}

	Eigen::Matrix<float, 8, 9> A;
	for (int i = 0; i < 8; ++i){
		Eigen::Vector3f p1 = pset1.col(i);
		Eigen::Vector3f p2 = pset2.col(i);
		A(i, 0) = p2(0) * p1(0);
		A(i, 1) = p2(0) * p1(1);
		A(i, 2) = p2(0) * 1.0f;
		A(i, 3) = p2(1) * p1(0);
		A(i, 4) = p2(1) * p1(1);
		A(i, 5) = p2(1) * 1.0f;
		A(i, 6) = 1.0f   * p1(0);
		A(i, 7) = 1.0f   * p1(1);
		A(i, 8) = 1.0f   * 1.0f;
	}

	Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeFullV | Eigen::ComputeFullU);
	Eigen::Matrix<float, 8, 8> U = svd.matrixU();
	Eigen::Matrix<float, 9, 9> V = svd.matrixV();

	Eigen::VectorXf solution = V.transpose().col(8);

	return solution;
}

Eigen::Matrix<float, 3, 3> enforceConstrains(Eigen::VectorXf vSolution) {
	Eigen::Matrix<float, 3, 3> fundamentalMatrix;
	for (size_t y = 0; y < 3; y++)
		for (size_t x = 0; x < 3; x++)
			fundamentalMatrix(y, x) = vSolution(y * 3 + x);

	Eigen::JacobiSVD<Eigen::MatrixXf> svd(fundamentalMatrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::Matrix3f U = svd.matrixU();
	Eigen::Matrix3f V = svd.matrixV();
	Eigen::Matrix3f S = svd.singularValues().asDiagonal();

	S(2, 2) = 0;

	Eigen::Matrix<float, 3, 3> outPut;
	outPut = U * S*V.transpose();
	return outPut;
}

double sampsonDistance(Eigen::Matrix<float, 3, 3> &vFoundamentalMatrix
	, std::pair<glm::vec2, glm::vec2> vCoordinates) {
	double p2_F_p1 = 0.0;
	p2_F_p1 += vCoordinates.second[0] * (vCoordinates.first[0] * vFoundamentalMatrix(0,0) 
		+ vCoordinates.first[1] * vFoundamentalMatrix(0,1) + vFoundamentalMatrix(0,2));
	p2_F_p1 += vCoordinates.second[1] * (vCoordinates.first[0] * vFoundamentalMatrix(1,0) 
		+ vCoordinates.first[1] * vFoundamentalMatrix(1,1) + vFoundamentalMatrix(1,2));
	p2_F_p1 += 1.0 * (vCoordinates.first[0] * vFoundamentalMatrix(2,0) 
		+ vCoordinates.first[1] * vFoundamentalMatrix(2,1) + vFoundamentalMatrix(2,2));
	p2_F_p1 *= p2_F_p1;

	double sum = 0.0;
	sum += pow(vCoordinates.first[0] * vFoundamentalMatrix(0,0) 
		+ vCoordinates.first[1] * vFoundamentalMatrix(0,1) + vFoundamentalMatrix(0,2), 2);
	sum += pow(vCoordinates.first[0] * vFoundamentalMatrix(1,0) 
		+ vCoordinates.first[1] * vFoundamentalMatrix(1,1) + vFoundamentalMatrix(1,2), 2);
	sum += pow(vCoordinates.second[0] * vFoundamentalMatrix(0,0) 
		+ vCoordinates.second[1] * vFoundamentalMatrix(1,0) + vFoundamentalMatrix(2,0), 2);
	sum += pow(vCoordinates.second[0] * vFoundamentalMatrix(0,1) 
		+ vCoordinates.second[1] * vFoundamentalMatrix(1,1) + vFoundamentalMatrix(2,1), 2);

	return p2_F_p1 / sum;
}

std::vector<size_t> findInlier(Eigen::Matrix<float, 3, 3> &vFoundamentalMatrix , PairWiseCamera& vPairCameras) {
	std::vector<size_t> result;
	result.resize(0);
	double const squared_thres = RANSAC_INLIER_THRESHOLD * RANSAC_INLIER_THRESHOLD;
	for (std::size_t i = 0; i < vPairCameras.matchResult.size(); ++i)
	{
		double error = sampsonDistance(vFoundamentalMatrix, vPairCameras.matchResult[i]);
		if (error < squared_thres)
			result.push_back(i);
	}
	return result;
}

void ransac(std::vector<PairWiseCamera>& pairCameras) {
	for (auto& pairItem : pairCameras) {
		std::vector<size_t> inliers;
		inliers.reserve(pairItem.matchResult.size());
		Eigen::Matrix<float, 3, 3> bestFoundamentalMatrix;
		for (size_t iter = 0; iter < 10000; iter++)
		{
			Eigen::VectorXf solution = estimate8Points(pairItem);
			Eigen::Matrix<float, 3, 3> foundamentalMatrix = enforceConstrains(solution);
			std::vector<size_t> localInlier = findInlier(foundamentalMatrix, pairItem);
			if (inliers.size() < localInlier.size()) {
				bestFoundamentalMatrix = foundamentalMatrix;
				inliers = localInlier;
			}
		}
		std::vector<std::pair<glm::vec3, glm::vec3>> newMatchResult;
		for (auto& inlierIndex : inliers)
			newMatchResult.push_back(pairItem.matchResult[inlierIndex]);
		pairItem.matchResult = newMatchResult;
		for (size_t y = 0; y < 3; y++)
			for (size_t x = 0; x < 3; x++)
				pairItem.foundamentalMatrix[y][x] = bestFoundamentalMatrix(y,x);
	}
	
}

void computeTracks(const std::vector<PairWiseCamera>& vPairCameras
	, std::vector<Viewport>& viewports , std::vector<Track>& vTracks) {
	for (std::size_t i = 0; i < viewports.size(); ++i){
		Viewport& viewport = viewports[i];
		viewport.trackIDs.resize(viewport.keyPoints->size(), -1);
	}

	for (size_t pairIndex = 0; pairIndex < vPairCameras.size(); pairIndex++){
		const PairWiseCamera& pairCamera = vPairCameras[pairIndex];
		Viewport& viewport1 = viewports[pairCamera.ID1];
		Viewport& viewport2 = viewports[pairCamera.ID2];

		for (size_t matchIndex = 0; matchIndex < pairCamera.matchResult.size(); ++matchIndex) {
			size_t featureIndex1 = pairCamera.matchResult[matchIndex].first.z;
			size_t featureIndex2 = pairCamera.matchResult[matchIndex].second.z;

			if (-1 == viewport1.trackIDs[featureIndex1] && -1 == viewport2.trackIDs[featureIndex2]) {
				vTracks.push_back(Track());
				Track& track = vTracks.back();
				track.tracks.push_back(std::make_pair(pairCamera.ID1, featureIndex1));
				track.tracks.push_back(std::make_pair(pairCamera.ID2, featureIndex2));
				viewport1.trackIDs[featureIndex1] = vTracks.size()-1;
				viewport2.trackIDs[featureIndex2] = vTracks.size()-1;
			}
			else if (-1 != viewport1.trackIDs[featureIndex1] && -1 == viewport2.trackIDs[featureIndex2]) {
				Track& track = vTracks[viewport1.trackIDs[featureIndex1]];
				viewport2.trackIDs[featureIndex2] = viewport1.trackIDs[featureIndex1];
				track.tracks.push_back(std::make_pair(pairCamera.ID2, featureIndex2));
			}
			else if (-1 == viewport1.trackIDs[featureIndex1] && -1 != viewport2.trackIDs[featureIndex2]) {
				Track& track = vTracks[viewport2.trackIDs[featureIndex2]];
				viewport1.trackIDs[featureIndex1] = viewport2.trackIDs[featureIndex2];
				track.tracks.push_back(std::make_pair(pairCamera.ID1, featureIndex1));
			}
			else if (-1 != viewport1.trackIDs[featureIndex1] && -1 != viewport2.trackIDs[featureIndex2]) {
				Track& track1 = vTracks[viewport1.trackIDs[featureIndex1]];
				Track& track2 = vTracks[viewport2.trackIDs[featureIndex2]];
				int eraseTrackID = viewport2.trackIDs[featureIndex2];
				for (size_t i = 0; i < track2.tracks.size(); ++i) {
					viewports[track2.tracks[i].first].trackIDs[track2.tracks[i].second] 
						= viewport1.trackIDs[featureIndex1];
				}

				track1.tracks.insert(track1.tracks.begin(), track2.tracks.begin(), track2.tracks.end());
				track2.tracks.clear();
			}
			else
				throw "lalala";
		}
	}

	std::vector<size_t> deleteSign(vTracks.size());
	int num_invalid_tracks = 0;
	for (std::size_t i = 0; i < vTracks.size(); ++i){
		if (vTracks.at(i).tracks.empty()){
			deleteSign[i] = true;
			continue;
		}

		std::set<int> view_ids;
		for (std::size_t j = 0; j < vTracks.at(i).tracks.size(); ++j){
			if (view_ids.insert(vTracks.at(i).tracks[j].first).second == false){
				num_invalid_tracks += 1;
				deleteSign[i] = true;
				break;
			}
		}
	}

	/* Create a mapping from old to new track IDs. */
	std::vector<int> id_mapping(deleteSign.size(), -1);
	int valid_track_counter = 0;
	for (std::size_t i = 0; i < deleteSign.size(); ++i)
	{
		if (deleteSign[i])
			continue;
		id_mapping[i] = valid_track_counter;
		valid_track_counter += 1;
	}

	/* Fix track IDs stored in the viewports. */
	for (std::size_t i = 0; i < viewports.size(); ++i){
		std::vector<int>& track_ids = viewports.at(i).trackIDs;
		for (std::size_t j = 0; j < track_ids.size(); ++j)
			if (track_ids[j] >= 0)
				track_ids[j] = id_mapping[track_ids[j]];
	}

	/* Clean the tracks from the vector. */
	std::vector<Track> t_tracks;
	for (size_t i = 0; i < vTracks.size(); i++)
	{
		if (!deleteSign[i])
			t_tracks.push_back(vTracks[i]);
	}
	std::swap(vTracks, t_tracks);

	return;
}

size_t computeHomographyInliers(const PairWiseCamera& vPairCamera) {
	std::vector<int> inliers;
	inliers.reserve(vPairCamera.matchResult.size());

	size_t bestInliersNum = 0;

	for (int iteration = 0; iteration < 1000; ++iteration) {
		if (vPairCamera.matchResult.size() < 4)
			throw std::invalid_argument("At least 4 matches required");

		std::set<int> result;
		std::uniform_int_distribution<int> gen(0, vPairCamera.matchResult.size()-1);
		std::mt19937 rd(time(0));
		while (result.size() < 4)
			result.insert(gen(rd));

		std::vector<std::pair<glm::vec3, glm::vec3>> fourFeatures(4);
		std::set<int>::const_iterator iter = result.begin();
		for (std::size_t i = 0; i < 4; ++i, ++iter)
			fourFeatures[i] = vPairCamera.matchResult[*iter];

		// Compute homography
		Eigen::Matrix<float,9,9> A;
		for (std::size_t featurePairID = 0; featurePairID < fourFeatures.size(); ++featurePairID)
		{
			std::size_t const row1 = 2 * featurePairID;
			std::size_t const row2 = 2 * featurePairID + 1;
			std::pair<glm::vec3, glm::vec3> const& match = fourFeatures[featurePairID];
			A(row2 , 0) = 0.0;
			A(row2 , 1) = 0.0;
			A(row2 , 2) = 0.0;
			A(row2 , 3) = match.first.x;
			A(row2 , 4) = match.first.y;
			A(row2 , 5) = 1.0;
			A(row2 , 6) = -match.second.y * match.first.x;
			A(row2 , 7) = -match.second.y * match.first.y;
			A(row2 , 8) = -match.second.y;

			A(row1 , 0) = match.first.x;
			A(row1 , 1) = match.first.y;
			A(row1 , 2) = 1.0;
			A(row1 , 3) = 0.0;
			A(row1 , 4) = 0.0;
			A(row1 , 5) = 0.0;
			A(row1 , 6) = -match.second.x * match.first.x;
			A(row1 , 7) = -match.second.x * match.first.y;
			A(row1 , 8) = -match.second.x;
		}

		/* Compute homography matrix using SVD. */
		Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
		Eigen::VectorXf V = svd.matrixV().col(9);
		glm::mat3 homographyMatrix;
		for (size_t y = 0; y < 3; y++)
			for (size_t x = 0; x < 3; x++)
				homographyMatrix[y][x] = V(y * 3 + x);

		//???????????????????????????????????????????????
		//*homography /= (*homography)[8];

		float const square_threshold = 0.05 * 0.05;
		inliers.resize(0);
		for (std::size_t i = 0; i < vPairCamera.matchResult.size(); ++i){
			const glm::vec3 point1 = vPairCamera.matchResult[i].first;
			const glm::vec3 point2 = vPairCamera.matchResult[i].second;
			/*
			 * Computes the symmetric transfer error for a given match and homography
			 * matrix. The error is computed as [Sect 4.2.2, Hartley, Zisserman]:
			 *
			 *   e = d(x, (H^-1)x')^2 + d(x', Hx)^2
			 */
			glm::vec3 p1(point1[0], point1[1], 1.0);
			glm::vec3 p2(point2[0], point2[1], 1.0);

			glm::mat3 invH = glm::inverse(homographyMatrix);
			glm::vec3 result = invH * p2;
			result /= result[2];
			float error = glm::distance(p1, result);

			result = homographyMatrix * p1;
			result /= result[2];
			error += glm::distance(result, p2);

			error = 0.5 * error;
			if (error < square_threshold)
				inliers.push_back(i);
		}

		if (inliers.size() > bestInliersNum){
			bestInliersNum = inliers.size();
		}
	}

	return bestInliersNum;
}

void poseFromEssential(const glm::mat3 vEssensialMatrix
	, std::vector<CameraPose> vPoses) {
	glm::mat3 W(0.0);
	W[0][1] = -1.0; W[1][0] = 1.0; W[2][2] = 1.0;
	glm::mat3 Wt(0.0);
	Wt[0][1] = 1.0; Wt[1][0] = -1.0; Wt[2][2] = 1.0;

	Eigen::Matrix<float, 3, 3> E;
	E<< vEssensialMatrix[0][0], vEssensialMatrix[0][1], vEssensialMatrix[0][2]
		, vEssensialMatrix[1][0], vEssensialMatrix[1][1], vEssensialMatrix[1][2]
		, vEssensialMatrix[2][0], vEssensialMatrix[2][1], vEssensialMatrix[2][2];

	Eigen::Matrix<float, 3, 3> U, S, V;
	Eigen::JacobiSVD<Eigen::MatrixXf> svd(E, Eigen::ComputeFullU || Eigen::ComputeFullV);
	U = svd.matrixU();
	V = svd.matrixV();

	if (U.determinant() < 0.0)
		for (int i = 0; i < 3; ++i)
			U(i, 2) = -U(i, 2);
	if (V.determinant() < 0.0)
		for (int i = 0; i < 3; ++i)
			V(i, 2) = -V(i, 2);

	V=V.transpose();
	glm::mat3 uGlm, vGlm, sGlm;
	for (int y = 0; y < 3; ++y)
		for (int x = 0; x < 3; ++x) {
			uGlm[y][x] = U(y, x);
			vGlm[y][x] = V(y, x);
		}
	vPoses.clear();
	vPoses.resize(4);
	vPoses.at(0).R = uGlm * W * vGlm;
	vPoses.at(1).R = vPoses.at(0).R;
	vPoses.at(2).R = uGlm * Wt * vGlm;
	vPoses.at(3).R = vPoses.at(2).R;
	vPoses.at(0).T = glm::transpose(uGlm)[2];
	vPoses.at(1).T = -vPoses.at(0).T;
	vPoses.at(2).T = vPoses.at(0).T;
	vPoses.at(3).T = -vPoses.at(0).T;
}

glm::vec3 triangulateMatch(const std::pair<glm::vec2, glm::vec2> vPairPos
	,CameraPose vPose1, CameraPose vPose2){
	/* The algorithm is described in HZ 12.2, page 312. */
	Eigen::Matrix<float, 3, 4> P1, P2;
	Eigen::Matrix<float, 3, 3> KR1=eigenFromGLM<3,3>(vPose1.K * vPose1.R);
	Eigen::Matrix<float, 3, 1> Kt1 = eigenFromGLM<3>(vPose1.K * vPose1.T);
	P1.block(0, 0, 3, 3) = KR1;
	P1.block(0, 3, 3, 1) = Kt1;
	Eigen::Matrix<float, 3, 3> KR2 = eigenFromGLM<3, 3>(vPose2.K * vPose2.R);
	Eigen::Matrix<float, 3, 1> Kt2 = eigenFromGLM<3>(vPose2.K * vPose2.T);
	P2.block(0, 0, 3, 3) = KR2;
	P2.block(0, 3, 3, 1) = Kt2;
	
	Eigen::Matrix<float, 4, 4> A;
	for (int i = 0; i < 4; ++i){
		A(0, i) = vPairPos.first[0] * P1(2, i) - P1(0, i);
		A(1, i) = vPairPos.first[1] * P1(2, i) - P1(1, i);
		A(2, i) = vPairPos.second[0] * P2(2, i) - P2(0, i);
		A(3, i) = vPairPos.second[1] * P2(2, i) - P2(1, i);
	}


	Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeFullU || Eigen::ComputeFullU);
	Eigen::Matrix<float, 4, 4> V = svd.matrixV();
	Eigen::VectorXf x = V.col(3);
	return glm::vec3(x[0] / x[3], x[1] / x[3], x[2] / x[3]);
}

bool computePose(const std::vector<Viewport>& viewports, const PairWiseCamera& vPairCamera) {
	// Compute fundamental matrix from pair correspondences.
	const glm::mat3 fundamental=vPairCamera.foundamentalMatrix;

	/* Populate K-matrices. */
	Viewport view_1 = viewports[vPairCamera.ID1];
	Viewport view_2 = viewports[vPairCamera.ID2];

	/* Compute essential matrix from fundamental matrix (HZ (9.12)). */
	glm::mat3 essensialMatrix = glm::transpose(view_2.pose.K) * fundamental * view_1.pose.K;

	/* Compute pose from essential. */
	std::vector<CameraPose> poses;
	poseFromEssential(essensialMatrix, poses);

	/* Find the correct pose using point test (HZ Fig. 9.12). */
	bool found_pose = false;
	for (std::size_t i = 0; i < poses.size(); ++i){
		poses[i].K = viewports[vPairCamera.ID2].pose.K;
		std::pair<glm::vec2, glm::vec2> matchPos = std::make_pair<glm::vec2, glm::vec2>(
			glm::vec2(vPairCamera.matchResult[0].first.x, vPairCamera.matchResult[0].first.y)
			, glm::vec2(vPairCamera.matchResult[0].second.x, vPairCamera.matchResult[0].second.y));
		glm::vec3 x = triangulateMatch(matchPos, view_1.pose, poses[i]);
		glm::vec3 x1 = view_1.pose.R * x + view_1.pose.T;
		glm::vec3 x2 = poses[i].R * x + poses[i].T;

		if (x1[2] > 0.0f && x2[2] > 0.0f){
			view_2.pose = poses[i];
			found_pose = true;
			break;
		}
	}
	return found_pose;
}

float angleForPose(const std::vector<Viewport>& viewports, const PairWiseCamera& vPairCamera) {
	const CameraPose& pose1 = viewports[vPairCamera.ID1].pose;
	const CameraPose& pose2 = viewports[vPairCamera.ID2].pose;
	// Compute transformation from image coordinates to viewing direction
	glm::mat3 T1 = glm::transpose(pose1.R) * glm::inverse(pose1.K);
	glm::mat3 T2 = glm::transpose(pose2.R) * glm::inverse(pose2.K);

	/* Compute triangulation angle for each correspondence. */
	std::vector<float> cos_angles;
	cos_angles.reserve(vPairCamera.matchResult.size());
	for (std::size_t i = 0; i < vPairCamera.matchResult.size(); ++i){
		glm::vec3 p1(vPairCamera.matchResult[i].first.x, vPairCamera.matchResult[i].first.y, 1.f);
		glm::vec3 p2(vPairCamera.matchResult[i].second.x, vPairCamera.matchResult[i].second.y, 1.f);
		p1 = glm::normalize(T1*p1);
		p2 = glm::normalize(T2*p2);
		cos_angles.push_back(glm::dot(p1,p2));
	}

	/* Return 50% median. */
	std::size_t median_index = cos_angles.size() / 2;
	std::nth_element(cos_angles.begin(),
		cos_angles.begin() + median_index, cos_angles.end());
	float const cos_angle = std::clamp(cos_angles[median_index], -1.f, 1.f);
	return std::acos(cos_angle);
}

float scoreForPair(const PairWiseCamera& vPairCamera,const size_t vNumInliers
	,const float vAngle) {
	float const matchesScore = static_cast<float>(vPairCamera.matchResult.size());
	float const inlierScore = static_cast<float>(vNumInliers) / matchesScore;
	float const angle_d = vAngle*180/3.1415926;

	/* Score for matches (min: 20, good: 200). */
	float f1 = 2.0 / (1.0 + std::exp((20.0 - matchesScore) * 6.0 / 200.0)) - 1.0;
	/* Score for angle (min 1 degree, good 8 degree). */
	float f2 = 2.0 / (1.0 + std::exp((1.0 - angle_d) * 6.0 / 8.0)) - 1.0;
	/* Score for H-Inlier (max 70%, good 40%). */
	float f3 = 2.0 / (1.0 + std::exp((inlierScore - 0.7) * 6.0 / 0.4)) - 1.0;

	f1 = std::clamp(f1, 0.0f, 1.0f);
	f2 = std::clamp(f2, 0.0f, 1.0f);
	f3 = std::clamp(f3, 0.0f, 1.0f);
	return f1 * f2 * f3;
}

glm::vec3 triangulateTrack(std::vector<glm::vec2>& vPostion, std::vector<CameraPose>& vPoses) {
	if (vPostion.size() != vPoses.size() || vPostion.size() < 2)
		throw std::invalid_argument("Invalid number of positions/poses");

	Eigen::Matrix<float, Eigen::Dynamic,4> A;
	A.resize(2 * vPoses.size(), 4);
	for (std::size_t i = 0; i < vPoses.size(); ++i){
		CameraPose const& pose = vPoses[i];
		glm::vec2 p= vPostion[i];
		Eigen::Matrix<float, 3, 4> P1;
		Eigen::Matrix<float, 3, 3> KR1=eigenFromGLM<3,3>(pose.K * pose.R);
		Eigen::Matrix<float, 3, 1> Kt1=eigenFromGLM<3>(pose.K * pose.T);
		P1.block(0, 0, 3, 3) = KR1;
		P1.block(2, 2, 1, 1) = Kt1;

		for (int j = 0; j < 4; ++j){
			A(2 * i + 0, j) = p[0] * P1(2, j) - P1(0, j);
			A(2 * i + 1, j) = p[1] * P1(2, j) - P1(1, j);
		}
	}

	/* Compute SVD. */
	Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeFullU || Eigen::ComputeFullU);
	Eigen::Matrix<float, 4, 4> V = svd.matrixV();
	Eigen::VectorXf x = V.col(3);
	return glm::vec3(x[0] / x[3], x[1] / x[3], x[2] / x[3]);
}

bool triangulate(std::vector<CameraPose>& vPoses,std::vector<glm::vec2>& vPositions
	,glm::vec3& vPos3D, std::vector<std::size_t>* vOutliers=nullptr) {
	if (vPoses.size() < 2)
		throw std::invalid_argument("At least two poses required");
	if (vPoses.size() != vPositions.size())
		throw std::invalid_argument("Poses and positions size mismatch");

	// Check all possible pose pairs for successful triangulation
	std::vector<std::size_t> bestOutliers(vPositions.size());
	glm::vec3 bestPos(0.0f);
	for (std::size_t p1 = 0; p1 < vPoses.size(); ++p1)
		for (std::size_t p2 = p1 + 1; p2 < vPoses.size(); ++p2){
			/* Triangulate position from current pair */
			std::vector<CameraPose> pose_pair;
			std::vector<glm::vec2> position_pair;
			pose_pair.push_back(vPoses[p1]);
			pose_pair.push_back(vPoses[p2]);
			position_pair.push_back(vPositions[p1]);
			position_pair.push_back(vPositions[p2]);
			glm::vec3 tmp_pos = triangulateTrack(position_pair, pose_pair);
			assert(!(std::isnan(tmp_pos[0]) || std::isnan(tmp_pos[0]) ||
				std::isnan(tmp_pos[1]) || std::isnan(tmp_pos[1]) ||
				std::isnan(tmp_pos[2]) || std::isnan(tmp_pos[2])));

			// Check if pair has small triangulation angle
			glm::vec3 camera_pos;
			camera_pos = -glm::inverse(pose_pair[0].R)*pose_pair[0].T;
			glm::vec3 ray0 = glm::normalize(tmp_pos - camera_pos);
			camera_pos = -glm::inverse(pose_pair[1].R)*pose_pair[1].T;
			glm::vec3 ray1 = glm::normalize(tmp_pos - camera_pos);
			float const cos_angle = glm::dot(ray0,ray1);
			if (cos_angle > std::cos(1*3.1415926/180))
				continue;

			// Check error in all input poses and find outliers.
			std::vector<std::size_t> tmp_outliers;
			for (std::size_t i = 0; i < vPoses.size(); ++i){

				glm::vec3 x = vPoses[i].R * tmp_pos + vPoses[i].T;

				/* Reject track if it appears behind the camera. */
				if (x[2] <= 0.0){
					tmp_outliers.push_back(i);
					continue;
				}


				x = vPoses[i].K * x;
				glm::vec2 x2d(x[0] / x[2], x[1] / x[2]);
				float error = glm::distance(vPositions[i], x2d);
				if (error > 0.01f)
					tmp_outliers.push_back(i);
			}

			// Select triangulation with lowest amount of outliers.
			if (tmp_outliers.size() < bestOutliers.size()){
				bestPos = tmp_pos;
				std::swap(bestOutliers, tmp_outliers);
			}

		}

	// If all pairs have small angles pos will be 0 here.
	if (glm::length(bestPos) == 0.0f){
		return false;
	}

	// Check if required number of inliers is found.
	if (vPoses.size() < bestOutliers.size() + 2){
		return false;
	}

	// Return final position and outliers.
	vPos3D = bestPos;

	if (0!=(*vOutliers).size())
		std::swap(*vOutliers, bestOutliers);

	return true;
}

size_t findInitPairs(const std::vector<Viewport>& vViewports, const std::vector<PairWiseCamera>& vPairCameras) {
	int viewID1 = -1, viewID2 = -1;
	std::vector<PairWiseCamera> candidates(vPairCameras);
	
	std::sort(candidates.begin(), candidates.end()
		, [](const PairWiseCamera& v1,const PairWiseCamera& v2)->bool {
		return v1.matchResult.size() > v2.matchResult.size();
	});

	bool found_pair = false;
	std::size_t found_pair_id = std::numeric_limits<std::size_t>::max();
	std::vector<float> pairScores(candidates.size(), 0.0f);

	for (std::size_t candidateIndex = 0; candidateIndex < candidates.size(); ++candidateIndex) {
		if (found_pair)
			continue;

		// Reject pairs with 50 or fewer matches.
		if (candidates[candidateIndex].matchResult.size() < 50){
			continue;
		}


		// Reject pairs with too high percentage of homography inliers.
		std::size_t numInliers = computeHomographyInliers(candidates[candidateIndex]);
		float percentage = static_cast<float>(numInliers) / candidates[candidateIndex].matchResult.size();
		if (percentage > 0.8f){
			continue;
		}

		// Compute initial pair pose.

		bool const found_pose = computePose(vViewports, candidates[candidateIndex]);
		if (!found_pose){
			continue;
		}

		// Rejects pairs with bad triangulation angle.
		float const angle = angleForPose(vViewports,vPairCameras[candidateIndex]);
		pairScores[candidateIndex] = scoreForPair(vPairCameras[candidateIndex], numInliers, angle);
		if (angle < 5 * 3.1415926 / 180)
			continue;

		// If all passes, run triangulation to ensure correct pair
		std::vector<CameraPose> poses;
		poses.push_back(vViewports[vPairCameras[candidateIndex].ID1].pose);
		poses.push_back(vViewports[vPairCameras[candidateIndex].ID2].pose);
		std::size_t successful_triangulations = 0;
		std::vector<glm::vec2> positions(2);
		for (std::size_t j = 0; j < vPairCameras[candidateIndex].matchResult.size(); ++j)
		{
			positions[0] = glm::vec2(vPairCameras[candidateIndex].matchResult[j].first.x
				, vPairCameras[candidateIndex].matchResult[j].first.y);
			positions[1] = glm::vec2(vPairCameras[candidateIndex].matchResult[j].second.x
				, vPairCameras[candidateIndex].matchResult[j].second.y);
			glm::vec3 pos3d;
			if (triangulate(poses, positions, pos3d))
				successful_triangulations += 1;
		}
		if (successful_triangulations * 2 < vPairCameras[candidateIndex].matchResult.size())
			continue;

		found_pair = true;
		viewID1 = candidates[candidateIndex].ID1;
		viewID2 = candidates[candidateIndex].ID2;
	}

	if (!found_pair)
		throw "lalala";
	return found_pair_id;
}

void triangulateNewTracks(std::vector<Viewport>& vViewports,std::vector<Track>& vTracks
	,const size_t vMinNumViews) {
	std::size_t initial_tracks_size = vTracks.size();
	for (std::size_t i = 0; i < vTracks.size(); ++i){
		/* Skip tracks that have already been triangulated. */
		Track const& track = vTracks[i];
		if (!track.valied)
			continue;

		/*
		 * Triangulate the track using all cameras. There can be more than two
		 * cameras if the track was rejected in previous triangulation attempts.
		 */
		std::vector<glm::vec2> pos;
		std::vector<CameraPose> poses;
		std::vector<std::size_t> view_ids;
		std::vector<std::size_t> feature_ids;
		for (std::size_t trackFeaturesIndex = 0; trackFeaturesIndex < track.tracks.size(); ++trackFeaturesIndex){
			int const view_id = track.tracks[trackFeaturesIndex].first;
			if (!vViewports.at(view_id).pose.valied)
				continue;
			Viewport const& viewport = vViewports.at(view_id);
			int const feature_id = track.tracks[trackFeaturesIndex].second;
			// ?????????????????????????????????????????????????????????????
			//pos.push_back(undistortFeature(
			//	viewport.features.positions[feature_id],
			//	static_cast<double>(viewport.radial_distortion[0]),
			//	static_cast<double>(viewport.radial_distortion[1]),
			//	viewport.focal_length));
			pos.push_back(glm::vec2(viewport.keyPoints->at(feature_id).pt.x
				, viewport.keyPoints->at(feature_id).pt.y));
			poses.push_back(vViewports.at(view_id).pose);
			view_ids.push_back(view_id);
			feature_ids.push_back(feature_id);
		}

		/* Skip tracks with too few valid cameras. */
		if ((int)poses.size() < vMinNumViews)
			continue;

		/* Accept track if triangulation was successful. */
		std::vector<std::size_t> outlier;
		glm::vec3 track_pos;
		if (!triangulate(poses, pos, track_pos,  &outlier))
			continue;
		vTracks.at(i).pos = track_pos;

		/* Check if track contains outliers */
		if (outlier.size() == 0)
			continue;

		/* Split outliers from track and generate new track */
		Track & inlier_track = vTracks.at(i);
		Track outlier_track;
		outlier_track.valied=false;
		//outlier_track.color = inlier_track.color;
		for (std::size_t i = 0; i < outlier.size(); ++i){
			size_t viewID = view_ids[outlier[i]];
			size_t featureID = feature_ids[outlier[i]];
			/* Remove outlier from inlier track */
			std::vector<std::pair<size_t, size_t>>::iterator iter = inlier_track.tracks.begin();
			while (iter!= inlier_track.tracks.end()){
				if ((*iter).first == viewID)
					inlier_track.tracks.erase(iter++);
			}
			/* Add features to new track */
			outlier_track.tracks.push_back(std::make_pair(viewID, featureID));
			/* Change TrackID in viewports */
			vViewports.at(viewID).trackIDs[featureID] = vTracks.size();
		}
		vTracks.push_back(outlier_track);
	}

}

void invalidateLargeErrorTracks(std::vector<Viewport>& vViewports,std::vector<Track>& vTracks) {
	/* Iterate over all tracks and sum reprojection error. */
	std::vector<std::pair<float, std::size_t> > all_errors;
	std::size_t num_valid_tracks = 0;
	for (std::size_t i = 0; i < vTracks.size(); ++i){
		if (!vTracks.at(i).valied)
			continue;

		num_valid_tracks += 1;
		glm::vec3 const& pos3d = vTracks.at(i).pos;
		std::vector<std::pair<size_t, size_t>> const& ref = vTracks.at(i).tracks;

		float total_error = 0.0f;
		int num_valid = 0;
		for (std::size_t j = 0; j < ref.size(); ++j){
			/* Get pose and 2D position of feature. */
			int view_id = ref[j].first;
			int feature_id = ref[j].second;

			Viewport const& viewport = vViewports.at(view_id);
			CameraPose const& pose = viewport.pose;
			if (!pose.valied)
				continue;

			glm::vec2 const& pos2d = glm::vec2(viewport.keyPoints->at(feature_id).pt.x
				, viewport.keyPoints->at(feature_id).pt.y);

			/* Project 3D feature and compute reprojection error. */
			glm::vec3 x = pose.R * pos3d + pose.T;
			glm::vec2 x2d(x[0] / x[2], x[1] / x[2]);
			float r2 = x2d.length();
			//x2d *= (1.0 + r2 * (viewport.radial_distortion[0]
			//	+ viewport.radial_distortion[1] * r2))
			//	* pose.get_focal_length();
			total_error += (pos2d - x2d).length();
			num_valid += 1;
		}
		total_error /= static_cast<double>(num_valid);
		all_errors.push_back(std::pair<double, int>(total_error, i));
	}

	if (num_valid_tracks < 2)
		return;

	/* Find the 1/2 percentile. */
	std::size_t const nth_position = all_errors.size() / 2;
	std::nth_element(all_errors.begin(),
		all_errors.begin() + nth_position, all_errors.end());
	float const square_threshold = all_errors[nth_position].first
		* 10.0;

	/* Delete all tracks with errors above the threshold. */
	int num_deleted_tracks = 0;
	for (std::size_t i = nth_position; i < all_errors.size(); ++i){
		if (all_errors[i].first > square_threshold){
			vTracks.at(all_errors[i].second).valied=false;
			num_deleted_tracks += 1;
		}
	}
}

void updateParameters(Eigen::VectorXf& delta_x, std::vector<Eigen::Vector3f>& vBAPoints3d
	, const size_t cameraNums) {
	for (std::size_t i = 0; i < vBAPoints3d.size(); ++i)
		vBAPoints3d[i] += delta_x;
}

void computeReprojectionErrors(Eigen::VectorXf& vVector, std::vector<Observation>& vBAPoints2d
	, std::vector<Eigen::Vector3f>& vBAPoints3d
	, std::vector<int>& vBAPointMapping, std::vector<Viewport>& vViewports
	, Eigen::VectorXf *delta_x = nullptr) {
	for (size_t i = 0; i < vVector.rows(); i++){
		const Viewport& viewport = vViewports.at(vBAPoints2d.at(i).camera_id);
		Eigen::Vector3f point = vBAPoints3d.at(vBAPoints2d.at(i).point_id);
		const glm::vec2 screenPos = vBAPoints2d.at(i).pos;

		if (delta_x != nullptr)
			point += *delta_x;

		// Project point onto image plane.
		point = eigenFromGLM(viewport.pose.R)*point + eigenFromGLM(viewport.pose.T);
		
		point(0) = point(0) / point(2);
		point(1) = point(1) / point(2);

		// Distort reprojection.
		//this->radial_distort(rp + 0, rp + 1, dist);

		/* Compute reprojection error. */
		vVector(i) = point(0) - screenPos[0];
		vVector(i) = point(1) - screenPos[1];
	}
}

float computeMSE(Eigen::VectorXf& vVector) {
	float mse = 0.0;
	for (std::size_t i = 0; i < vVector.rows(); ++i)
		mse += vVector(i) * vVector(i);
	return mse / static_cast<float>(vVector.rows() / 2);
}

void analyticJacobian(Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& vMatrix) {

}

bool linearSolve(Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& F1
	, Eigen::VectorXf& vVector, Eigen::VectorXf delta_x) {

}

void baOptimize(std::vector<Observation>& vBAPoints2d, std::vector<Eigen::Vector3f>& vBAPoints3d
	, std::vector<int>& vBAPointMapping, std::vector<Viewport>& vViewports) {
	Eigen::VectorXf F, F_new;
	F.resize(vBAPoints2d.size());
	F_new.resize(vBAPoints2d.size());
	computeReprojectionErrors(F,vBAPoints2d,vBAPoints3d,vBAPointMapping,vViewports);
	float currentMSE = computeMSE(F);

	/* Levenberg-Marquard main loop. */
	for (int lm_iter = 0; ; ++lm_iter){
		if (lm_iter + 1 > 0
			&& (currentMSE < 1e-8)){
			cout << "BA: Satisfied MSE threshold." << std::endl;
			break;
		}

		/* Compute Jacobian. */
		Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> Jp;
		analyticJacobian(Jp);

		/* Perform linear step. */
		Eigen::VectorXf delta_x;
		bool linearSuccess=linearSolve(Jp, F, delta_x);

		/* Update reprojection errors and MSE after linear step. */
		double new_mse, delta_mse, delta_mse_ratio = 1.0;
		if (linearSuccess){
			computeReprojectionErrors(F_new, vBAPoints2d, vBAPoints3d, vBAPointMapping, vViewports, &delta_x);
			new_mse = computeMSE(F_new);
			delta_mse = currentMSE - new_mse;
			delta_mse_ratio = 1.0 - new_mse / currentMSE;
		}
		else{
			new_mse = currentMSE;
			delta_mse = 0.0;
		}
		bool successful_iteration = delta_mse > 0.0;

		/*
		 * Apply delta to parameters after successful step.
		 * Adjust the trust region to increase/decrease regularization.
		 */
		if (successful_iteration){
			cout << "BA: #" << std::setw(2) << std::left << lm_iter
				<< " success" << std::right
				<< ", MSE " << std::setw(11) << currentMSE
				<< " -> " << std::setw(11) << new_mse
				//<< ", CG " << std::setw(3) << cg_status.num_cg_iterations
				//<< ", TRR " << pcg_opts.trust_region_radius
				<< std::endl;

			//this->status.num_lm_iterations += 1;
			//this->status.num_lm_successful_iterations += 1;
			updateParameters(delta_x, vBAPoints3d,vViewports.size());
			std::swap(F, F_new);
			currentMSE = new_mse;

			/* Compute trust region update. FIXME delta_norm or mse? */
			//double const gain_ratio = delta_mse * (F.size() / 2)
			//	/ cg_status.predicted_error_decrease;
			//double const trust_region_update = 1.0 / std::max(1.0 / 3.0,
			//	(1.0 - MATH_POW3(2.0 * gain_ratio - 1.0)));
			//pcg_opts.trust_region_radius *= trust_region_update;
		}
		else{
			cout << "BA: #" << std::setw(2) << std::left << lm_iter
				<< " failure" << std::right
				<< ", MSE " << std::setw(11) << currentMSE
				<< ",    " << std::setw(11) << " "
				//<< " CG " << std::setw(3) << cg_status.num_cg_iterations
				//<< ", TRR " << pcg_opts.trust_region_radius
				<< std::endl;

			//this->status.num_lm_iterations += 1;
			//this->status.num_lm_unsuccessful_iterations += 1;
			//pcg_opts.trust_region_radius *= TRUST_REGION_RADIUS_DECREMENT;
		}

		/* Check termination due to LM iterations. */
		if (lm_iter + 1 < 0)
			continue;
		if (lm_iter + 1 >= 50){
			cout << "BA: Reached maximum LM iterations of "
				<< 50 << std::endl;
			break;
		}

		/* Check threshold on the norm of delta_x. */
		if (successful_iteration){
			if (delta_mse_ratio < 1e-4){
				cout << "BA: Satisfied delta mse ratio threshold of "
					<< 1e-4 << std::endl;
				break;
			}
		}
	}
}

struct baCamera {
	double *params;
};

struct baPoint {
	double *pos;
};

struct baObservation {
	double *screenPos;
	size_t cameraID;
	size_t pointID;
};

struct SnavelyReprojectionError {
	SnavelyReprojectionError(double observed_x, double observed_y)
		: observed_x(observed_x), observed_y(observed_y) {}

	template <typename T>
	bool operator()(const T* const camera,
		const T* const point,
		T* residuals) const {
		// camera[0,1,2] are the angle-axis rotation.
		T p[3];
		ceres::AngleAxisRotatePoint(camera, point, p);
		// camera[3,4,5] are the translation.
		p[0] += camera[3]; p[1] += camera[4]; p[2] += camera[5];

		// Compute the center of distortion. The sign change comes from
		// the camera model that Noah Snavely's Bundler assumes, whereby
		// the camera coordinate system has a negative z axis.
		T xp = -p[0] / p[2];
		T yp = -p[1] / p[2];

		// Apply second and fourth order radial distortion.
		const T& l1 = camera[7];
		const T& l2 = camera[8];
		T r2 = xp * xp + yp * yp;
		T distortion = T(1.0) + r2 * (l1 + l2 * r2);

		// Compute final projected point position.
		const T& focal = camera[6];
		T predicted_x = focal * distortion * xp;
		T predicted_y = focal * distortion * yp;

		// The error is the difference between the predicted and observed position.
		residuals[0] = predicted_x - T(observed_x);
		residuals[1] = predicted_y - T(observed_y);
		return true;
	}

	// Factory to hide the construction of the CostFunction object from the client code.
	static ceres::CostFunction* Create(const double observed_x,
		const double observed_y) {
		return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 9, 3>(
			new SnavelyReprojectionError(observed_x, observed_y)));
	}

	double observed_x;
	double observed_y;
};

void baOptimizeWithCeres(std::vector<baObservation>& vBAObservations
	, std::vector<baCamera>& vCameras
	, std::vector<baPoint>& vPointsWorld) {
	ceres::Problem problem;
	for (int i = 0; i < vBAObservations.size(); ++i) {
		ceres::CostFunction* cost_function =
			SnavelyReprojectionError::Create(
				vBAObservations.at(i).screenPos[0],
				vBAObservations.at(i).screenPos[1]);
		problem.AddResidualBlock(cost_function,
			NULL /* squared loss */,
			vCameras[vBAObservations.at(i).cameraID].params,
			vPointsWorld[vBAObservations.at(i).pointID].pos);
	}

	ceres::Solver::Options options;
	options.linear_solver_type = ceres::DENSE_SCHUR;
	options.minimizer_progress_to_stdout = true;
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);
	std::cout << summary.FullReport() << "\n";
}

void bundleAdjustmentFull(std::vector<Viewport>& vViewports, std::vector<Track>& vTracks) {
	/* Convert tracks and observations to BA data structures. */
	std::vector<Observation> ba_points_2d;
	std::vector<Eigen::Vector3f> ba_points_3d;
	std::vector<int> ba_tracks_mapping(vTracks.size(), -1);
	for (std::size_t i = 0; i < vTracks.size(); ++i){
		Track const& track = vTracks.at(i);
		if (!track.valied)
			continue;

		/* Add corresponding 3D point to BA. */
		glm::vec3 point=vTracks[i].pos;
		ba_tracks_mapping[i] = ba_points_3d.size();
		ba_points_3d.push_back(eigenFromGLM(point));

		/* Add all observations to BA. */
		for (std::size_t j = 0; j < track.tracks.size(); ++j){
			int const view_id = track.tracks[j].first;
			if (!vViewports.at(view_id).pose.valied)
				continue;

			int const feature_id = track.tracks[j].second;
			Viewport const& view = vViewports.at(view_id);
			const glm::vec2 & f2d = glm::vec2(view.keyPoints->at(feature_id).pt.x
				, view.keyPoints->at(feature_id).pt.y);

			Observation point;
			point.pos = f2d;
			point.camera_id = view_id;
			point.point_id = ba_tracks_mapping[i];
			ba_points_2d.push_back(point);
		}
	}

	/* Run bundle adjustment. */
	baOptimize(ba_points_2d, ba_points_3d, ba_tracks_mapping, vViewports);

	/* Transfer tracks back to SfM data structures. */
	std::size_t ba_track_counter = 0;
	for (std::size_t i = 0; i < vTracks.size(); ++i){
		Track& track = vTracks.at(i);
		if (!track.valied)
			continue;

		const Eigen::Vector3f & point = ba_points_3d[ba_track_counter];
		track.pos = glm::vec3(point(0), point(1) , point(2));
		ba_track_counter += 1;
	}
}

void bundleAdjustmentFullWithCeres(std::vector<Viewport>& vViewports, std::vector<Track>& vTracks) {	
	/* Convert tracks and observations to BA data structures. */
	std::vector<baObservation> baObservations;
	std::vector<baCamera> baCameras;
	std::vector<baPoint> pointsWorld;
	for (std::size_t i = 0; i < vViewports.size(); ++i) {
		baCameras.push_back(baCamera());
		baCamera& camera = baCameras.back();
		camera.params = new double[9];
		glm::vec3 angleVector = glm::eulerAngles(glm::quat_cast(vViewports[i].pose.R));
		camera.params[0] = angleVector[0];
		camera.params[1] = angleVector[1];
		camera.params[2] = angleVector[2];
		camera.params[3] = vViewports[i].pose.T[0];
		camera.params[4] = vViewports[i].pose.T[1];
		camera.params[5] = vViewports[i].pose.T[2];
		camera.params[6] = vViewports[i].pose.K[0][0];
		camera.params[7] = vViewports[i].pose.K[0][1];
		camera.params[8] = vViewports[i].pose.K[1][2];
	}

	for (std::size_t i = 0; i < vTracks.size(); ++i) {
		Track const& track = vTracks.at(i);
		if (!track.valied)
			continue;

		/* Add corresponding 3D point to BA. */
		baPoint point;
		point.pos = new double[3];
		point.pos[0] = vTracks[i].pos[0];
		point.pos[1] = vTracks[i].pos[1];
		point.pos[2] = vTracks[i].pos[2];
		pointsWorld.push_back(point);

		/* Add all observations to BA. */
		for (std::size_t j = 0; j < track.tracks.size(); ++j) {
			int const view_id = track.tracks[j].first;
			if (!vViewports.at(view_id).pose.valied)
				continue;

			int const feature_id = track.tracks[j].second;
			Viewport const& view = vViewports.at(view_id);
			const glm::vec2 & f2d = glm::vec2(view.keyPoints->at(feature_id).pt.x
				, view.keyPoints->at(feature_id).pt.y);

			baObservation point;
			point.screenPos = new double[2];
			point.screenPos[0] = f2d[0];
			point.screenPos[1] = f2d[1];
			point.cameraID = view_id;
			point.pointID = j;
			baObservations.push_back(point);
		}
	}

	/* Run bundle adjustment. */
	baOptimizeWithCeres(baObservations, baCameras, pointsWorld);

	/* Transfer tracks back to SfM data structures. */
	for (std::size_t i = 0; i < vViewports.size(); ++i) {
		baCamera& camera = baCameras[i];

		glm::mat4 R = glm::rotate(glm::mat4(1.f)
			, static_cast<float>(camera.params[0]),glm::vec3(1, 0, 0));
		R = glm::rotate(R, static_cast<float>(camera.params[1]), glm::vec3(0, 1, 0));
		R = glm::rotate(R, static_cast<float>(camera.params[2]), glm::vec3(0, 0, 1));
		vViewports.at(i).pose.R = R;
		vViewports.at(i).pose.T[0] = camera.params[3];
		vViewports.at(i).pose.T[1] = camera.params[4];
		vViewports.at(i).pose.T[2] = camera.params[5];
		vViewports.at(i).pose.K[0][0] = camera.params[6];
		vViewports.at(i).pose.K[0][1] = camera.params[7];
		vViewports.at(i).pose.K[1][2] = camera.params[8];
	}

	for (std::size_t i = 0; i < vTracks.size(); ++i) {
		Track & track = vTracks.at(i);
		if (!track.valied)
			continue;

		track.pos[0] = pointsWorld[i].pos[0];
		track.pos[1] = pointsWorld[i].pos[1];
		track.pos[2] = pointsWorld[i].pos[2];
	}
}

void findNextViews(std::vector<int>& next_views) {

}

bool reconstructNextView(int next_views) {

	return true;
}

void bundleAdjustmentSingleCamera(int next_views) {

}

void tryRestoreTracksForViews() {

}

int main(){
	std::vector<std::string> imageLists;
	imageLists.push_back("../../../my_test/00000.jpeg");
	imageLists.push_back("../../../my_test/00001.jpeg");
	imageLists.push_back("../../../my_test/00002.jpeg");
	std::vector<Viewport> viewports;
	std::vector<PairWiseCamera> pairCameras;
	std::vector<Track> tracks;

	getViewportFeatures(imageLists, viewports);
	normalizeImageCoordinate(viewports, 3000, 2000);
	matchViewportFeature(viewports, pairCameras);
	ransac(pairCameras);
	computeTracks(pairCameras, viewports, tracks);
	size_t initPairID=findInitPairs(viewports,pairCameras);

	triangulateNewTracks(viewports,tracks,2);
	invalidateLargeErrorTracks(viewports, tracks);
	bundleAdjustmentFull(viewports, tracks);

	/* Reconstruct remaining views. */
	int num_cameras_reconstructed = 2;
	int full_ba_num_skipped = 0;
	while (true){
		/* Find suitable next views for reconstruction. */
		std::vector<int> next_views;
		findNextViews(next_views);

		/* Reconstruct the next view. */
		int next_view_id = -1;
		for (std::size_t i = 0; i < next_views.size(); ++i){
			std::cout << std::endl;
			std::cout << "Adding next view ID " << next_views[i]
				<< " (" << (num_cameras_reconstructed + 1) << " of "
				<< viewports.size() << ")..." << std::endl;
			if (reconstructNextView(next_views[i])){
				next_view_id = next_views[i];
				break;
			}
		}

		if (next_view_id < 0){
			if (full_ba_num_skipped == 0){
				std::cout << "No valid next view." << std::endl;
				std::cout << "SfM reconstruction finished." << std::endl;
				break;
			}
			else{
				triangulateNewTracks(viewports, tracks, 3);
				std::cout << "Running full bundle adjustment..." << std::endl;
				bundleAdjustmentFull(viewports, tracks);
				invalidateLargeErrorTracks(viewports, tracks);
				full_ba_num_skipped = 0;
				continue;
			}
		}

		/* Run single-camera bundle adjustment. */
		//std::cout << "Running single camera bundle adjustment..." << std::endl;
		//bundleAdjustmentSingleCamera(next_view_id);
		num_cameras_reconstructed += 1;

		/* Run full bundle adjustment only after a couple of views. */
		int const full_ba_skip_views = false ? 0
			: std::min(100, num_cameras_reconstructed / 10);
		if (full_ba_num_skipped < full_ba_skip_views){
			std::cout << "Skipping full bundle adjustment (skipping "
				<< full_ba_skip_views << " views)." << std::endl;
			full_ba_num_skipped += 1;
		}
		else{
			triangulateNewTracks(viewports, tracks, 3);
			tryRestoreTracksForViews();
			std::cout << "Running full bundle adjustment..." << std::endl;
			bundleAdjustmentFull(viewports, tracks);
			invalidateLargeErrorTracks(viewports, tracks);
			full_ba_num_skipped = 0;
		}
	}

    return 0;
}
