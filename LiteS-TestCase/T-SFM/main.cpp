#include "CSIFT.h"
#include <Eigen/Core>
#include <Eigen/SVD>  
#include <Eigen/Dense> 
#include <Eigen/Geometry> 

#include <iostream>
#include <random>
#include <utility>
#include "CImage.h"
#include "util.h"
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "foundamental.h"
#include "unitTest.h"

#include <tbb/parallel_for.h>
#include <tbb/parallel_reduce.h>
#include <tbb/blocked_range.h>
#include <tbb/blocked_range2d.h>
#include <tbb/spin_mutex.h>
#include <tbb/task_scheduler_init.h>
#include <tbb/concurrent_vector.h>

const float RANSAC_INLIER_THRESHOLD = 0.0015f;

void getViewportFeatures(const std::vector<std::string> &vImageLists
	,std::vector<Viewport> &vOutViewports) {
	for (size_t i = 0; i < vImageLists.size();++i) {
		vOutViewports.push_back(Viewport());
		vOutViewports.back().ID = i;
		vOutViewports.back().keyPoints = new std::vector<cv::KeyPoint>;
		vOutViewports.back().descriptor = new cv::Mat;
		vOutViewports.back().keyPoints->clear();
		computeFeaturesOpenCV(vImageLists[i], vOutViewports.back().keyPoints
			, vOutViewports.back().descriptor, vOutViewports.back().originalImage
			, vOutViewports.back().width, vOutViewports.back().height);
		//CImage<float>* tempImage = downsampleBy2(downsampleBy2(&vOutViewports.back().originalImage));
		//tempImage->save(std::string("../../../my_test/test_low/") + std::to_string(i) + std::string(".png"));

	}
}

void matchViewportFeature(const std::vector<Viewport> &vViewports
	, std::vector<PairWiseCamera> &vOutPairs) {
	for (size_t view1 = 0; view1 < vViewports.size(); view1++){
		for (size_t view2 = 0; view2 < view1; view2++) {
			if(view1==view2)
				continue;
			std::vector<cv::DMatch> goodMatches;
			matchFeatureOpenCV(vViewports[view1].descriptor, vViewports[view2].descriptor
				, goodMatches);
			vOutPairs.push_back(PairWiseCamera());
			PairWiseCamera& p = vOutPairs.back();
			p.ID1 = view1;
			p.ID2 = view2;
			for (auto &matchItem : goodMatches) {
				float x1 = vViewports[view1].keyPoints->at(matchItem.queryIdx).pt.x;
				float y1 = vViewports[view1].keyPoints->at(matchItem.queryIdx).pt.y;
				float x2 = vViewports[view2].keyPoints->at(matchItem.trainIdx).pt.x;
				float y2 = vViewports[view2].keyPoints->at(matchItem.trainIdx).pt.y;
			
				p.matchResult.push_back(std::make_pair < Eigen::Vector3f, Eigen::Vector3f>(
					Eigen::Vector3f(x1, y1, matchItem.queryIdx)
					, Eigen::Vector3f(x2, y2, matchItem.trainIdx)));
			}
		}
	}
}

void drawMatchResult(const std::vector<Viewport> &vViewports
	,const std::vector<PairWiseCamera> &vPaitCameras) {
	for (const PairWiseCamera& pairItem : vPaitCameras) {
		const Viewport& view1 = vViewports[pairItem.ID1];
		const Viewport& view2 = vViewports[pairItem.ID2];

		cv::Mat companionImage(cv::Size(view1.width + view2.width + 5
			, std::max(view1.height, view2.height))
			, CV_8UC1, cv::Scalar(0));
		for (int y = 0; y < companionImage.rows; ++y) {
			for (int x = 0; x < companionImage.cols; x++)
			{
				if (x < view1.width)
					companionImage.at<uchar>(y, x) = view1.originalImage.at(x, y)*255.0f;
				else if (x >= view1.width&&x < view1.width + 5)
					companionImage.at<uchar>(y, x) = 0;
				else
					companionImage.at<uchar>(y, x) = view2.originalImage.at(x - (int)view1.width - 5, y)*255.0f;
			}
		}

		for (auto &matchItem : pairItem.matchResult) {
			size_t x1 = matchItem.first(0)*view1.width;
			size_t y1 = matchItem.first(1)*view1.height;
			size_t x2 = matchItem.second(0) * view2.width;
			size_t y2 = matchItem.second(1) * view2.height;

			cv::line(companionImage, cv::Point2f(x1, y1), cv::Point2f(x2 + view1.width + 5, y2)
				, cv::Scalar(0, 0, 255), 1, CV_AA);
		}

		cv::namedWindow("MyWindow", CV_WINDOW_NORMAL);
		cv::resizeWindow("MyWindow", cv::Size(1200, 600));
		cv::imshow("MyWindow", companionImage);
		cv::waitKey(0);

	}
}

void findInlier(Eigen::Matrix<float, 3, 3> &vFoundamentalMatrix 
	,const PairWiseCamera& vPairCameras, std::vector<size_t> &result) {
	result.resize(0);
	double const squared_thres = RANSAC_INLIER_THRESHOLD * RANSAC_INLIER_THRESHOLD;
	for (std::size_t i = 0; i < vPairCameras.matchResult.size(); ++i)
	{
		double error = sampsonDistance(vFoundamentalMatrix, vPairCameras.matchResult[i]);
		if (error < squared_thres)
			result.push_back(i);
	}
}

void ransac(std::vector<PairWiseCamera>& pairCameras) {
	for (auto& pairItem : pairCameras) {
		if(pairItem.matchResult.size() < 8)
			continue;

		std::pair<Eigen::Matrix<float, 3, 3>, std::vector<size_t>> result = 
			tbb::parallel_reduce(tbb::blocked_range<size_t>(size_t(0), size_t(10000))
			, std::make_pair<Eigen::Matrix<float, 3, 3>, std::vector<size_t>>(Eigen::Matrix<float, 3, 3>(), std::vector<size_t>())
			, [=](const tbb::blocked_range<size_t>& r
				, std::pair<Eigen::Matrix<float, 3, 3>, std::vector<size_t>> value)
			->std::pair<Eigen::Matrix<float, 3, 3>, std::vector<size_t>> {
			std::vector<size_t> bestInliers;
			Eigen::Matrix<float, 3, 3> bestFoundamentalMatrix;

			for (int i = r.begin(); i != r.end(); ++i) {
				Eigen::VectorXf solution = estimate8Points(pairItem);
				Eigen::Matrix<float, 3, 3> foundamentalMatrix = enforceConstrains(solution);
				std::vector<size_t> localInlier;
				findInlier(foundamentalMatrix, pairItem, localInlier);
				if (bestInliers.size() < localInlier.size()) {
					bestFoundamentalMatrix = foundamentalMatrix;
					bestInliers = localInlier;
				}
			}
			if (value.second.size() >= bestInliers.size())
				return value;
			else
				return std::make_pair(bestFoundamentalMatrix, bestInliers);
		}
			, [](std::pair<Eigen::Matrix<float, 3, 3>, std::vector<size_t>> x
				, std::pair< Eigen::Matrix<float, 3, 3>, std::vector<size_t>> y)
			->std::pair<Eigen::Matrix<float, 3, 3>, std::vector<size_t>> {
			return x.second.size() > y.second.size() ? x : y;
		});


		//std::vector<size_t> inliers;
		//Eigen::Matrix<float, 3, 3> bestFoundamentalMatrix;
		//for (size_t iter = 0; iter < 10000; iter++){
		//	Eigen::VectorXf solution = estimate8Points(pairItem);
		//	Eigen::Matrix<float, 3, 3> foundamentalMatrix = enforceConstrains(solution);
		//	std::vector<size_t> localInlier;
		//	findInlier(foundamentalMatrix, pairItem, localInlier);
	
		//	if (inliers.size() < localInlier.size()) {
		//		bestFoundamentalMatrix = foundamentalMatrix;
		//		inliers = localInlier;
		//	}
		//}


		std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> newMatchResult;
		for (auto& inlierIndex : result.second)
			newMatchResult.push_back(pairItem.matchResult[inlierIndex]);
		pairItem.matchResult = newMatchResult;
		pairItem.foundamentalMatrix = result.first;

		//cout << bestFoundamentalMatrix << endl;
		//cout << endl;
		//cout << result.first << endl;
		//cout << endl;
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
			size_t featureIndex1 = pairCamera.matchResult[matchIndex].first(2);
			size_t featureIndex2 = pairCamera.matchResult[matchIndex].second(2);

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

		std::random_device rd;
		std::set<int> result;
		std::uniform_int_distribution<int> gen(0, vPairCamera.matchResult.size()-1);
		std::mt19937 myRand(rd());
		while (result.size() < 4)
			result.insert(gen(myRand));

		std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> fourFeatures(4);
		std::set<int>::const_iterator iter = result.begin();
		for (std::size_t i = 0; i < 4; ++i, ++iter)
			fourFeatures[i] = vPairCamera.matchResult[*iter];

		// Compute homography
		Eigen::Matrix<float,8,9> A;
		for (std::size_t featurePairID = 0; featurePairID < fourFeatures.size(); ++featurePairID)
		{
			std::size_t const row1 = 2 * featurePairID;
			std::size_t const row2 = 2 * featurePairID + 1;
			std::pair<Eigen::Vector3f, Eigen::Vector3f> const& match = fourFeatures[featurePairID];
			A(row1 , 0) = 0.0;
			A(row1 , 1) = 0.0;
			A(row1 , 2) = 0.0;
			A(row1 , 3) = match.first(0);
			A(row1 , 4) = match.first(1);
			A(row1 , 5) = 1.0;
			A(row1 , 6) = -match.second(1) * match.first(0);
			A(row1 , 7) = -match.second(1) * match.first(1);
			A(row1 , 8) = -match.second(1);

			A(row2, 0) = -match.first(0);
			A(row2, 1) = -match.first(1);
			A(row2, 2) = -1.0;
			A(row2, 3) = 0.0;
			A(row2, 4) = 0.0;
			A(row2, 5) = 0.0;
			A(row2, 6) = match.second(0) * match.first(0);
			A(row2, 7) = match.second(0) * match.first(1);
			A(row2, 8) = match.second(0);
		}

		/* Compute homography matrix using SVD. */
		Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
		Eigen::VectorXf V = svd.matrixV().col(8);
		Eigen::Matrix3f homographyMatrix;
		for (size_t y = 0; y < 3; y++)
			for (size_t x = 0; x < 3; x++)
				homographyMatrix(y,x) = V(y * 3 + x);

		//???????????????????????????????????????????????
		homographyMatrix /= homographyMatrix(2,2);

		float const square_threshold = 0.05f * 0.05f;
		inliers.resize(0);
		for (std::size_t i = 0; i < vPairCamera.matchResult.size(); ++i){
			float error = symmetricTransferError(homographyMatrix, vPairCamera.matchResult[i]);
			if (error < square_threshold)
				inliers.push_back(i);
		}

		if (inliers.size() > bestInliersNum){
			bestInliersNum = inliers.size();
		}
	}

	return bestInliersNum;
}

bool computePose(std::vector<Viewport>& viewports, const PairWiseCamera& vPairCamera
	, CameraPose& vPose1, CameraPose& vPose2) {
	// Compute fundamental matrix from pair correspondences.
	const Eigen::Matrix3f fundamental=vPairCamera.foundamentalMatrix;

	/* Populate K-matrices. */
	Viewport& view_1 = viewports[vPairCamera.ID1];
	Viewport& view_2 = viewports[vPairCamera.ID2];

	/* Compute essential matrix from fundamental matrix (HZ (9.12)). */
	Eigen::Matrix3f essensialMatrix = view_2.pose.K.inverse().transpose()
		* fundamental * view_1.pose.K.inverse();

	/* Compute pose from essential. */
	std::vector<CameraPose> poses;
	poseFromEssential(essensialMatrix, poses);

	/* Find the correct pose using point test (HZ Fig. 9.12). */
	bool found_pose = false;
	for (std::size_t i = 0; i < poses.size(); ++i){
		poses[i].K = viewports[vPairCamera.ID2].pose.K;
		std::pair<Eigen::Vector2f, Eigen::Vector2f> matchPos = std::make_pair<Eigen::Vector2f, Eigen::Vector2f>(
			Eigen::Vector2f(vPairCamera.matchResult[0].first(0), vPairCamera.matchResult[0].first(1))
			, Eigen::Vector2f(vPairCamera.matchResult[0].second(0), vPairCamera.matchResult[0].second(1)));
		Eigen::Vector3f x = triangulateMatch(matchPos, view_1.pose, poses[i]);
		Eigen::Vector3f x1 = view_1.pose.R * x + view_1.pose.T;
		Eigen::Vector3f x2 = poses[i].R * x + poses[i].T;

		if (x1[2] > 0.0f && x2[2] > 0.0f){
			vPose2 = poses[i];
			found_pose = true;
			break;
		}
	}
	return found_pose;
}

float angleForPose(const PairWiseCamera& vPairCamera
	,const CameraPose& vPose1, const CameraPose& vPose2) {
	// Compute transformation from image coordinates to viewing direction
	Eigen::Matrix3f T1 = vPose1.R.transpose() * vPose1.K.inverse();
	Eigen::Matrix3f T2 = vPose2.R.transpose() * vPose2.K.inverse();

	/* Compute triangulation angle for each correspondence. */
	std::vector<float> cos_angles;
	cos_angles.reserve(vPairCamera.matchResult.size());
	for (std::size_t i = 0; i < vPairCamera.matchResult.size(); ++i){
		Eigen::Vector3f p1(vPairCamera.matchResult[i].first(0), vPairCamera.matchResult[i].first(1), 1.f);
		Eigen::Vector3f p2(vPairCamera.matchResult[i].second(0), vPairCamera.matchResult[i].second(1), 1.f);
		p1 = (T1*p1);
		p1.normalize();
		p2 = (T2*p2);
		p2.normalize();
		cos_angles.push_back(p1.dot(p2));
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
	float const angle_d = vAngle*180.f/3.1415926f;

	/* Score for matches (min: 20, good: 200). */
	float f1 = 2.0f / (1.0f + std::exp((20.0f - matchesScore) * 6.0f / 200.0f)) - 1.0f;
	/* Score for angle (min 1 degree, good 8 degree). */
	float f2 = 2.0f / (1.0f + std::exp((1.0f - angle_d) * 6.0f / 8.0f)) - 1.0f;
	/* Score for H-Inlier (max 70%, good 40%). */
	float f3 = 2.0f / (1.0f + std::exp((inlierScore - 0.7f) * 6.0f / 0.4f)) - 1.0f;

	f1 = std::clamp(f1, 0.0f, 1.0f);
	f2 = std::clamp(f2, 0.0f, 1.0f);
	f3 = std::clamp(f3, 0.0f, 1.0f);
	return f1 * f2 * f3;
}

bool triangulate(std::vector<CameraPose>& vPoses,std::vector<Eigen::Vector2f>& vPositions
	, Eigen::Vector3f& vPos3D, std::vector<std::size_t>* vOutliers=nullptr) {
	if (vPoses.size() < 2)
		throw std::invalid_argument("At least two poses required");
	if (vPoses.size() != vPositions.size())
		throw std::invalid_argument("Poses and positions size mismatch");

	// Check all possible pose pairs for successful triangulation
	std::vector<std::size_t> bestOutliers(vPositions.size());
	Eigen::Vector3f bestPos(0.0f,0.f,0.f);
	for (std::size_t p1 = 0; p1 < vPoses.size(); ++p1)
		for (std::size_t p2 = p1 + 1; p2 < vPoses.size(); ++p2){
			/* Triangulate position from current pair */
			std::vector<CameraPose> pose_pair;
			std::vector<Eigen::Vector2f> position_pair;
			pose_pair.push_back(vPoses[p1]);
			pose_pair.push_back(vPoses[p2]);
			position_pair.push_back(vPositions[p1]);
			position_pair.push_back(vPositions[p2]);
			Eigen::Vector3f tmp_pos = triangulateTrack(position_pair, pose_pair);
			assert(!(std::isnan(tmp_pos[0]) || std::isnan(tmp_pos[0]) ||
				std::isnan(tmp_pos[1]) || std::isnan(tmp_pos[1]) ||
				std::isnan(tmp_pos[2]) || std::isnan(tmp_pos[2])));

			// Check if pair has small triangulation angle
			Eigen::Vector3f camera_pos;
			camera_pos = -(pose_pair[0].R).inverse()*pose_pair[0].T;
			Eigen::Vector3f ray0 = (tmp_pos - camera_pos).normalized();
			camera_pos = -(pose_pair[1].R).inverse()*pose_pair[1].T;
			Eigen::Vector3f ray1 = (tmp_pos - camera_pos).normalized();
			float const cos_angle = ray0.dot(ray1);
			if (cos_angle > std::cos(1*3.1415926/180))
				continue;

			// Check error in all input poses and find outliers.
			std::vector<std::size_t> tmp_outliers;
			for (std::size_t i = 0; i < vPoses.size(); ++i){

				Eigen::Vector3f x = vPoses[i].R * tmp_pos   + vPoses[i].T;

				/* Reject track if it appears behind the camera. */
				if (x[2] <= 0.0){
					tmp_outliers.push_back(i);
					continue;
				}


				x = vPoses[i].K * x;
				Eigen::Vector2f x2d(x[0] / x[2], x[1] / x[2]);
				float error = (vPositions[i]-x2d).norm();
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
	if ((bestPos).norm() == 0.0f){
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

size_t findInitPairs(std::vector<Viewport>& vViewports
	, const std::vector<PairWiseCamera>& vPairCameras) {
	int viewID1 = -1, viewID2 = -1;
	std::vector<size_t> candidates(vPairCameras.size());
	
	std::iota(candidates.begin(), candidates.end(), 0);
	std::sort(candidates.begin(), candidates.end()
		, [=](const size_t& v1,const size_t& v2)->bool {
		return vPairCameras[v1].matchResult.size() > vPairCameras[v2].matchResult.size();
	});

	bool found_pair = false;
	size_t resultPairID =std::numeric_limits<size_t>::max();
	std::vector<float> pairScores(candidates.size(), 0.0f);

	CameraPose pose1, pose2;

	for (std::size_t candidateIndex = 0; candidateIndex < candidates.size(); ++candidateIndex) {
		if (found_pair)
			continue;

		// Reject pairs with 50 or fewer matches.
		if (vPairCameras[candidates[candidateIndex]].matchResult.size() < 50){
			continue;
		}


		// Reject pairs with too high percentage of homography inliers.
		std::size_t numInliers = computeHomographyInliers(vPairCameras[candidates[candidateIndex]]);
		float percentage = static_cast<float>(numInliers) / vPairCameras[candidates[candidateIndex]].matchResult.size();
		if (percentage > 0.8f){
			continue;
		}

		// Compute initial pair pose.
		bool const found_pose = computePose(vViewports, vPairCameras[candidates[candidateIndex]], pose1,pose2);
		if (!found_pose){
			continue;
		}

		// Rejects pairs with bad triangulation angle.
		float const angle = angleForPose(vPairCameras[candidates[candidateIndex]], pose1, pose2);
		pairScores[candidateIndex] = scoreForPair(vPairCameras[candidates[candidateIndex]], numInliers, angle);
		if (angle < 5 * 3.1415926 / 180)
			continue;

		// If all passes, run triangulation to ensure correct pair
		std::vector<CameraPose> poses;
		poses.push_back(pose1);
		poses.push_back(pose2);
		std::size_t successful_triangulations = 0;
		std::vector<Eigen::Vector2f> positions(2);
		for (std::size_t j = 0; j < vPairCameras[candidates[candidateIndex]].matchResult.size(); ++j)
		{
			positions[0] = Eigen::Vector2f(vPairCameras[candidates[candidateIndex]].matchResult[j].first(0)
				, vPairCameras[candidates[candidateIndex]].matchResult[j].first(1));
			positions[1] = Eigen::Vector2f(vPairCameras[candidates[candidateIndex]].matchResult[j].second(0)
				, vPairCameras[candidates[candidateIndex]].matchResult[j].second(1));
			Eigen::Vector3f pos3d;
			if (triangulate(poses, positions, pos3d))
				successful_triangulations += 1;
		}
		if (successful_triangulations * 2 < vPairCameras[candidates[candidateIndex]].matchResult.size())
			continue;

		found_pair = true;
		viewID1 = vPairCameras[candidates[candidateIndex]].ID1;
		viewID2 = vPairCameras[candidates[candidateIndex]].ID2;
		resultPairID = candidates[candidateIndex];
		break;
	}

	
	if (found_pair) {
		vViewports[viewID1].pose = pose1;
		vViewports[viewID2].pose = pose2;
		return resultPairID;
	}

	float best_score = 0.0f;
	for (std::size_t i = 0; i < pairScores.size(); ++i){
		if (pairScores[i] <= best_score)
			continue;
		best_score = pairScores[i];
		resultPairID = candidates[i];
	}

	/* Recompute pose for pair with best score. */
	if (best_score > 0.0f) {
		computePose(vViewports,vPairCameras[resultPairID],
			vViewports[vPairCameras[resultPairID].ID1].pose
			, vViewports[vPairCameras[resultPairID].ID2].pose);
		return resultPairID;
	}
	else
		throw "lalala";
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
		std::vector<Eigen::Vector2f> pos;
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
			pos.push_back(Eigen::Vector2f(viewport.keyPoints->at(feature_id).pt.x
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
		Eigen::Vector3f track_pos;
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
		Eigen::Vector3f const& pos3d = vTracks.at(i).pos;
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

			Eigen::Vector2f const& pos2d = Eigen::Vector2f(viewport.keyPoints->at(feature_id).pt.x
				, viewport.keyPoints->at(feature_id).pt.y);

			/* Project 3D feature and compute reprojection error. */
			Eigen::Vector3f x = pose.R * pos3d + pose.T;
			Eigen::Vector2f x2d(x[0] / x[2], x[1] / x[2]);
			float r2 = x2d.norm();
			//x2d *= (1.0 + r2 * (viewport.radial_distortion[0]
			//	+ viewport.radial_distortion[1] * r2))
			//	* pose.get_focal_length();
			total_error += (pos2d - x2d).norm();
			num_valid += 1;
		}
		total_error /= static_cast<float>(num_valid);
		all_errors.push_back(std::pair<float, size_t>(total_error, i));
	}

	if (num_valid_tracks < 2)
		return;

	/* Find the 1/2 percentile. */
	std::size_t const nth_position = all_errors.size() / 2;
	std::nth_element(all_errors.begin(),
		all_errors.begin() + nth_position, all_errors.end());
	float const square_threshold = all_errors[nth_position].first
		* 10.0f;

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
		const Eigen::Vector2f screenPos = vBAPoints2d.at(i).pos;

		if (delta_x != nullptr)
			point += *delta_x;

		// Project point onto image plane.
		point = viewport.pose.R*point + viewport.pose.T;
		
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
	return false;
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
		Eigen::Vector3f point=vTracks[i].pos;
		ba_tracks_mapping[i] = ba_points_3d.size();
		ba_points_3d.push_back(point);

		/* Add all observations to BA. */
		for (std::size_t j = 0; j < track.tracks.size(); ++j){
			int const view_id = track.tracks[j].first;
			if (!vViewports.at(view_id).pose.valied)
				continue;

			int const feature_id = track.tracks[j].second;
			Viewport const& view = vViewports.at(view_id);
			const Eigen::Vector2f & f2d = Eigen::Vector2f(view.keyPoints->at(feature_id).pt.x
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
		track.pos = Eigen::Vector3f(point(0), point(1) , point(2));
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
		Eigen::Vector3f angleVector = vViewports[i].pose.R.eulerAngles(0,1,2);
		
		camera.params[0] = angleVector[0];
		camera.params[1] = angleVector[1];
		camera.params[2] = angleVector[2];
		camera.params[3] = vViewports[i].pose.T[0];
		camera.params[4] = vViewports[i].pose.T[1];
		camera.params[5] = vViewports[i].pose.T[2];
		camera.params[6] = vViewports[i].pose.K(0,0);
		camera.params[7] = vViewports[i].pose.K(0,1);
		camera.params[8] = vViewports[i].pose.K(1,2);
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

		Eigen::Matrix3f R;
		R = Eigen::AngleAxisf(static_cast<float>(camera.params[0]), Eigen::Vector3f::UnitX())
			* Eigen::AngleAxisf(static_cast<float>(camera.params[1]), Eigen::Vector3f::UnitY())
			* Eigen::AngleAxisf(static_cast<float>(camera.params[2]), Eigen::Vector3f::UnitZ());
		vViewports.at(i).pose.R = R;
		vViewports.at(i).pose.T[0] = static_cast<float>(camera.params[3]);
		vViewports.at(i).pose.T[1] = static_cast<float>(camera.params[4]);
		vViewports.at(i).pose.T[2] = static_cast<float>(camera.params[5]);
		vViewports.at(i).pose.K(0,0) = static_cast<float>(camera.params[6]);
		vViewports.at(i).pose.K(0,1) = static_cast<float>(camera.params[7]);
		vViewports.at(i).pose.K(1,2) = static_cast<float>(camera.params[8]);
	}

	for (std::size_t i = 0; i < vTracks.size(); ++i) {
		Track & track = vTracks.at(i);
		if (!track.valied)
			continue;

		track.pos[0] = static_cast<float>(pointsWorld[i].pos[0]);
		track.pos[1] = static_cast<float>(pointsWorld[i].pos[1]);
		track.pos[2] = static_cast<float>(pointsWorld[i].pos[2]);
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



int main(int argc, char* argv[]){
	bool testEnable = true;
	if (testEnable)
		unitTest(argc, argv);

	std::vector<std::string> imageLists;
	imageLists.push_back("../../../my_test/test_low/0.png");
	imageLists.push_back("../../../my_test/test_low/1.png");
	imageLists.push_back("../../../my_test/test_low/2.png");
	imageLists.push_back("../../../my_test/test_low/3.png");
	imageLists.push_back("../../../my_test/test_low/4.png");
	//imageLists.push_back("../../../my_test/test/00000.jpeg");
	//imageLists.push_back("../../../my_test/test/00001.jpeg");
	//imageLists.push_back("../../../my_test/test/00002.jpeg");
	//imageLists.push_back("../../../my_test/test/00003.jpeg");
	//imageLists.push_back("../../../my_test/test/00004.jpeg");
	std::vector<Viewport> viewports;
	std::vector<PairWiseCamera> pairCameras;
	std::vector<Track> tracks;

	getViewportFeatures(imageLists, viewports);
	matchViewportFeature(viewports, pairCameras);
	//drawMatchResult(viewports, pairCameras);
	ransac(pairCameras);
	//drawMatchResult(viewports, pairCameras);
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
