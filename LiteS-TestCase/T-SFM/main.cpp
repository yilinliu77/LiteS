#include "CSIFT.h"
#include <glm/matrix.hpp>
#include <Eigen/Core>
#include <Eigen/SVD>  
#include <Eigen/Dense>  
#include <iostream>
#include <random>
#include <utility>
#include "CImage.h"


const float RANSAC_INLIER_THRESHOLD = 0.0015f;

struct Viewport {
	size_t ID;

	std::vector<cv::KeyPoint>* keyPoints;
	cv::Mat* descriptor;

	std::vector<int> trackIDs;
};

struct PairWiseCamera {
	size_t ID1;
	size_t ID2;
	
	std::vector<std::pair<glm::vec3, glm::vec3>> matchResult;
	glm::mat3 foundamentalMatrix;
};

struct Track {

	std::vector<std::pair<size_t, size_t>> tracks;
};

void getViewportFeatures(const std::vector<std::string> &vImageLists
	,std::vector<Viewport> &vOutViewports) {
	for (size_t i = 0; i < vImageLists.size();++i) {
		vOutViewports.push_back(Viewport());
		vOutViewports.back().ID = i;
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
			p.ID2 = view1;
			for (auto &matchItem : goodMatches) {
				float x1 = vViewports[view1].keyPoints->at(matchItem.trainIdx).pt.x;
				float y1 = vViewports[view1].keyPoints->at(matchItem.trainIdx).pt.y;
				float x2 = vViewports[view2].keyPoints->at(matchItem.queryIdx).pt.x;
				float y2 = vViewports[view2].keyPoints->at(matchItem.queryIdx).pt.y;
			
				p.matchResult.push_back(std::make_pair < glm::vec3, glm::vec3 >(
					glm::vec3(x1, y1, matchItem.trainIdx), glm::vec3(x2, y2, matchItem.queryIdx)));
			}
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
				viewport1.trackIDs[featureIndex1] = vTracks.size();
				viewport2.trackIDs[featureIndex2] = vTracks.size();
			}
			else if (-1 != viewport1.trackIDs[featureIndex1] && -1 == viewport2.trackIDs[featureIndex2]) {
				Track& track = vTracks[viewport2.trackIDs[featureIndex2]];
				viewport1.trackIDs[featureIndex1] = viewport2.trackIDs[featureIndex2];
				track.tracks.push_back(std::make_pair(pairCamera.ID1, featureIndex1));
			}
			else if (-1 == viewport1.trackIDs[featureIndex1] && -1 != viewport2.trackIDs[featureIndex2]) {
				Track& track = vTracks[viewport1.trackIDs[featureIndex1]];
				viewport2.trackIDs[featureIndex2] = viewport1.trackIDs[featureIndex1];
				track.tracks.push_back(std::make_pair(pairCamera.ID2, featureIndex2));
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
			A[row2 + 0] = 0.0;
			A[row2 + 1] = 0.0;
			A[row2 + 2] = 0.0;
			A[row2 + 3] = match.first.x;
			A[row2 + 4] = match.first.y;
			A[row2 + 5] = 1.0;
			A[row2 + 6] = -match.second.y * match.first.x;
			A[row2 + 7] = -match.second.y * match.first.y;
			A[row2 + 8] = -match.second.y;

			A[row1 + 0] = match.first.x;
			A[row1 + 1] = match.first.y;
			A[row1 + 2] = 1.0;
			A[row1 + 3] = 0.0;
			A[row1 + 4] = 0.0;
			A[row1 + 5] = 0.0;
			A[row1 + 6] = -match.second.x * match.first.x;
			A[row1 + 7] = -match.second.x * match.first.y;
			A[row1 + 8] = -match.second.x;
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

void computePose(const PairWiseCamera& vPairCamera) {
	// Compute fundamental matrix from pair correspondences.
	const glm::mat3 fundamental=vPairCamera.foundamentalMatrix;

	/* Populate K-matrices. */
	Viewport const& view_1 = this->viewports->at(candidate.view_1_id);
	Viewport const& view_2 = this->viewports->at(candidate.view_2_id);
	pose1->set_k_matrix(view_1.focal_length, 0.0, 0.0);
	pose1->init_canonical_form();
	pose2->set_k_matrix(view_2.focal_length, 0.0, 0.0);

	/* Compute essential matrix from fundamental matrix (HZ (9.12)). */
	EssentialMatrix E = pose2->K.transposed() * fundamental * pose1->K;

	/* Compute pose from essential. */
	std::vector<CameraPose> poses;
	pose_from_essential(E, &poses);

	/* Find the correct pose using point test (HZ Fig. 9.12). */
	bool found_pose = false;
	for (std::size_t i = 0; i < poses.size(); ++i)
	{
		poses[i].K = pose2->K;
		if (is_consistent_pose(candidate.matches[0], *pose1, poses[i]))
		{
			*pose2 = poses[i];
			found_pose = true;
			break;
		}
	}
	return found_pose;
}

void findInitPairs(const std::vector<Viewport> vViewports, const std::vector<PairWiseCamera> vPairCameras) {
	int viewID1 = -1, viewID2 = -1;
	std::vector<PairWiseCamera> candidates(vPairCameras);
	
	std::sort(candidates.begin(), candidates.end()
		, [](const PairWiseCamera& v1,const PairWiseCamera& v2)->bool {
		if (v1.matchResult.size() > v2.matchResult.size())
			return true;
	});

	bool found_pair = false;
	std::size_t found_pair_id = std::numeric_limits<std::size_t>::max();
	std::vector<float> pair_scores(candidates.size(), 0.0f);

	for (std::size_t candidateIndex = 0; candidateIndex < candidates.size(); ++candidateIndex) {
		if (found_pair)
			continue;

		// Reject pairs with 8 or fewer matches.
		if (candidates[candidateIndex].matchResult.size() < 50){
			continue;
		}

		// Reject pairs with too high percentage of homograhy inliers.
		std::size_t num_inliers = computeHomographyInliers(candidates[candidateIndex]);
		float percentage = static_cast<float>(num_inliers) / candidates[candidateIndex].matchResult.size();
		if (percentage > 0.8f){
			continue;
		}

		// Compute initial pair pose.
		CameraPose pose1, pose2;
		bool const found_pose = computePose(candidate, &pose1, &pose2);
		if (!found_pose){
			this->debug_output(candidate, num_inliers);
			continue;
		}

		// Rejects pairs with bad triangulation angle.
		double const angle = this->angle_for_pose(candidate, pose1, pose2);
		pair_scores[i] = this->score_for_pair(candidate, num_inliers, angle);
		this->debug_output(candidate, num_inliers, angle);
		if (angle < this->opts.min_triangulation_angle)
			continue;

		// If all passes, run triangulation to ensure correct pair
		Triangulate::Options triangulate_opts;
		Triangulate triangulator(triangulate_opts);
		std::vector<CameraPose const*> poses;
		poses.push_back(&pose1);
		poses.push_back(&pose2);
		std::size_t successful_triangulations = 0;
		std::vector<math::Vec2f> positions(2);
		Triangulate::Statistics stats;
		for (std::size_t j = 0; j < candidate.matches.size(); ++j)
		{
			positions[0] = math::Vec2f(candidate.matches[j].p1);
			positions[1] = math::Vec2f(candidate.matches[j].p2);
			math::Vec3d pos3d;
			if (triangulator.triangulate(poses, positions, &pos3d, &stats))
				successful_triangulations += 1;
		}
		if (successful_triangulations * 2 < candidate.matches.size())
			continue;

		found_pair = true;
		viewID1 = candidates[candidateIndex].ID1;
		viewID2 = candidates[candidateIndex].ID2;
	}
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
	matchViewportFeature(viewports,pairCameras);
	ransac(pairCameras);
	computeTracks(pairCameras, viewports, tracks);
	findInitPairs();


    return 0;
}
