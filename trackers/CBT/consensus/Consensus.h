#ifndef CONSENSUS_H

#define CONSENSUS_H

#include "common.h"

namespace cmt {

class Consensus
{
public:
    Consensus(){};

    void initialize(const vector<Point2f> & points_normalized);
    void estimateScaleRotation(const vector<Point2f> & points, float & scale, float & rotation);
    bool findConsensus(const vector<Point2f> & points, const float scale, const float rotation,
            Point2f & center, vector<Point2f> & points_inlier);

    bool estimate_scale;
    bool estimate_rotation;

private:
    float thr_cutoff;
    vector<Point2f> points_normalized;
    Mat distances_pairwise;
    Mat angles_pairwise;
};

} /* namespace cmt */

#endif /* end of include guard: CONSENSUS_H */
