#pragma once

#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>

namespace g2o 
{

    class EdgeSE3Mixture : public EdgeSE3
    {
    public:
        EdgeSE3Mixture();

        virtual bool read(std::istream& is);
        virtual bool write(std::ostream& os) const;
        void computeError();
        void linearizeOplus();
        bool isOutlier();

        double weight;
        double determinant_c;
        double determinant_null;
        bool determinant_set;
        bool nullHypothesisMoreLikely;
        
        InformationType information_nullHypothesis;
        double nu_nullHypothesis;
        InformationType information_constraint;
        double nu_constraint ;
    };
}