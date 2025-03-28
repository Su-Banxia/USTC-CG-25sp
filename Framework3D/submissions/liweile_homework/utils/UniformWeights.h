#pragma once
#include "WeightCalculator.h"

namespace USTC_CG
{

class UniformWeights : public WeightCalculator
{
public:
    using WeightCalculator::WeightCalculator;
    
    virtual double calculator_weight () override
    {
        return 1.0; // v_i.valence();
    }
};

}