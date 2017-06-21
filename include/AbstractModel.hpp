#pragma once

#include <iostream>
#include <stdexcept>
#include <vector>
#include <array>
#include <memory>

namespace GRANSAC
{
    typedef double VPFloat;

    // Each abstract model is made of abstract parameters
    // Could be anything from a point (that make a 2D line or 3D plane or image correspondences) to a line
    class AbstractParameter
    {
    public:
	virtual ~AbstractParameter(void) {}; // To make this polymorphic we add dummy destructor
    };

    // Abstract model type for generic RANSAC model fitting
    template <int t_NumParams> /* Minimum number of parameters required to define this model*/
    class AbstractModel
    {
    protected:
	std::array<std::shared_ptr<AbstractParameter>, t_NumParams> m_MinModelParams;

        virtual VPFloat ComputeDistanceMeasure(std::shared_ptr<AbstractParameter> Param) = 0;

    public:
        virtual void Initialize(const std::vector<std::shared_ptr<AbstractParameter>> &InputParams) = 0;
        virtual std::pair<VPFloat, std::vector<std::shared_ptr<AbstractParameter>>> Evaluate(const std::vector<std::shared_ptr<AbstractParameter>> &EvaluateParams, VPFloat Threshold) = 0;

        virtual std::array<std::shared_ptr<AbstractParameter>, t_NumParams> GetModelParams(void) { return m_MinModelParams; };
    };
} // namespace GRANSAC
