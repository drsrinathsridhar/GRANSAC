#pragma once

#include "AbstractModel.hpp"

typedef std::array<GRANSAC::VPFloat, 2> Vector2VP;

class Point2D
    : public GRANSAC::AbstractParameter
{
public:
    Point2D(GRANSAC::VPFloat x, GRANSAC::VPFloat y)
    {
	m_Point2D[0] = x;
	m_Point2D[1] = y;
    };

    Vector2VP m_Point2D;
};

class Line2DModel
    : public GRANSAC::AbstractModel<2>
{
protected:
    // Parametric form
    GRANSAC::VPFloat m_a, m_b, m_c; // ax + by + c = 0
    GRANSAC::VPFloat m_DistDenominator; // = sqrt(a^2 + b^2). Stored for efficiency reasons

    // Another parametrization y = mx + d
    GRANSAC::VPFloat m_m; // Slope
    GRANSAC::VPFloat m_d; // Intercept

    virtual GRANSAC::VPFloat ComputeDistanceMeasure(std::shared_ptr<GRANSAC::AbstractParameter> Param) override
    {
	auto ExtPoint2D = std::dynamic_pointer_cast<Point2D>(Param);
	if(ExtPoint2D == nullptr)
	    throw std::runtime_error("Line2DModel::ComputeDistanceMeasure() - Passed parameter are not of type Point2D.");

	// Return distance between passed "point" and this line
	// http://mathworld.wolfram.com/Point-LineDistance2-Dimensional.html
	GRANSAC::VPFloat Numer = fabs(m_a * ExtPoint2D->m_Point2D[0] + m_b * ExtPoint2D->m_Point2D[1] + m_c);
	GRANSAC::VPFloat Dist = Numer / m_DistDenominator;

	// // Debug
	// std::cout << "Point: " << ExtPoint2D->m_Point2D[0] << ", " << ExtPoint2D->m_Point2D[1] << std::endl;
	// std::cout << "Line: " << m_a << " x + " << m_b << " y + "  << m_c << std::endl;
	// std::cout << "Distance: " << Dist << std::endl << std::endl;

	return Dist;
    };

public:
    Line2DModel(std::vector<std::shared_ptr<GRANSAC::AbstractParameter>> InputParams)
    {
	Initialize(InputParams);
    };

    virtual void Initialize(std::vector<std::shared_ptr<GRANSAC::AbstractParameter>> InputParams) override
    {
	if(InputParams.size() != 2)
	    throw std::runtime_error("Line2DModel - Number of input parameters does not match minimum number required for this model.");

	// Check for AbstractParamter types
	auto Point1 = std::dynamic_pointer_cast<Point2D>(InputParams[0]);
	auto Point2 = std::dynamic_pointer_cast<Point2D>(InputParams[1]);
	if(Point1 == nullptr || Point2 == nullptr)
	    throw std::runtime_error("Line2DModel - InputParams type mismatch. It is not a Point2D.");

	std::copy(InputParams.begin(), InputParams.end(), m_MinModelParams.begin());

	// Compute the line parameters
	m_m = (Point2->m_Point2D[1] - Point1->m_Point2D[1]) / (Point2->m_Point2D[0] - Point1->m_Point2D[0]); // Slope
	m_d = Point1->m_Point2D[1] - m_m * Point1->m_Point2D[0]; // Intercept
	// m_d = Point2->m_Point2D[1] - m_m * Point2->m_Point2D[0]; // Intercept - alternative should be the same as above

	// mx - y + d = 0
	m_a = m_m;
	m_b = -1.0;
	m_c = m_d;

	m_DistDenominator = sqrt(m_a * m_a + m_b * m_b); // Cache square root for efficiency
    };

    virtual std::pair<GRANSAC::VPFloat, std::vector<std::shared_ptr<GRANSAC::AbstractParameter>>> Evaluate(std::vector<std::shared_ptr<GRANSAC::AbstractParameter>> EvaluateParams, GRANSAC::VPFloat Threshold)
    {
	std::vector<std::shared_ptr<GRANSAC::AbstractParameter>> Inliers;
	int nTotalParams = EvaluateParams.size();
	int nInliers = 0;

	for(auto& Param : EvaluateParams)
	{
	    if(ComputeDistanceMeasure(Param) < Threshold)
	    {
		Inliers.push_back(Param);
		nInliers++;
	    }
	}

	GRANSAC::VPFloat InlierFraction = GRANSAC::VPFloat(nInliers) / GRANSAC::VPFloat(nTotalParams); // This is the inlier fraction

	return std::make_pair(InlierFraction, Inliers);
    };
};

