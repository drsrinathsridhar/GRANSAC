#pragma once

#include <iostream>
#include <cmath>
#include <string>
#include <random>
#include <memory>
#include <algorithm>
#include <vector>
#include <omp.h>

#include "AbstractModel.hpp"

namespace GRANSAC
{
    // T - AbstractModel
    template <class T, int t_NumParams>
    class RANSAC
    {
    private:
	std::vector<std::shared_ptr<AbstractParameter>> m_Data; // All the data

	std::vector<std::shared_ptr<T>> m_SampledModels; // Vector of all sampled models
	std::shared_ptr<T> m_BestModel; // Pointer to the best model, valid only after Estimate() is called
	std::vector<std::shared_ptr<AbstractParameter>> m_BestInliers;

	int m_MaxIterations; // Number of iterations before termination
	VPFloat m_Threshold; // The threshold for computing model consensus
	VPFloat m_BestModelScore; // The score of the best model
	int m_BestModelIdx;

	std::vector<std::mt19937> m_RandEngines; // Mersenne twister high quality RNG that support *OpenMP* multi-threading

    public:
	RANSAC(void)
	{
	    int nThreads = std::max(1, omp_get_max_threads());
	    std::cout << "[ INFO ]: Maximum usable threads: " << nThreads << std::endl;
	    for(int i = 0; i < nThreads; ++i)
	    {
		std::random_device SeedDevice;
		m_RandEngines.push_back(std::mt19937(SeedDevice()));
	    }

	    Reset();
	};

	virtual ~RANSAC(void) {};

	void Reset(void)
	{
	    // Clear sampled models, etc. and prepare for next call. Reset RANSAC estimator state
	    m_Data.clear();
	    m_SampledModels.clear();

	    m_BestModelIdx = -1;
	    m_BestModelScore = 0.0;
	};

	void Initialize(VPFloat Threshold, int MaxIterations = 1000)
	{
	    m_Threshold = Threshold;
	    m_MaxIterations = MaxIterations;
	};

	std::shared_ptr<T> GetBestModel(void) { return m_BestModel; };
	const std::vector<std::shared_ptr<AbstractParameter>>& GetBestInliers(void) { return m_BestInliers; };

	bool Estimate(std::vector<std::shared_ptr<AbstractParameter>> Data)
	{
	    if(Data.size() <= t_NumParams)
	    {
		std::cout << "[ WARN ]: RANSAC - Number of data points is too less. Not doing anything." << std::endl;
		return false;
	    }

	    m_Data = Data;
	    int DataSize = m_Data.size();
	    std::uniform_int_distribution<int> UniDist(0, int(DataSize-1)); // Both inclusive

	    std::vector<VPFloat> InlierFractionAccum(m_MaxIterations);
	    std::vector<std::vector<std::shared_ptr<AbstractParameter>>> InliersAccum(m_MaxIterations);
	    m_SampledModels.resize(m_MaxIterations);
	
	    int nThreads = std::max(1, omp_get_max_threads());
	    omp_set_dynamic(0); // Explicitly disable dynamic teams
	    omp_set_num_threads(nThreads);
#pragma omp parallel for
	    for(int i = 0; i < m_MaxIterations; ++i)
	    {
		// Select t_NumParams random samples
		std::vector<std::shared_ptr<AbstractParameter>> RandomSamples(t_NumParams);
		std::vector<std::shared_ptr<AbstractParameter>> RemainderSamples = m_Data; // Without the chosen random samples

		std::shuffle(RemainderSamples.begin(), RemainderSamples.end(), m_RandEngines[omp_get_thread_num()]); // To avoid picking the same element more than once
		std::copy(RemainderSamples.begin(), RemainderSamples.begin() + t_NumParams, RandomSamples.begin());
		RemainderSamples.erase(RemainderSamples.begin(), RemainderSamples.begin() + t_NumParams);

		std::shared_ptr<T> RandomModel = std::make_shared<T>(RandomSamples);

		// Check if the sampled model is the best so far
		std::pair<VPFloat, std::vector<std::shared_ptr<AbstractParameter>>> EvalPair = RandomModel->Evaluate(RemainderSamples, m_Threshold);
		InlierFractionAccum[i] = EvalPair.first;
		InliersAccum[i] = EvalPair.second;

		// Push back into history. Could be removed later
		m_SampledModels[i] = RandomModel;
	    }

	    for(int i = 0; i < m_MaxIterations; ++i)
	    {
		if(InlierFractionAccum[i] > m_BestModelScore)
		{
		    m_BestModelScore = InlierFractionAccum[i];
		    m_BestModelIdx = m_SampledModels.size() - 1; 
		    m_BestModel = m_SampledModels[i];
		    m_BestInliers = InliersAccum[i];
		}
	    }

	    // std::cout << "BestInlierFraction: " << m_BestModelScore << std::endl;

	    Reset();

	    return true;
	};
    };
} // namespace GRANSAC
