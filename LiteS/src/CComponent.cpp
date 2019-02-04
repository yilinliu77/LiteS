#include "CComponent.h"

bool CComponent::extraInit() {
	return true;
}

void CComponent::extraAlgorithm() {}

void CComponent::continueExtraInit() {
	std::unique_lock<std::mutex> lock(this->m_shouldContinueMutex);
	this->m_shouldContinue = true;
	this->m_shouldContinueCV.notify_one();
}

void CComponent::waitForContinueSignal() {
	std::unique_lock<std::mutex> lock(this->m_shouldContinueMutex);
	while (!this->m_shouldContinue)
		this->m_shouldContinueCV.wait(lock);
	this->m_shouldContinue = false;
}
