//------------------------------------------------------------------------------------------------------------------------------
// CustomUtils.h
//------------------------------------------------------------------------------------------------------------------------------
#pragma once
#include <stdint.h>
#include <string>

//------------------------------------------------------------------------------------------------------------------------------
double GetCurrentTimeSeconds();

uint64_t GetCurrentTimeHPC();
double GetHPCToSeconds(uint64_t hpc);

std::string GetDateTime();

float			Clamp(float x, float minClamp, float maxClamp);