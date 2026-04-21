#pragma once

#include "EqFparser.hpp"
#include "VectorNav.hpp"
#include "EqFalgo.hpp"
// RAW DATA FILE writes to a file in log/

//void logRaw();

// FILTERED DATA FILE writes to another file in log/

// takes in of type EqFOutput
// writes as csv file with this header

// timestamp(s), orientation(n/a, 9 entries),pos(m, 3 entries),vel(m/s, 3 entries),biasomega(rad/s, 3 entries),biasvirtual(m/s, 3 entries), biasacc(m/s², 3 entries), 
// trace of [0-2] orientation uncertainty, trace of [3-5] pos uncertainty, trace of [6-8] vel uncertainty, trace of [9-11] bias omega uncertainty, trace of [12-14] bias vel uncertainty, trace of [15-17] bias acc uncertainty 

//void logFiltered(EqFOutput output);


void printRaw(const EqFparserResult& result);
void printMeasurements(const EqFparserResult& result, const vectornavData& data);
void printVNEstimate(const vectornavData& data);
void printTGEqFEstimate(const EqFOutput& output, const vectornavData& data, const char* title);
void logMeasurements(const EqFparserResult& result, const vectornavData& data);
void logVNEstimate(const vectornavData& data);
void logTGEqFEstimate(const EqFOutput& output, const vectornavData& data, const char* csvPath);
