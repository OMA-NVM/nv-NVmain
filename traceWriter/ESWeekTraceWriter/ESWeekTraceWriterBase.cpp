#include "ESWeekTraceWriterBase.h"

#include <stdexcept>

using namespace ESWeek;

// Public
bool ESWeekTraceWriterBase::SetNextAccess(NVM::TraceLine* nextAccess) {
    if(flag) {
        CreateTrace();
        flag = false;
    }

    return true;
}

std::string ESWeekTraceWriterBase::GetTraceFile() {
    return traceFileAddress;
}

void ESWeekTraceWriterBase::SetTraceFile(std::string file) {
    traceFileAddress = file;

    traceFile.open(traceFileAddress.c_str());

    if(!traceFile.is_open()) {
        throw std::exception();
    }
}