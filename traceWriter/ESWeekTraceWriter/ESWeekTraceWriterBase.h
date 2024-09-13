#ifndef __ESWEEK_TRACE_WRITER_BASE__
#define __ESWEEK_TRACE_WRITER_BASE__

#include "../../traceReader/TraceLine.h"
#include "../GenericTraceWriter.h"

#include <fstream>
#include <string>

namespace ESWeek {
    class ESWeekTraceWriterBase : public NVM::GenericTraceWriter {
        // Functions
        public:
            virtual ~ESWeekTraceWriterBase() = default;

            /*
             * This function is the callhook for the simulation. It is entered
             * whenever the memory is about to get accessed. The passed
             * argument contains the most important information, e.g.
             * the the address or the data that is to be written.
             * A valid access has to return true.
             */
            virtual bool SetNextAccess(NVM::TraceLine *nextAccess) override;

            /*
             * This function is required to return a string with the path
             * of the trace file that the trace writer will write to.
             */
            virtual std::string GetTraceFile() override final;

            /*
             * This function is a callhook for NVMain to give you the path
             * to the trace file because it is configured in the config file
             * and also depends on the file hierarchy.
             */
            virtual void SetTraceFile(std::string file) override final;

        private:
            /*
             * This function is where to place the "Hello World" from
             * exercise 1 in the "ESWeekTraceWriter.cpp".
             */
            virtual void CreateTrace() = 0;

        // Variables
        protected:
            std::ofstream traceFile;

        private:
            bool flag = true;
            std::string traceFileAddress;
    };
}

#endif