#ifndef __ESWEEK_TRACE_WRITER__
#define __ESWEEK_TRACE_WRITER__

#include "ESWeekTraceWriterBase.h"

namespace ESWeek {
    class ESWeekTraceWriter : public ESWeekTraceWriterBase {
        // Functions
        public:
            virtual ~ESWeekTraceWriter() = default;

        private:
            /*
             * ESWeek-Tutorial Exercise 1:
             * Add the function "CreateTrace()" from the base class
             * "ESWeekTraceWriterBase". Remember to make it virtual,
             * i.e., place the keyword "virtual" in front of the
             * function's declaration. Its return type is void
             * and it does not have any arguments.
             */
            virtual void CreateTrace();

            /*
             * ESWeek-Tutorial Exercise 2:
             * Add the function "SetNextAccess(...)" from the base class
             * "ESWeekTraceWriterBase". Remember to make it virtual,
             * i.e., place the keyword "virtual" in front of the
             * function's declaration. Its return type is void
             * and its parameter is a NVM::TraceLine pointer named
             * "nextAccess".
             */
            virtual bool SetNextAccess(NVM::TraceLine* nextAccess) override;
    };
}

#endif