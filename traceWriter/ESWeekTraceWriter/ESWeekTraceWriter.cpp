#include "ESWeekTraceWriter.h"

using namespace ESWeek;

/*
 * ESWeek-Tutorial Exercise 1:
 * Add the definition of the function "CreateTrace()".
 * In this function, put "Hello World" to the resulting
 * trace file. The trace file can be accessed via the
 * variable "traceFile". You can write to the trace file
 * with the standard operator <<. E.g.:
 * traceFile << "My" << " first " << "text" << std::endl;
 */

/*
 * ESWeek-Tutorial Exercise 2:
 * Add the definition of the function "SetNextAccess(...)".
 * In this function, you want to fill you trace with interesting
 * data. For this, you want to put the address of the access and
 * its last byte of data to the trace file, like you did for
 * exercise 1, in case it is a writing access. For this you need
 * the following information.
 * - Address: You can retrieve the address by calling
 *            "GetAddress().GetPhysicalAddress()" on the TraceLine
 *            argument.
 * - Data: You can get the access' data bock by calling "GetData()"
 *         on the TraceLine. However, all data in this data block
 *         is separated into single bytes, including the last byte
 *         you are looking for. A byte can be retrieved from the
 *         data block, by calling "GetByte(index)" on it. The index
 *         can be calculated by getting the size of the data block
 *         by calling "GetSize()" on the data block and substracting
 *         one.
 * - AccessType. You can get the type of access by calling "GetOperation()"
 *               on the TraceLine. The access type you are interested in is
 *               "NVM::WRITE".
 * If you are not familiar with the used variable types, just use "auto" as
 * a type.
 * 
 * With these information available to you, do the following:
 * 1) Check if the access is a writing access. If not, return "true".
 * 2) Else, put the data you've just collected to the trace file
 *    the same way you did for exercise 1. We propose the format
 *    "Address | DataByte".
 *    Note that the data byte will be in an 8 bit format. The resulting
 *    ASCII trace file can not interpret this format and you will get
 *    weird symbols. To avoid this, cast the data type to an unsigned int
 *    before you put it to the trace file. E.g.:
 *    traceFile << static_cast<unsigned int>(data) << std::endl;
 * 3) Return "true" at the end of thefunction.
 */