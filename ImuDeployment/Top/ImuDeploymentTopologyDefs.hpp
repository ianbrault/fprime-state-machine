// ======================================================================
// \title  ImuDeploymentTopologyDefs.hpp
// \brief required header file containing the required definitions for the topology autocoder
//
// ======================================================================
#ifndef IMUDEPLOYMENT_IMUDEPLOYMENTTOPOLOGYDEFS_HPP
#define IMUDEPLOYMENT_IMUDEPLOYMENTTOPOLOGYDEFS_HPP

// Subtopology PingEntries includes
#include "Svc/Subtopologies/CdhCore/PingEntries.hpp"
#include "Svc/Subtopologies/ComCcsds/PingEntries.hpp"
#include "Svc/Subtopologies/DataProducts/PingEntries.hpp"
#include "Svc/Subtopologies/FileHandling/PingEntries.hpp"

// SubtopologyTopologyDefs includes
#include "Svc/Subtopologies/CdhCore/SubtopologyTopologyDefs.hpp"
#include "Svc/Subtopologies/ComCcsds/SubtopologyTopologyDefs.hpp"
#include "Svc/Subtopologies/DataProducts/SubtopologyTopologyDefs.hpp"
#include "Svc/Subtopologies/FileHandling/SubtopologyTopologyDefs.hpp"

//ComCcsds Enum Includes
#include "Svc/Subtopologies/ComCcsds/Ports_ComPacketQueueEnumAc.hpp"
#include "Svc/Subtopologies/ComCcsds/Ports_ComBufferQueueEnumAc.hpp"

// Include autocoded FPP constants
#include "ImuDeployment/Top/FppConstantsAc.hpp"

/**
 * \brief required ping constants
 *
 * The topology autocoder requires a WARN and FATAL constant definition for each component that supports the health-ping
 * interface. These are expressed as enum constants placed in a namespace named for the component instance. These
 * are all placed in the PingEntries namespace.
 *
 * Each constant specifies how many missed pings are allowed before a WARNING_HI/FATAL event is triggered. In the
 * following example, the health component will emit a WARNING_HI event if the component instance cmdDisp does not
 * respond for 3 pings and will FATAL if responses are not received after a total of 5 pings.
 *
 * ```c++
 * namespace PingEntries {
 * namespace cmdDisp {
 *     enum { WARN = 3, FATAL = 5 };
 * }
 * }
 * ```
 */
namespace PingEntries {
    namespace ImuDeployment_rateGroup1 {enum { WARN = 3, FATAL = 5 };}
    namespace ImuDeployment_rateGroup2 {enum { WARN = 3, FATAL = 5 };}
    namespace ImuDeployment_rateGroup3 {enum { WARN = 3, FATAL = 5 };}
    namespace ImuDeployment_cmdSeq {enum { WARN = 3, FATAL = 5 };}
}  // namespace PingEntries

// Definitions are placed within a namespace named after the deployment
namespace ImuDeployment {

/**
 * \brief required type definition to carry state
 *
 * The topology autocoder requires an object that carries state with the name `ImuDeployment::TopologyState`. Only the type
 * definition is required by the autocoder and the contents of this object are otherwise opaque to the autocoder. The
 * contents are entirely up to the definition of the project. This deployment uses subtopologies.
 */
struct TopologyState {
    const char* hostname;   //!< Hostname for TCP communication
    U16 port;              //!< Port for TCP communication
    CdhCore::SubtopologyState cdhCore;           //!< Subtopology state for CdhCore
    ComCcsds::SubtopologyState comCcsds;         //!< Subtopology state for ComCcsds 
    DataProducts::SubtopologyState dataProducts; //!< Subtopology state for DataProducts
    FileHandling::SubtopologyState fileHandling; //!< Subtopology state for FileHandling
};

namespace PingEntries = ::PingEntries;
}  // namespace ImuDeployment
#endif
