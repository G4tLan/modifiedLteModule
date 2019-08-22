/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2011 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
 * Copyright (c) 2015, University of Padova, Dep. of Information Engineering, SIGNET lab.
 * Copyright (c) 2018 Fraunhofer ESK
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Marco Miozzo <marco.miozzo@cttc.es>
 *         Nicola Baldo  <nbaldo@cttc.es>
 * Modified by:
 *          Danilo Abrignani <danilo.abrignani@unibo.it> (Carrier Aggregation - GSoC 2015)
 *          Biljana Bojovic <biljana.bojovic@cttc.es> (Carrier Aggregation)
 *
 * Modified by Michele Polese <michele.polese@gmail.com>
 *    (support for RACH realistic model)
 *
 * Modified by Vignesh Babu <ns3-dev@esk.fraunhofer.de>
 *    (support for uplink synchronization;
 *    integrated the RACH realistic model and RRC_CONNECTED->RRC_IDLE
 *    state transition (taken from Lena-plus(work of Michele Polese)) and also enhanced both the modules)
 */


#include <ns3/log.h>
#include <ns3/pointer.h>
#include <ns3/packet.h>
#include <ns3/simulator.h>

#include "lte-amc.h"
#include "lte-control-messages.h"
#include "lte-enb-net-device.h"
#include "lte-ue-net-device.h"

#include <ns3/lte-enb-mac.h>
#include <ns3/lte-radio-bearer-tag.h>
#include <ns3/lte-ue-phy.h>

#include "ns3/lte-mac-sap.h"
#include "ns3/lte-enb-cmac-sap.h"
#include <ns3/lte-common.h>
#include <ns3/lte-vendor-specific-parameters.h>
#include "ns3/vector.h"
#include <limits>


namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("LteEnbMac");

NS_OBJECT_ENSURE_REGISTERED (LteEnbMac);



// //////////////////////////////////////
// member SAP forwarders
// //////////////////////////////////////


/// EnbMacMemberLteEnbCmacSapProvider class
class EnbMacMemberLteEnbCmacSapProvider : public LteEnbCmacSapProvider
{
public:
  /**
   * Constructor
   *
   * \param mac the MAC
   */
  EnbMacMemberLteEnbCmacSapProvider (LteEnbMac* mac);

  // inherited from LteEnbCmacSapProvider
  virtual void ConfigureMac (uint8_t ulBandwidth, uint8_t dlBandwidth);
  virtual void AddUe (uint16_t rnti);
  virtual void RemoveUe (uint16_t rnti);
  virtual void AddLc (LcInfo lcinfo, LteMacSapUser* msu);
  virtual void ReconfigureLc (LcInfo lcinfo);
  virtual void ReleaseLc (uint16_t rnti, uint8_t lcid);
  virtual void ReleaseLc (uint16_t rnti);
  virtual void UeUpdateConfigurationReq (UeConfig params);
  virtual RachConfig GetRachConfig ();
  virtual AllocateNcRaPreambleReturnValue AllocateNcRaPreamble (uint16_t rnti);
  virtual void SetRntiImsiMap (uint16_t rnti, uint64_t imsi);
  virtual void StartTimeAlignmentTimer (Time timeAlignmentTimer, uint16_t rnti);
  virtual void SetMsg4Ready (uint16_t rnti);


private:
  LteEnbMac* m_mac; ///< the MAC
};


EnbMacMemberLteEnbCmacSapProvider::EnbMacMemberLteEnbCmacSapProvider (LteEnbMac* mac)
  : m_mac (mac)
{
}

void
EnbMacMemberLteEnbCmacSapProvider::ConfigureMac (uint8_t ulBandwidth, uint8_t dlBandwidth)
{
  m_mac->DoConfigureMac (ulBandwidth, dlBandwidth);
}

void
EnbMacMemberLteEnbCmacSapProvider::AddUe (uint16_t rnti)
{
  m_mac->DoAddUe (rnti);
}

void
EnbMacMemberLteEnbCmacSapProvider::RemoveUe (uint16_t rnti)
{
  m_mac->DoRemoveUe (rnti);
}

void
EnbMacMemberLteEnbCmacSapProvider::AddLc (LcInfo lcinfo, LteMacSapUser* msu)
{
  m_mac->DoAddLc (lcinfo, msu);
}

void
EnbMacMemberLteEnbCmacSapProvider::ReconfigureLc (LcInfo lcinfo)
{
  m_mac->DoReconfigureLc (lcinfo);
}

void
EnbMacMemberLteEnbCmacSapProvider::ReleaseLc (uint16_t rnti)
{
  m_mac->DoReleaseLc (rnti);
}

void
EnbMacMemberLteEnbCmacSapProvider::ReleaseLc (uint16_t rnti, uint8_t lcid)
{
  m_mac->DoReleaseLc (rnti, lcid);
}

void
EnbMacMemberLteEnbCmacSapProvider::UeUpdateConfigurationReq (UeConfig params)
{
  m_mac->DoUeUpdateConfigurationReq (params);
}

LteEnbCmacSapProvider::RachConfig
EnbMacMemberLteEnbCmacSapProvider::GetRachConfig ()
{
  return m_mac->DoGetRachConfig ();
}

LteEnbCmacSapProvider::AllocateNcRaPreambleReturnValue
EnbMacMemberLteEnbCmacSapProvider::AllocateNcRaPreamble (uint16_t rnti)
{
  return m_mac->DoAllocateNcRaPreamble (rnti);
}

void
EnbMacMemberLteEnbCmacSapProvider::SetRntiImsiMap (uint16_t rnti, uint64_t imsi)
{
  m_mac->DoSetRntiImsiMap (rnti, imsi);
}

void
EnbMacMemberLteEnbCmacSapProvider::StartTimeAlignmentTimer (Time timeAlignmentTimer, uint16_t rnti)
{
  m_mac->DoStartTimeAlignmentTimer (timeAlignmentTimer, rnti);
}

void
EnbMacMemberLteEnbCmacSapProvider::SetMsg4Ready (uint16_t rnti)
{
  m_mac->DoSetMsg4Ready (rnti);
}

/// EnbMacMemberFfMacSchedSapUser class
class EnbMacMemberFfMacSchedSapUser : public FfMacSchedSapUser
{
public:
  /**
   * Constructor
   *
   * \param mac the MAC
   */
  EnbMacMemberFfMacSchedSapUser (LteEnbMac* mac);


  virtual void SchedDlConfigInd (const struct SchedDlConfigIndParameters& params);
  virtual void SchedUlConfigInd (const struct SchedUlConfigIndParameters& params);
private:
  LteEnbMac* m_mac; ///< the MAC
};


EnbMacMemberFfMacSchedSapUser::EnbMacMemberFfMacSchedSapUser (LteEnbMac* mac)
  : m_mac (mac)
{
}


void
EnbMacMemberFfMacSchedSapUser::SchedDlConfigInd (const struct SchedDlConfigIndParameters& params)
{
  m_mac->DoSchedDlConfigInd (params);
}



void
EnbMacMemberFfMacSchedSapUser::SchedUlConfigInd (const struct SchedUlConfigIndParameters& params)
{
  m_mac->DoSchedUlConfigInd (params);
}


/// EnbMacMemberFfMacCschedSapUser class
class EnbMacMemberFfMacCschedSapUser : public FfMacCschedSapUser
{
public:
  /**
   * Constructor
   *
   * \param mac the MAC
   */
  EnbMacMemberFfMacCschedSapUser (LteEnbMac* mac);

  virtual void CschedCellConfigCnf (const struct CschedCellConfigCnfParameters& params);
  virtual void CschedUeConfigCnf (const struct CschedUeConfigCnfParameters& params);
  virtual void CschedLcConfigCnf (const struct CschedLcConfigCnfParameters& params);
  virtual void CschedLcReleaseCnf (const struct CschedLcReleaseCnfParameters& params);
  virtual void CschedUeReleaseCnf (const struct CschedUeReleaseCnfParameters& params);
  virtual void CschedUeConfigUpdateInd (const struct CschedUeConfigUpdateIndParameters& params);
  virtual void CschedCellConfigUpdateInd (const struct CschedCellConfigUpdateIndParameters& params);

private:
  LteEnbMac* m_mac; ///< the MAC
};


EnbMacMemberFfMacCschedSapUser::EnbMacMemberFfMacCschedSapUser (LteEnbMac* mac)
  : m_mac (mac)
{
}

void
EnbMacMemberFfMacCschedSapUser::CschedCellConfigCnf (const struct CschedCellConfigCnfParameters& params)
{
  m_mac->DoCschedCellConfigCnf (params);
}

void
EnbMacMemberFfMacCschedSapUser::CschedUeConfigCnf (const struct CschedUeConfigCnfParameters& params)
{
  m_mac->DoCschedUeConfigCnf (params);
}

void
EnbMacMemberFfMacCschedSapUser::CschedLcConfigCnf (const struct CschedLcConfigCnfParameters& params)
{
  m_mac->DoCschedLcConfigCnf (params);
}

void
EnbMacMemberFfMacCschedSapUser::CschedLcReleaseCnf (const struct CschedLcReleaseCnfParameters& params)
{
  m_mac->DoCschedLcReleaseCnf (params);
}

void
EnbMacMemberFfMacCschedSapUser::CschedUeReleaseCnf (const struct CschedUeReleaseCnfParameters& params)
{
  m_mac->DoCschedUeReleaseCnf (params);
}

void
EnbMacMemberFfMacCschedSapUser::CschedUeConfigUpdateInd (const struct CschedUeConfigUpdateIndParameters& params)
{
  m_mac->DoCschedUeConfigUpdateInd (params);
}

void
EnbMacMemberFfMacCschedSapUser::CschedCellConfigUpdateInd (const struct CschedCellConfigUpdateIndParameters& params)
{
  m_mac->DoCschedCellConfigUpdateInd (params);
}



/// ---------- PHY-SAP
class EnbMacMemberLteEnbPhySapUser : public LteEnbPhySapUser
{
public:
  /**
   * Constructor
   *
   * \param mac the MAC
   */
  EnbMacMemberLteEnbPhySapUser (LteEnbMac* mac);

  // inherited from LteEnbPhySapUser
  virtual void ReceivePhyPdu (Ptr<Packet> p);
  virtual void SubframeIndication (uint32_t frameNo, uint32_t subframeNo);
  virtual void ReceiveLteControlMessage (Ptr<LteControlMessage> msg);
  virtual void ReceiveRachPreamble (Ptr<RachPreambleLteControlMessage> msg);
  virtual void UlCqiReport (FfMacSchedSapProvider::SchedUlCqiInfoReqParameters ulcqi);
  virtual void UlInfoListElementHarqFeeback (UlInfoListElement_s params);
  virtual void DlInfoListElementHarqFeeback (DlInfoListElement_s params);

private:
  LteEnbMac* m_mac; ///< the MAC
};

EnbMacMemberLteEnbPhySapUser::EnbMacMemberLteEnbPhySapUser (LteEnbMac* mac) : m_mac (mac)
{
}


void
EnbMacMemberLteEnbPhySapUser::ReceivePhyPdu (Ptr<Packet> p)
{
  m_mac->DoReceivePhyPdu (p);
}

void
EnbMacMemberLteEnbPhySapUser::SubframeIndication (uint32_t frameNo, uint32_t subframeNo)
{
  m_mac->DoSubframeIndication (frameNo, subframeNo);
}

void
EnbMacMemberLteEnbPhySapUser::ReceiveLteControlMessage (Ptr<LteControlMessage> msg)
{
  m_mac->DoReceiveLteControlMessage (msg);
}

void
EnbMacMemberLteEnbPhySapUser::ReceiveRachPreamble (Ptr<RachPreambleLteControlMessage> msg)
{
  m_mac->DoReceiveRachPreamble (msg);
}

void
EnbMacMemberLteEnbPhySapUser::UlCqiReport (FfMacSchedSapProvider::SchedUlCqiInfoReqParameters ulcqi)
{
  m_mac->DoUlCqiReport (ulcqi);
}

void
EnbMacMemberLteEnbPhySapUser::UlInfoListElementHarqFeeback (UlInfoListElement_s params)
{
  m_mac->DoUlInfoListElementHarqFeeback (params);
}

void
EnbMacMemberLteEnbPhySapUser::DlInfoListElementHarqFeeback (DlInfoListElement_s params)
{
  m_mac->DoDlInfoListElementHarqFeeback (params);
}


// //////////////////////////////////////
// generic LteEnbMac methods
// //////////////////////////////////////

// constant used to evaluate if there is a RACH preamble collision is detected
static const double cpConstant = (299792458) / (2 * 1.08e6);

TypeId
LteEnbMac::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::LteEnbMac")
    .SetParent<Object> ()
    .SetGroupName ("Lte")
    .AddConstructor<LteEnbMac> ()
    .AddAttribute ("PreambleInitialReceivedTargetPower",
                   "The target power of initial RACH preamble in dBm",
                   IntegerValue ((-110)), // dBm
                   MakeIntegerAccessor (&LteEnbMac::m_preambleInitialReceivedTargetPower),
                   MakeIntegerChecker<int8_t> (-120, -90))
    .AddAttribute ("PowerRampingStep",
                   "The step by which the tx power of rach preamble is increased after each failure in dBm",
                   UintegerValue (2), // dBm
                   MakeUintegerAccessor (&LteEnbMac::m_powerRampingStep),
                   MakeUintegerChecker<int8_t> (0, 6))
    .AddAttribute ("NumberOfRaPreambles",
                   "how many random access preambles are available for the contention based RACH process",
                   UintegerValue (52),
                   MakeUintegerAccessor (&LteEnbMac::m_numberOfRaPreambles),
                   MakeUintegerChecker<uint8_t> (4, 64)) //only defined preamble values should be assigned otherwise serialization error occurs in rrc header(SerializeRachConfigCommon)
    .AddAttribute ("PreambleTransMax",
                   "Maximum number of random access preamble transmissions",
                   UintegerValue (10),
                   MakeUintegerAccessor (&LteEnbMac::m_preambleTransMax),
                   MakeUintegerChecker<uint8_t> (3, 200))
    .AddAttribute ("RaResponseWindowSize",
                   "length of the window (in TTIs) for the reception of the random access response (RAR); the resulting RAR timeout is this value + 3 ms",
                   UintegerValue (4),
                   MakeUintegerAccessor (&LteEnbMac::m_raResponseWindowSize),
                   MakeUintegerChecker<uint8_t> (2, 10))
    .AddAttribute ("ContentionResolutionTimer",
                   "length of the window (in TTIs) for the reception of the contention resolution msg",
                   UintegerValue (32), // check 3GPP TS 36.331 rach config common
                   MakeUintegerAccessor (&LteEnbMac::m_contentionResolutionTimer),
                   MakeUintegerChecker<uint8_t> (8, 64))
    .AddAttribute ("PRachConfigurationIndex",
                   "Configuration index of PRACH as in 3GPP TS 36.211 5.7.1",
                   UintegerValue (1),
                   MakeUintegerAccessor (&LteEnbMac::m_pRachConfigurationIndex),
                   MakeUintegerChecker<uint8_t> (0, 63))
    .AddAttribute ("BackoffIndicator",
                   "Backoff indicator for rach procedure as in 3GPP TS 36.321 Table 7.2-1",
                   UintegerValue (0),
                   MakeUintegerAccessor (&LteEnbMac::m_backoffIndicator),
                   MakeUintegerChecker<uint8_t> (0, 15))
    .AddTraceSource ("DlScheduling",
                     "Information regarding DL scheduling.",
                     MakeTraceSourceAccessor (&LteEnbMac::m_dlScheduling),
                     "ns3::LteEnbMac::DlSchedulingTracedCallback")
    .AddTraceSource ("UlScheduling",
                     "Information regarding UL scheduling.",
                     MakeTraceSourceAccessor (&LteEnbMac::m_ulScheduling),
                     "ns3::LteEnbMac::UlSchedulingTracedCallback")
    .AddAttribute ("ComponentCarrierId",
                   "ComponentCarrier Id, needed to reply on the appropriate sap.",
                   UintegerValue (0),
                   MakeUintegerAccessor (&LteEnbMac::m_componentCarrierId),
                   MakeUintegerChecker<uint8_t> (0,4))
    .AddTraceSource ("RaPreambleReceived",
                     "Triggered when a ra preamble is received",
                     MakeTraceSourceAccessor (&LteEnbMac::m_receiveRachPreambleTrace),
                     "nse::LteEnbMac::RachPreambleReceivedTracedCallback")

    .AddAttribute ("TxRxInactivityTimeoutDuration",
                   "This attribute gives the time duration of the TxRxInactivityTimer. If data exchange between UE "
                   "and eNodeB occurs, then inactivity timer is started/restarted. The timing advance command is sent "
                   "to UE only when this timer is active i.e data transfer is still ongoing. Once this timer expires, "
                   "timing advance is not sent to the UE. .Similar to DRX Inactivity Timer (Value:1-2560ms)",
                   TimeValue (MilliSeconds (100)),
                   MakeTimeAccessor (&LteEnbMac::m_txRxInactivityTimeoutDuration),
                   MakeTimeChecker ())

    .AddAttribute ("DynamicTimerConfiguration",
                   "If true, uplink timeAlignmentTimer value is based on mobility of UE"
                   "If false, uplink timeAlignmentTimer value is fixed.",
                   BooleanValue (false),
                   MakeBooleanAccessor (&LteEnbMac::m_dynamicTimerConfigured),
                   MakeBooleanChecker ())
  ;

  return tid;
}


LteEnbMac::LteEnbMac ()
  : m_ccmMacSapUser (0)
{
  NS_LOG_FUNCTION (this);
  m_macSapProvider = new EnbMacMemberLteMacSapProvider<LteEnbMac> (this);
  m_cmacSapProvider = new EnbMacMemberLteEnbCmacSapProvider (this);
  m_schedSapUser = new EnbMacMemberFfMacSchedSapUser (this);
  m_cschedSapUser = new EnbMacMemberFfMacCschedSapUser (this);
  m_enbPhySapUser = new EnbMacMemberLteEnbPhySapUser (this);
  m_ccmMacSapProvider = new MemberLteCcmMacSapProvider<LteEnbMac> (this);
}


LteEnbMac::~LteEnbMac ()
{
  NS_LOG_FUNCTION (this);
}

void
LteEnbMac::DoDispose ()
{
  NS_LOG_FUNCTION (this);
  m_dlCqiReceived.clear ();
  m_ulCqiReceived.clear ();
  m_ulCeReceived.clear ();
  m_dlInfoListReceived.clear ();
  m_ulInfoListReceived.clear ();
  m_miDlHarqProcessesPackets.clear ();
  delete m_macSapProvider;
  delete m_cmacSapProvider;
  delete m_schedSapUser;
  delete m_cschedSapUser;
  delete m_enbPhySapUser;
  delete m_ccmMacSapProvider;
}

void
LteEnbMac::SetComponentCarrierId (uint8_t index)
{
  m_componentCarrierId = index;
}

void
LteEnbMac::SetFfMacSchedSapProvider (FfMacSchedSapProvider* s)
{
  m_schedSapProvider = s;
}

FfMacSchedSapUser*
LteEnbMac::GetFfMacSchedSapUser (void)
{
  return m_schedSapUser;
}

void
LteEnbMac::SetFfMacCschedSapProvider (FfMacCschedSapProvider* s)
{
  m_cschedSapProvider = s;
}

FfMacCschedSapUser*
LteEnbMac::GetFfMacCschedSapUser (void)
{
  return m_cschedSapUser;
}



void
LteEnbMac::SetLteMacSapUser (LteMacSapUser* s)
{
  m_macSapUser = s;
}

LteMacSapProvider*
LteEnbMac::GetLteMacSapProvider (void)
{
  return m_macSapProvider;
}

void
LteEnbMac::SetLteEnbCmacSapUser (LteEnbCmacSapUser* s)
{
  m_cmacSapUser = s;
}

LteEnbCmacSapProvider*
LteEnbMac::GetLteEnbCmacSapProvider (void)
{
  return m_cmacSapProvider;
}

void
LteEnbMac::SetLteEnbPhySapProvider (LteEnbPhySapProvider* s)
{
  m_enbPhySapProvider = s;
}


LteEnbPhySapUser*
LteEnbMac::GetLteEnbPhySapUser ()
{
  return m_enbPhySapUser;
}

void
LteEnbMac::SetLteCcmMacSapUser (LteCcmMacSapUser* s)
{
  m_ccmMacSapUser = s;
}


LteCcmMacSapProvider*
LteEnbMac::GetLteCcmMacSapProvider ()
{
  return m_ccmMacSapProvider;
}

void
LteEnbMac::SetPrachMode (bool ideal)
{ // m_realPrach is true --> use real PRACH
  m_realPrach = ideal;
}

bool
LteEnbMac::GetPrachMode () const
{
  return m_realPrach;
}

void
LteEnbMac::DoSubframeIndication (uint32_t frameNo, uint32_t subframeNo) //preamble collision detection
{
  NS_LOG_FUNCTION (this << " EnbMac - frame " << frameNo << " subframe " << subframeNo);

  // Store current frame / subframe number
  m_frameNo = frameNo;
  m_subframeNo = subframeNo;


  // --- DOWNLINK ---
  // Send Dl-CQI info to the scheduler
  if (m_dlCqiReceived.size () > 0)
    {
      FfMacSchedSapProvider::SchedDlCqiInfoReqParameters dlcqiInfoReq;
      dlcqiInfoReq.m_sfnSf = ((0x3FF & frameNo) << 4) | (0xF & subframeNo);
      dlcqiInfoReq.m_cqiList.insert (dlcqiInfoReq.m_cqiList.begin (), m_dlCqiReceived.begin (), m_dlCqiReceived.end ());
      m_dlCqiReceived.erase (m_dlCqiReceived.begin (), m_dlCqiReceived.end ());
      m_schedSapProvider->SchedDlCqiInfoReq (dlcqiInfoReq);
    }

  if (!m_receivedRachPreambleCount.empty ())
    {
      // process received RACH preambles and notify the scheduler
      FfMacSchedSapProvider::SchedDlRachInfoReqParameters rachInfoReqParams;
      NS_ASSERT (subframeNo > 0 && subframeNo <= 10); // subframe in 1..10
      std::map<uint8_t, Vector>::const_iterator it = m_receivedRachPreambleCount.begin ();
      while (it != m_receivedRachPreambleCount.end ())
        {
          NS_LOG_INFO (this << " lteEnbMac " << " preambleId " << (uint32_t) it->first << ": " << it->second << " received");
          // get the number of msg with this rapId
          uint32_t numPossibleCollPreambles = m_receivedRachPreambleCount.count (it->first);
          std::pair<RapIdPositionMap_t::iterator, RapIdPositionMap_t::iterator> iterPair = m_receivedRachPreambleCount.equal_range (it->first);
          NS_ASSERT (numPossibleCollPreambles != 0);
          bool collision = false;
          if (numPossibleCollPreambles > 1)
            {
              // collision may be detected or not
              if (m_realPrach)
                {
                  // evaluate the possible collision
                  // get the greatest distance
                  std::list<Vector> pointList;
                  for (RapIdPositionMap_t::iterator iterLoop = iterPair.first; iterLoop != iterPair.second; ++iterLoop)
                    {
                      pointList.push_back (iterLoop->second);
                    }

                  collision = EvaluateCollisionProbability (pointList);
                }
              else
                {
                  collision = true;
                }

              if (collision)
                {
                  NS_LOG_DEBUG ("preambleId " << (uint32_t) it->first << ": collision detected");
                }
              else
                {
                  NS_LOG_DEBUG ("preambleId " << (uint32_t) it->first << ": undetected collision");
                }
            }

          // in case of collision we assume that no preamble is
          // successfully received, hence no RAR is sent
          if (!collision)
            {
              uint16_t rnti;
              std::map<uint8_t, NcRaPreambleInfo>::iterator jt = m_allocatedNcRaPreambleMap.find (it->first);
              if (jt != m_allocatedNcRaPreambleMap.end ())
                {
                  rnti = jt->second.rnti;
                  NS_LOG_INFO ("preambleId previously allocated for NC based RA, RNTI =" << (uint32_t) rnti << ", sending RAR");

                  //if preamble received is for PDCCH order based RACH, set the flag to true
                  std::map<uint16_t, UeInfo>::iterator uit = m_ueInfoMap.find (rnti);
                  if (uit != m_ueInfoMap.end () && uit->second.isUplinkOutOfSync)
                    {
                      uit->second.isNonContentionBasedRandomAccess = true;
                    }

                }
              else
                {
                  rnti = m_cmacSapUser->AllocateTemporaryCellRnti (); // this function returns the first available rnti of the cell
                 //if RNTI equals 0, then it means the UE context for this preamble was not created due to lack of SRS
                  if(rnti==0)
                    {
                      //RAR is not sent for this premable and execution continues to check for other received preambles
                      it = iterPair.second;
                      continue;
                    }
                  NS_LOG_INFO ("preambleId " << (uint32_t) it->first << ": allocated T-C-RNTI " << (uint32_t) rnti << ", sending RAR");
                }

              RachListElement_s rachLe;
              rachLe.m_rnti = rnti;
              rachLe.m_estimatedSize = 144; // to be confirmed
              rachInfoReqParams.m_rachList.push_back (rachLe);
              m_rapIdRntiMap.insert (std::pair <uint16_t, uint32_t> (rnti, it->first)); // it->first is the rapId
            }
          it = iterPair.second;
        }
      m_schedSapProvider->SchedDlRachInfoReq (rachInfoReqParams);
      m_receivedRachPreambleCount.clear ();
    }
  // Get downlink transmission opportunities
  uint32_t dlSchedFrameNo = m_frameNo;
  uint32_t dlSchedSubframeNo = m_subframeNo;
  //   NS_LOG_DEBUG (this << " sfn " << frameNo << " sbfn " << subframeNo);
  if (dlSchedSubframeNo + m_macChTtiDelay > 10)
    {
      dlSchedFrameNo++;
      dlSchedSubframeNo = (dlSchedSubframeNo + m_macChTtiDelay) % 10;
    }
  else
    {
      dlSchedSubframeNo = dlSchedSubframeNo + m_macChTtiDelay;
    }
  FfMacSchedSapProvider::SchedDlTriggerReqParameters dlparams;
  dlparams.m_sfnSf = ((0x3FF & dlSchedFrameNo) << 4) | (0xF & dlSchedSubframeNo);

  // Forward DL HARQ feebacks collected during last TTI
  if (m_dlInfoListReceived.size () > 0)
    {
      //to prevent errors after UE context deletion in scheduler----------------------
      for (std::vector<DlInfoListElement_s>::iterator dit = m_dlInfoListReceived.begin ();
           dit != m_dlInfoListReceived.end (); )
        {
          if (m_rlcAttached.find (dit->m_rnti) == m_rlcAttached.end ())
            {
              dit = m_dlInfoListReceived.erase (dit);
            }
          else
            {
              ++dit;
            }
        } //--------------------
      dlparams.m_dlInfoList = m_dlInfoListReceived;
      // empty local buffer
      m_dlInfoListReceived.clear ();
    }

  m_schedSapProvider->SchedDlTriggerReq (dlparams);


  // --- UPLINK ---
  // Send UL-CQI info to the scheduler
  for (uint16_t i = 0; i < m_ulCqiReceived.size (); i++)
    {
      if (subframeNo > 1)
        {
          m_ulCqiReceived.at (i).m_sfnSf = ((0x3FF & frameNo) << 4) | (0xF & (subframeNo - 1));
        }
      else
        {
          m_ulCqiReceived.at (i).m_sfnSf = ((0x3FF & (frameNo - 1)) << 4) | (0xF & 10);
        }
      m_schedSapProvider->SchedUlCqiInfoReq (m_ulCqiReceived.at (i));
    }
  m_ulCqiReceived.clear ();

  // Send BSR reports to the scheduler
  if (m_ulCeReceived.size () > 0)
    {
      FfMacSchedSapProvider::SchedUlMacCtrlInfoReqParameters ulMacReq;
      ulMacReq.m_sfnSf = ((0x3FF & frameNo) << 4) | (0xF & subframeNo);
      ulMacReq.m_macCeList.insert (ulMacReq.m_macCeList.begin (), m_ulCeReceived.begin (), m_ulCeReceived.end ());
      m_ulCeReceived.erase (m_ulCeReceived.begin (), m_ulCeReceived.end ());
      m_schedSapProvider->SchedUlMacCtrlInfoReq (ulMacReq);
    }


  // Get uplink transmission opportunities
  uint32_t ulSchedFrameNo = m_frameNo;
  uint32_t ulSchedSubframeNo = m_subframeNo;
  //   NS_LOG_DEBUG (this << " sfn " << frameNo << " sbfn " << subframeNo);
  if (ulSchedSubframeNo + (m_macChTtiDelay + UL_PUSCH_TTIS_DELAY) > 10)
    {
      ulSchedFrameNo++;
      ulSchedSubframeNo = (ulSchedSubframeNo + (m_macChTtiDelay + UL_PUSCH_TTIS_DELAY)) % 10;
    }
  else
    {
      ulSchedSubframeNo = ulSchedSubframeNo + (m_macChTtiDelay + UL_PUSCH_TTIS_DELAY);
    }
  FfMacSchedSapProvider::SchedUlTriggerReqParameters ulparams;
  ulparams.m_sfnSf = ((0x3FF & ulSchedFrameNo) << 4) | (0xF & ulSchedSubframeNo);

  // Forward DL HARQ feebacks collected during last TTI
  if (m_ulInfoListReceived.size () > 0)
    {
      ulparams.m_ulInfoList = m_ulInfoListReceived;
      // empty local buffer
      m_ulInfoListReceived.clear ();
    }

  m_schedSapProvider->SchedUlTriggerReq (ulparams);

}


void
LteEnbMac::DoReceiveLteControlMessage  (Ptr<LteControlMessage> msg)
{
  NS_LOG_FUNCTION (this << msg);
  if (msg->GetMessageType () == LteControlMessage::DL_CQI)
    {
      Ptr<DlCqiLteControlMessage> dlcqi = DynamicCast<DlCqiLteControlMessage> (msg);
      ReceiveDlCqiLteControlMessage (dlcqi);
      uint16_t rnti = dlcqi->GetDlCqi ().m_rnti;
      //trigger timing advance calculation only after UE receives rrc connection request
      std::map<uint16_t, UeInfo>::iterator uit = m_ueInfoMap.find (rnti); //send TAC only when UE is in-sync
      if (m_rntiImsiMap.find (rnti) != m_rntiImsiMap.end () && uit != m_ueInfoMap.end ()
          && !uit->second.isUplinkOutOfSync && uit->second.txRxInactivityTimeout.IsRunning ())
        {
          SendTimingAdvanceCommand (rnti);
        }
    }
  else if (msg->GetMessageType () == LteControlMessage::BSR)
    {
      Ptr<BsrLteControlMessage> bsr = DynamicCast<BsrLteControlMessage> (msg);
      ReceiveBsrMessage (bsr->GetBsr ());
      uint16_t rnti = bsr->GetBsr ().m_rnti;
      RestartInactivityTimers (rnti); //restart inactivity timer when ul data sent by UE
      //trigger timing advance calculation only after UE receives rrc connection request
      std::map<uint16_t, UeInfo>::iterator uit = m_ueInfoMap.find (rnti); //send TAC only when UE is in-sync
      if (m_rntiImsiMap.find (rnti) != m_rntiImsiMap.end () && uit != m_ueInfoMap.end ()
          && !uit->second.isUplinkOutOfSync && uit->second.txRxInactivityTimeout.IsRunning ())
        {
          SendTimingAdvanceCommand (rnti);
        }
    }
  else if (msg->GetMessageType () == LteControlMessage::DL_HARQ)
    {
      Ptr<DlHarqFeedbackLteControlMessage> dlharq = DynamicCast<DlHarqFeedbackLteControlMessage> (msg);
      DoDlInfoListElementHarqFeeback (dlharq->GetDlHarqFeedback ());
      uint16_t rnti = dlharq->GetDlHarqFeedback ().m_rnti;
      //trigger timing advance calculation only after UE receives rrc connection request
      std::map<uint16_t, UeInfo>::iterator uit = m_ueInfoMap.find (rnti); //send TAC only when UE is in-sync
      if (m_rntiImsiMap.find (rnti) != m_rntiImsiMap.end () && uit != m_ueInfoMap.end ()
          && !uit->second.isUplinkOutOfSync && uit->second.txRxInactivityTimeout.IsRunning ())
        {
          SendTimingAdvanceCommand (rnti);
        }
    }
  else if (msg->GetMessageType () == LteControlMessage::CRNTI)
    {
      Ptr<CRntiLteControlMessage> crntiMsg = DynamicCast<CRntiLteControlMessage> (msg);
      uint16_t rnti = crntiMsg->GetCRnti ().m_rnti;
      NS_LOG_INFO ("CRNTI MAC received from rnti: " << rnti);
      std::map<uint16_t, UeInfo>::iterator uit = m_ueInfoMap.find (rnti);
      NS_ASSERT_MSG (m_componentCarrierId == 0, "CRNTI ctrl msg received on carrier other than primary");
      if (uit != m_ueInfoMap.end () && !uit->second.isNonContentionBasedRandomAccess)
        {
          uit->second.isUplinkOutOfSync = false;
          if (m_realPrach)
            {
              uit->second.isMsg4Ready = true;
            }
          uit->second.isRandomAccessResponse = true;
          SendTimingAdvanceCommand (rnti);
          //send L2/L3 msg4 (RRC connection reconfiguration)
          m_cmacSapUser->SendConnectionReconfigurationMsg (rnti);
        }
    }
  else
    {
      NS_LOG_LOGIC (this << " LteControlMessage type " << msg->GetMessageType () << " not recognized");
    }
}

void
LteEnbMac::DoReceiveRachPreamble  (Ptr<RachPreambleLteControlMessage> msg)
{
  uint8_t rapId = msg->GetRapId ();
  m_receiveRachPreambleTrace (msg);
  Time current = Simulator::Now ();
  NS_LOG_INFO ("Enb " << this << " has received " << (uint32_t) rapId << " at time " << current.GetSeconds () << "\n");
  // just record that the preamble has been received; it will be processed later
  m_receivedRachPreambleCount.insert (std::pair<uint8_t, Vector> (rapId, msg->GetPosition ())); // will create entry if not exists
}

void
LteEnbMac::DoUlCqiReport (FfMacSchedSapProvider::SchedUlCqiInfoReqParameters ulcqi)
{
  if (ulcqi.m_ulCqi.m_type == UlCqi_s::PUSCH)
    {
      NS_LOG_DEBUG (this << " eNB rxed an PUSCH UL-CQI");
    }
  else if (ulcqi.m_ulCqi.m_type == UlCqi_s::SRS)
    {
      NS_LOG_DEBUG (this << " eNB rxed an SRS UL-CQI");

      //New code for generating timing advance when  srs received
      Ptr<SrsCqiRntiVsp> srscqi = DynamicCast<SrsCqiRntiVsp> ((ulcqi.m_vendorSpecificList.at (0).m_value));
      uint16_t rnti = srscqi->GetRnti ();
      NS_LOG_DEBUG (" srs generated by UE with rnti: " << rnti);
      std::map<uint16_t, UeInfo>::iterator uit = m_ueInfoMap.find (rnti); //send TAC only when UE is in-sync
      if (m_rntiImsiMap.find (rnti) != m_rntiImsiMap.end () && uit != m_ueInfoMap.end ()
          && !uit->second.isUplinkOutOfSync && uit->second.txRxInactivityTimeout.IsRunning ())
        {
          SendTimingAdvanceCommand (rnti);
        }
    }
  m_ulCqiReceived.push_back (ulcqi);
}


void
LteEnbMac::ReceiveDlCqiLteControlMessage  (Ptr<DlCqiLteControlMessage> msg)
{
  NS_LOG_FUNCTION (this << msg);

  CqiListElement_s dlcqi = msg->GetDlCqi ();
  NS_LOG_LOGIC (this << "Enb Received DL-CQI rnti " << dlcqi.m_rnti);
  NS_ASSERT (dlcqi.m_rnti != 0);
  m_dlCqiReceived.push_back (dlcqi);

}


void
LteEnbMac::ReceiveBsrMessage  (MacCeListElement_s bsr)
{
  NS_LOG_FUNCTION (this);
  m_ccmMacSapUser->UlReceiveMacCe (bsr, m_componentCarrierId);
}

void
LteEnbMac::DoReportMacCeToScheduler (MacCeListElement_s bsr)
{
  NS_LOG_FUNCTION (this);
  NS_LOG_DEBUG (this << " bsr Size " << (uint16_t) m_ulCeReceived.size ());
  //send to LteCcmMacSapUser
  m_ulCeReceived.push_back (bsr); // this to called when LteUlCcmSapProvider::ReportMacCeToScheduler is called
  NS_LOG_DEBUG (this << " bsr Size after push_back " << (uint16_t) m_ulCeReceived.size ());
}


void
LteEnbMac::DoReceivePhyPdu (Ptr<Packet> p)
{
  NS_LOG_FUNCTION (this);
  LteRadioBearerTag tag;
  p->RemovePacketTag (tag);

  // store info of the packet received

//   std::map <uint16_t,UlInfoListElement_s>::iterator it;
//   u_int rnti = tag.GetRnti ();
//  u_int lcid = tag.GetLcid ();
//   it = m_ulInfoListElements.find (tag.GetRnti ());
//   if (it == m_ulInfoListElements.end ())
//     {
//       // new RNTI
//       UlInfoListElement_s ulinfonew;
//       ulinfonew.m_rnti = tag.GetRnti ();
//       // always allocate full size of ulReception vector, initializing all elements to 0
//       ulinfonew.m_ulReception.assign (MAX_LC_LIST+1, 0);
//       // set the element for the current LCID
//       ulinfonew.m_ulReception.at (tag.GetLcid ()) = p->GetSize ();
//       ulinfonew.m_receptionStatus = UlInfoListElement_s::Ok;
//       ulinfonew.m_tpc = 0; // Tx power control not implemented at this stage
//       m_ulInfoListElements.insert (std::pair<uint16_t, UlInfoListElement_s > (tag.GetRnti (), ulinfonew));
//
//     }
//   else
//     {
//       // existing RNTI: we just set the value for the current
//       // LCID. Note that the corresponding element had already been
//       // allocated previously.
//       NS_ASSERT_MSG ((*it).second.m_ulReception.at (tag.GetLcid ()) == 0, "would overwrite previously written ulReception element");
//       (*it).second.m_ulReception.at (tag.GetLcid ()) = p->GetSize ();
//       (*it).second.m_receptionStatus = UlInfoListElement_s::Ok;
//     }



  // forward the packet to the correspondent RLC
  uint16_t rnti = tag.GetRnti ();
  uint8_t lcid = tag.GetLcid ();
  std::map <uint16_t, std::map<uint8_t, LteMacSapUser*> >::iterator rntiIt = m_rlcAttached.find (rnti);

  /**
   * The assert message is commented and changed to info message
   * so that the simulation can be continued without interruption
   * when packets arrive after UE is disconnected from the eNodeB.
   * The packets are discarded if the UE is no longer connected to
   * the eNodeB.
   */
  // NS_ASSERT_MSG (rntiIt != m_rlcAttached.end (), "could not find RNTI" << rnti);
  if (rntiIt != m_rlcAttached.end ())
    {
      std::map<uint8_t, LteMacSapUser*>::iterator lcidIt = rntiIt->second.find (lcid);
      // NS_ASSERT_MSG (lcidIt != rntiIt->second.end (), "could not find LCID" << lcid);


      std::map<uint16_t, UeInfo>::iterator uit = m_ueInfoMap.find (rnti);
      NS_ASSERT_MSG (uit != m_ueInfoMap.end (),"could not find any UE with rnti " << rnti);

      //Receive PDU only if LCID is found and UE is in sync
      if (lcidIt != rntiIt->second.end () && !uit->second.isUplinkOutOfSync)
        {
          (*lcidIt).second->ReceivePdu (p, rnti, lcid);
        }
    }
  else
    {
      NS_LOG_INFO ("reject packet from UE: " << rnti << " since it is already disconnected from EnodeB");
    }
}



// ////////////////////////////////////////////
// CMAC SAP
// ////////////////////////////////////////////

void
LteEnbMac::DoConfigureMac (uint8_t ulBandwidth, uint8_t dlBandwidth)
{
  NS_LOG_FUNCTION (this << " ulBandwidth=" << (uint16_t) ulBandwidth << " dlBandwidth=" << (uint16_t) dlBandwidth);
  FfMacCschedSapProvider::CschedCellConfigReqParameters params;
  // Configure the subset of parameters used by FfMacScheduler
  params.m_ulBandwidth = ulBandwidth;
  params.m_dlBandwidth = dlBandwidth;
  m_macChTtiDelay = m_enbPhySapProvider->GetMacChTtiDelay ();
  // ...more parameters can be configured
  m_cschedSapProvider->CschedCellConfigReq (params);
}


void
LteEnbMac::DoAddUe (uint16_t rnti)
{
  NS_LOG_FUNCTION (this << " rnti=" << rnti);
  std::map<uint8_t, LteMacSapUser*> empty;
  std::pair <std::map <uint16_t, std::map<uint8_t, LteMacSapUser*> >::iterator, bool>
  ret = m_rlcAttached.insert (std::pair <uint16_t,  std::map<uint8_t, LteMacSapUser*> >
                                (rnti, empty));
  NS_ASSERT_MSG (ret.second, "element already present, RNTI already existed");

  FfMacCschedSapProvider::CschedUeConfigReqParameters params;
  params.m_rnti = rnti;
  params.m_transmissionMode = 0; // set to default value (SISO) for avoiding random initialization (valgrind error)

  m_cschedSapProvider->CschedUeConfigReq (params);

  // Create DL trasmission HARQ buffers
  std::vector < Ptr<PacketBurst> > dlHarqLayer0pkt;
  dlHarqLayer0pkt.resize (8);
  for (uint8_t i = 0; i < 8; i++)
    {
      Ptr<PacketBurst> pb = CreateObject <PacketBurst> ();
      dlHarqLayer0pkt.at (i) = pb;
    }
  std::vector < Ptr<PacketBurst> > dlHarqLayer1pkt;
  dlHarqLayer1pkt.resize (8);
  for (uint8_t i = 0; i < 8; i++)
    {
      Ptr<PacketBurst> pb = CreateObject <PacketBurst> ();
      dlHarqLayer1pkt.at (i) = pb;
    }
  DlHarqProcessesBuffer_t buf;
  buf.push_back (dlHarqLayer0pkt);
  buf.push_back (dlHarqLayer1pkt);
  m_miDlHarqProcessesPackets.insert (std::pair <uint16_t, DlHarqProcessesBuffer_t> (rnti, buf));
  SetUeInfoMap (rnti);
}

void
LteEnbMac::DoRemoveUe (uint16_t rnti)
{
  NS_LOG_FUNCTION (this << " rnti=" << rnti);
  DoReleaseLc (rnti);
  FfMacCschedSapProvider::CschedUeReleaseReqParameters params;
  params.m_rnti = rnti;
  m_cschedSapProvider->CschedUeReleaseReq (params); //remove Ue context in scheduler
  m_rlcAttached.erase (rnti);
  m_miDlHarqProcessesPackets.erase (rnti);


  std::map<uint16_t, UeInfo>::iterator uit = m_ueInfoMap.find (rnti);
  if (uit != m_ueInfoMap.end ())
    {
      uit->second.timeAlignmentTimeout.Cancel ();
      uit->second.txRxInactivityTimeout.Cancel ();
      uit->second.enterTimeAlignmentTimerMargin.Cancel ();
    }
  m_ueInfoMap.erase (rnti);
  m_rntiImsiMap.erase (rnti);
  NS_LOG_DEBUG ("start checking for unprocessed preamble for rnti: " << rnti);
  //remove unprocessed preamble received for RACH during handover
  std::map<uint8_t, NcRaPreambleInfo>::iterator jt;
  for (jt = m_allocatedNcRaPreambleMap.begin (); jt != m_allocatedNcRaPreambleMap.end (); )
    {
      if (jt->second.rnti == rnti)
        {
          std::map<uint8_t, Vector>::const_iterator it = m_receivedRachPreambleCount.find (jt->first);
          if (it != m_receivedRachPreambleCount.end ())
            {
              m_receivedRachPreambleCount.erase (it->first);
            }
          jt = m_allocatedNcRaPreambleMap.erase (jt);
        }
      else
        {
          ++jt;
        }
    }
  for (std::vector<MacCeListElement_s>::iterator bit = m_ulCeReceived.begin ();
       bit != m_ulCeReceived.end (); )
    {
      if (bit->m_rnti == rnti)
        {
          bit = m_ulCeReceived.erase (bit);
        }
      else
        {
          ++bit;
        }
    }

}

void
LteEnbMac::DoAddLc (LteEnbCmacSapProvider::LcInfo lcinfo, LteMacSapUser* msu)
{
  NS_LOG_FUNCTION (this << lcinfo.rnti << (uint16_t) lcinfo.lcId);

  std::map <LteFlowId_t, LteMacSapUser* >::iterator it;

  LteFlowId_t flow (lcinfo.rnti, lcinfo.lcId);

  std::map <uint16_t, std::map<uint8_t, LteMacSapUser*> >::iterator rntiIt = m_rlcAttached.find (lcinfo.rnti);
  NS_ASSERT_MSG (rntiIt != m_rlcAttached.end (), "RNTI not found");
  std::map<uint8_t, LteMacSapUser*>::iterator lcidIt = rntiIt->second.find (lcinfo.lcId);
  if (lcidIt == rntiIt->second.end ())
    {
      rntiIt->second.insert (std::pair<uint8_t, LteMacSapUser*> (lcinfo.lcId, msu));
    }
  else
    {
      NS_LOG_ERROR ("LC already exists");
    }

  // CCCH (LCID 0) is pre-configured
  // see FF LTE MAC Scheduler
  // Interface Specification v1.11,
  // 4.3.4 logicalChannelConfigListElement
  if (lcinfo.lcId != 0)
    {
      struct FfMacCschedSapProvider::CschedLcConfigReqParameters params;
      params.m_rnti = lcinfo.rnti;
      params.m_reconfigureFlag = false;

      struct LogicalChannelConfigListElement_s lccle;
      lccle.m_logicalChannelIdentity = lcinfo.lcId;
      lccle.m_logicalChannelGroup = lcinfo.lcGroup;
      lccle.m_direction = LogicalChannelConfigListElement_s::DIR_BOTH;
      lccle.m_qosBearerType = lcinfo.isGbr ? LogicalChannelConfigListElement_s::QBT_GBR : LogicalChannelConfigListElement_s::QBT_NON_GBR;
      lccle.m_qci = lcinfo.qci;
      lccle.m_eRabMaximulBitrateUl = lcinfo.mbrUl;
      lccle.m_eRabMaximulBitrateDl = lcinfo.mbrDl;
      lccle.m_eRabGuaranteedBitrateUl = lcinfo.gbrUl;
      lccle.m_eRabGuaranteedBitrateDl = lcinfo.gbrDl;
      params.m_logicalChannelConfigList.push_back (lccle);

      m_cschedSapProvider->CschedLcConfigReq (params);
    }
}

void
LteEnbMac::DoReconfigureLc (LteEnbCmacSapProvider::LcInfo lcinfo)
{
  NS_FATAL_ERROR ("not implemented");
}

void
LteEnbMac::DoReleaseLc (uint16_t rnti)
{
  //Find user based on rnti and then erase lcid stored against the same
  std::map <uint16_t, std::map<uint8_t, LteMacSapUser*> >::iterator rntiIt = m_rlcAttached.find (rnti);
  // cycle on all the LC and delete one by one
    std::map <uint8_t, LteMacSapUser*>::iterator lcidIt = rntiIt->second.begin (); 
    std::vector<uint8_t> tempIt;
    for(;lcidIt != rntiIt->second.end();++lcidIt){
      tempIt.push_back(lcidIt->first);
    }
    lcidIt = rntiIt->second.begin ();
    int count = 0;
    while (true)
    {
      DoReleaseLc (rnti, lcidIt->first);
      if(count >= (int)tempIt.size()){
        break;
      }
      if(tempIt.at(count) != lcidIt->first){
        break;
      }
      lcidIt++;
      count++;
      if(lcidIt == rntiIt->second.end()){
        break;
      }
    }
}

void
LteEnbMac::DoReleaseLc (uint16_t rnti, uint8_t lcid)
{
  NS_LOG_FUNCTION (this);

  //Find user based on rnti and then erase lcid stored against the same
  std::map <uint16_t, std::map<uint8_t, LteMacSapUser*> >::iterator rntiIt = m_rlcAttached.find (rnti);
  if(rntiIt != m_rlcAttached.end()){
    rntiIt->second.erase (lcid);

    struct FfMacCschedSapProvider::CschedLcReleaseReqParameters params;
    params.m_rnti = rnti;
    params.m_logicalChannelIdentity.push_back (lcid);
    m_cschedSapProvider->CschedLcReleaseReq (params);
  }
}

void
LteEnbMac::DoUeUpdateConfigurationReq (LteEnbCmacSapProvider::UeConfig params)
{
  NS_LOG_FUNCTION (this);

  // propagates to scheduler
  FfMacCschedSapProvider::CschedUeConfigReqParameters req;
  req.m_rnti = params.m_rnti;
  req.m_transmissionMode = params.m_transmissionMode;
  req.m_reconfigureFlag = true;
  m_cschedSapProvider->CschedUeConfigReq (req);
}

LteEnbCmacSapProvider::RachConfig
LteEnbMac::DoGetRachConfig ()
{
  struct LteEnbCmacSapProvider::RachConfig rc;
  rc.numberOfRaPreambles = m_numberOfRaPreambles;
  rc.preambleTransMax = m_preambleTransMax;
  rc.raResponseWindowSize = m_raResponseWindowSize;


  rc.pRachConfigurationIndex = m_pRachConfigurationIndex;
  rc.powerRampingStep = m_powerRampingStep;
  rc.preambleInitialReceivedTargetPower = m_preambleInitialReceivedTargetPower;
  rc.contentionResolutionTimer = m_contentionResolutionTimer;

  return rc;
}

LteEnbCmacSapProvider::AllocateNcRaPreambleReturnValue
LteEnbMac::DoAllocateNcRaPreamble (uint16_t rnti)
{
  bool found = false;
  uint8_t preambleId;
  for (preambleId = m_numberOfRaPreambles; preambleId < 64; ++preambleId)
    {
      std::map<uint8_t, NcRaPreambleInfo>::iterator it = m_allocatedNcRaPreambleMap.find (preambleId);
      //allocate preamble only if its free
      if ((it != m_allocatedNcRaPreambleMap.end ()) && (it->second.expiryTime < Simulator::Now ()))
        {
          std::map<uint16_t, UeInfo>::iterator uit = m_ueInfoMap.find (rnti);
          NS_ASSERT_MSG (uit != m_ueInfoMap.end (),"could not find any UE with rnti " << rnti);
          if (!(!uit->second.isUplinkOutOfSync && m_cmacSapUser->IsRandomAccessCompleted (rnti)))
            {
              continue;
            }
        }
      if ((it ==  m_allocatedNcRaPreambleMap.end ())
          || (it->second.expiryTime < Simulator::Now ()))
        {
          found = true;
          NcRaPreambleInfo preambleInfo;
          uint32_t expiryIntervalMs = (uint32_t) m_preambleTransMax * ((uint32_t) m_raResponseWindowSize + 5);

          preambleInfo.expiryTime = Simulator::Now () + MilliSeconds (expiryIntervalMs);
          preambleInfo.rnti = rnti;
          NS_LOG_INFO ("allocated preamble for NC based RA: preamble " << preambleId << ", RNTI " << preambleInfo.rnti << ", exiryTime " << preambleInfo.expiryTime);
          m_allocatedNcRaPreambleMap[preambleId] = preambleInfo; // create if not exist, update otherwise
          break;
        }
    }
  LteEnbCmacSapProvider::AllocateNcRaPreambleReturnValue ret;
  if (found)
    {
      ret.valid = true;
      ret.raPreambleId = preambleId;
      ret.raPrachMaskIndex = 0;
    }
  else
    {
      ret.valid = false;
      ret.raPreambleId = 0;
      ret.raPrachMaskIndex = 0;
    }
  return ret;
}



// ////////////////////////////////////////////
// MAC SAP
// ////////////////////////////////////////////


void
LteEnbMac::DoTransmitPdu (LteMacSapProvider::TransmitPduParameters params)
{
  NS_LOG_FUNCTION (this);
  //to avoid transmission errors when out of sync due to data in scheduler buffers
  std::map<uint16_t, UeInfo>::iterator uit = m_ueInfoMap.find (params.rnti);
  NS_ASSERT_MSG (uit != m_ueInfoMap.end (),"could not find any UE with rnti " << params.rnti);
  if (uit->second.isUplinkOutOfSync)
    {
      NS_LOG_ERROR ("invalid transmission when UE with RNTI " << params.rnti << " is out of sync");
      return;
    }

  LteRadioBearerTag tag (params.rnti, params.lcid, params.layer);
  params.pdu->AddPacketTag (tag);
  params.componentCarrierId = m_componentCarrierId;
  // Store pkt in HARQ buffer
  std::map <uint16_t, DlHarqProcessesBuffer_t>::iterator it =  m_miDlHarqProcessesPackets.find (params.rnti);
  NS_ASSERT (it != m_miDlHarqProcessesPackets.end ());
  NS_LOG_DEBUG (this << " LAYER " << (uint16_t)tag.GetLayer () << " HARQ ID " << (uint16_t)params.harqProcessId);

  //(*it).second.at (params.layer).at (params.harqProcessId) = params.pdu;//->Copy ();
  (*it).second.at (params.layer).at (params.harqProcessId)->AddPacket (params.pdu);
  m_enbPhySapProvider->SendMacPdu (params.pdu);
  //Send contention resolution with msg4
  if (uit->second.isMsg4Ready == true)
    {
      NS_LOG_INFO ("UE contention resolution message is sent to UE with rnti " << params.rnti);
      Ptr<CriLteControlMessage> msg = Create<CriLteControlMessage> ();
      msg->SetUeContentionResolutionIdentity (m_rntiImsiMap.at (params.rnti));
      msg->SetRnti (params.rnti);
      m_enbPhySapProvider->SendLteControlMessage (msg);
    }
}

void
LteEnbMac::DoReportBufferStatus (LteMacSapProvider::ReportBufferStatusParameters params)
{
  NS_LOG_FUNCTION (this);
  std::map<uint16_t, UeInfo>::iterator uit = m_ueInfoMap.find (params.rnti);
  RestartInactivityTimers (params.rnti); //restart inactivity timer when dl data sent to UE
  if (uit != m_ueInfoMap.end () && uit->second.isUplinkOutOfSync
      && !uit->second.rachStarted && m_componentCarrierId == 0)
    {
      uit->second.rachStarted = true;
      SendPdcchOrder (params.rnti);
      return;
    }

  FfMacSchedSapProvider::SchedDlRlcBufferReqParameters req;
  req.m_rnti = params.rnti;
  req.m_logicalChannelIdentity = params.lcid;
  req.m_rlcTransmissionQueueSize = params.txQueueSize;
  req.m_rlcTransmissionQueueHolDelay = params.txQueueHolDelay;
  req.m_rlcRetransmissionQueueSize = params.retxQueueSize;
  req.m_rlcRetransmissionHolDelay = params.retxQueueHolDelay;
  req.m_rlcStatusPduSize = params.statusPduSize;
  m_schedSapProvider->SchedDlRlcBufferReq (req);
}



// ////////////////////////////////////////////
// SCHED SAP
// ////////////////////////////////////////////



void
LteEnbMac::DoSchedDlConfigInd (FfMacSchedSapUser::SchedDlConfigIndParameters ind)
{
  NS_LOG_FUNCTION (this);
  // Create DL PHY PDU
  Ptr<PacketBurst> pb = CreateObject<PacketBurst> ();
  std::map <LteFlowId_t, LteMacSapUser* >::iterator it;

  for (unsigned int i = 0; i < ind.m_buildDataList.size (); i++)
    {
      for (uint16_t layer = 0; layer < ind.m_buildDataList.at (i).m_dci.m_ndi.size (); layer++)
        {
          if (ind.m_buildDataList.at (i).m_dci.m_ndi.at (layer) == 1)
            {
              // new data -> force emptying correspondent harq pkt buffer
              std::map <uint16_t, DlHarqProcessesBuffer_t>::iterator it = m_miDlHarqProcessesPackets.find (ind.m_buildDataList.at (i).m_rnti);
              NS_ASSERT (it != m_miDlHarqProcessesPackets.end ());
              for (uint16_t lcId = 0; lcId < (*it).second.size (); lcId++)
                {
                  Ptr<PacketBurst> pb = CreateObject <PacketBurst> ();
                  (*it).second.at (lcId).at (ind.m_buildDataList.at (i).m_dci.m_harqProcess) = pb;
                }
            }
        }
      for (unsigned int j = 0; j < ind.m_buildDataList.at (i).m_rlcPduList.size (); j++)
        {
          for (uint16_t k = 0; k < ind.m_buildDataList.at (i).m_rlcPduList.at (j).size (); k++)
            {
              if (ind.m_buildDataList.at (i).m_dci.m_ndi.at (k) == 1)
                {
                  // New Data -> retrieve it from RLC
                  uint16_t rnti = ind.m_buildDataList.at (i).m_rnti;
                  uint8_t lcid = ind.m_buildDataList.at (i).m_rlcPduList.at (j).at (k).m_logicalChannelIdentity;
                  std::map <uint16_t, std::map<uint8_t, LteMacSapUser*> >::iterator rntiIt = m_rlcAttached.find (rnti);
                  NS_ASSERT_MSG (rntiIt != m_rlcAttached.end (), "could not find RNTI" << rnti);
                  std::map<uint8_t, LteMacSapUser*>::iterator lcidIt = rntiIt->second.find (lcid);
                  NS_ASSERT_MSG (lcidIt != rntiIt->second.end (), "could not find LCID" << (uint32_t)lcid << " carrier id:" << (uint16_t)m_componentCarrierId);
                  NS_LOG_DEBUG (this << " rnti= " << rnti << " lcid= " << (uint32_t) lcid << " layer= " << k);
                  (*lcidIt).second->NotifyTxOpportunity (ind.m_buildDataList.at (i).m_rlcPduList.at (j).at (k).m_size, k, ind.m_buildDataList.at (i).m_dci.m_harqProcess, m_componentCarrierId, rnti, lcid);
                }
              else
                {
                  if (ind.m_buildDataList.at (i).m_dci.m_tbsSize.at (k) > 0)
                    {
                      // HARQ retransmission -> retrieve TB from HARQ buffer
                      std::map <uint16_t, DlHarqProcessesBuffer_t>::iterator it = m_miDlHarqProcessesPackets.find (ind.m_buildDataList.at (i).m_rnti);
                      NS_ASSERT (it != m_miDlHarqProcessesPackets.end ());
                      Ptr<PacketBurst> pb = (*it).second.at (k).at ( ind.m_buildDataList.at (i).m_dci.m_harqProcess);
                      for (std::list<Ptr<Packet> >::const_iterator j = pb->Begin (); j != pb->End (); ++j)
                        {
                          Ptr<Packet> pkt = (*j)->Copy ();
                          m_enbPhySapProvider->SendMacPdu (pkt);
                        }
                    }
                }
            }
        }
      //send dci if only UE is connected to the network
      std::map <uint16_t, std::map<uint8_t, LteMacSapUser*> >::iterator it = m_rlcAttached.find (ind.m_buildDataList.at (i).m_rnti);
      if (it != m_rlcAttached.end ())
        {
          // send the relative DCI
          Ptr<DlDciLteControlMessage> msg = Create<DlDciLteControlMessage> ();
          msg->SetDci (ind.m_buildDataList.at (i).m_dci);
          m_enbPhySapProvider->SendLteControlMessage (msg);
        }
    }

  // Fire the trace with the DL information
  for (  uint32_t i  = 0; i < ind.m_buildDataList.size (); i++ )
    {
      // Only one TB used
      if (ind.m_buildDataList.at (i).m_dci.m_tbsSize.size () == 1)
        {
          DlSchedulingCallbackInfo dlSchedulingCallbackInfo;
          dlSchedulingCallbackInfo.frameNo = m_frameNo;
          dlSchedulingCallbackInfo.subframeNo = m_subframeNo;
          dlSchedulingCallbackInfo.rnti = ind.m_buildDataList.at (i).m_dci.m_rnti;
          dlSchedulingCallbackInfo.mcsTb1 = ind.m_buildDataList.at (i).m_dci.m_mcs.at (0);
          dlSchedulingCallbackInfo.sizeTb1 = ind.m_buildDataList.at (i).m_dci.m_tbsSize.at (0);
          dlSchedulingCallbackInfo.mcsTb2 = 0;
          dlSchedulingCallbackInfo.sizeTb2 = 0;
          dlSchedulingCallbackInfo.componentCarrierId = m_componentCarrierId;
          m_dlScheduling (dlSchedulingCallbackInfo);
        }
      // Two TBs used
      else if (ind.m_buildDataList.at (i).m_dci.m_tbsSize.size () == 2)
        {
          DlSchedulingCallbackInfo dlSchedulingCallbackInfo;
          dlSchedulingCallbackInfo.frameNo = m_frameNo;
          dlSchedulingCallbackInfo.subframeNo = m_subframeNo;
          dlSchedulingCallbackInfo.rnti = ind.m_buildDataList.at (i).m_dci.m_rnti;
          dlSchedulingCallbackInfo.mcsTb1 = ind.m_buildDataList.at (i).m_dci.m_mcs.at (0);
          dlSchedulingCallbackInfo.sizeTb1 = ind.m_buildDataList.at (i).m_dci.m_tbsSize.at (0);
          dlSchedulingCallbackInfo.mcsTb2 = ind.m_buildDataList.at (i).m_dci.m_mcs.at (1);
          dlSchedulingCallbackInfo.sizeTb2 = ind.m_buildDataList.at (i).m_dci.m_tbsSize.at (1);
          dlSchedulingCallbackInfo.componentCarrierId = m_componentCarrierId;
          m_dlScheduling (dlSchedulingCallbackInfo);
        }
      else
        {
          NS_FATAL_ERROR ("Found element with more than two transport blocks");
        }
    }

  // Random Access procedure: send RARs
  Ptr<RarLteControlMessage> rarMsg = Create<RarLteControlMessage> ();
  // see TS 36.321 5.1.4;  preambles were sent two frames ago
  // (plus 3GPP counts subframes from 0, not 1)
  uint16_t raRnti;
  if (m_subframeNo < 3)
    {
      raRnti = m_subframeNo + 7; // equivalent to +10-3
    }
  else
    {
      raRnti = m_subframeNo - 3;
    }
  rarMsg->SetRaRnti (raRnti);
  rarMsg->SetBackoffIndicator (m_backoffIndicator);
  for (unsigned int i = 0; i < ind.m_buildRarList.size (); i++)
    {
      std::map <uint16_t, uint32_t>::iterator itRapId = m_rapIdRntiMap.find (ind.m_buildRarList.at (i).m_rnti);
      if (itRapId == m_rapIdRntiMap.end ())
        {
          NS_FATAL_ERROR ("Unable to find rapId of RNTI " << ind.m_buildRarList.at (i).m_rnti);
        }
      RarLteControlMessage::Rar rar;
      rar.rapId = itRapId->second;
      rar.rarPayload = ind.m_buildRarList.at (i);
      rarMsg->AddRar (rar);
      NS_LOG_INFO (this << " Send RAR message to RNTI " << ind.m_buildRarList.at (i).m_rnti << " rapId " << itRapId->second);
      //Set RAR is true in ueInfo
      std::map<uint16_t, UeInfo>::iterator uit = m_ueInfoMap.find (ind.m_buildRarList.at (i).m_rnti);
      if (uit != m_ueInfoMap.end () )
        {
          uit->second.isRandomAccessResponse = true;
        }
      m_cmacSapUser->CheckIfUeConnected (ind.m_buildRarList.at (i).m_rnti);
    }
  if (ind.m_buildRarList.size () > 0)
    {
      m_enbPhySapProvider->SendLteControlMessage (rarMsg);
    }
  m_rapIdRntiMap.clear ();
}


void
LteEnbMac::DoSchedUlConfigInd (FfMacSchedSapUser::SchedUlConfigIndParameters ind)
{
  NS_LOG_FUNCTION (this);

  for (unsigned int i = 0; i < ind.m_dciList.size (); i++)
    {
      // send the correspondent ul dci
      Ptr<UlDciLteControlMessage> msg = Create<UlDciLteControlMessage> ();
      msg->SetDci (ind.m_dciList.at (i));
      m_enbPhySapProvider->SendLteControlMessage (msg);
    }

  // Fire the trace with the UL information
  for (  uint32_t i  = 0; i < ind.m_dciList.size (); i++ )
    {
      m_ulScheduling (m_frameNo, m_subframeNo, ind.m_dciList.at (i).m_rnti,
                      ind.m_dciList.at (i).m_mcs, ind.m_dciList.at (i).m_tbSize, m_componentCarrierId);
    }



}




// ////////////////////////////////////////////
// CSCHED SAP
// ////////////////////////////////////////////


void
LteEnbMac::DoCschedCellConfigCnf (FfMacCschedSapUser::CschedCellConfigCnfParameters params)
{
  NS_LOG_FUNCTION (this);
}

void
LteEnbMac::DoCschedUeConfigCnf (FfMacCschedSapUser::CschedUeConfigCnfParameters params)
{
  NS_LOG_FUNCTION (this);
}

void
LteEnbMac::DoCschedLcConfigCnf (FfMacCschedSapUser::CschedLcConfigCnfParameters params)
{
  NS_LOG_FUNCTION (this);
  // Call the CSCHED primitive
  // m_cschedSap->LcConfigCompleted();
}

void
LteEnbMac::DoCschedLcReleaseCnf (FfMacCschedSapUser::CschedLcReleaseCnfParameters params)
{
  NS_LOG_FUNCTION (this);
}

void
LteEnbMac::DoCschedUeReleaseCnf (FfMacCschedSapUser::CschedUeReleaseCnfParameters params)
{
  NS_LOG_FUNCTION (this);
}

void
LteEnbMac::DoCschedUeConfigUpdateInd (FfMacCschedSapUser::CschedUeConfigUpdateIndParameters params)
{
  NS_LOG_FUNCTION (this);
  // propagates to RRC
  LteEnbCmacSapUser::UeConfig ueConfigUpdate;
  ueConfigUpdate.m_rnti = params.m_rnti;
  ueConfigUpdate.m_transmissionMode = params.m_transmissionMode;
  m_cmacSapUser->RrcConfigurationUpdateInd (ueConfigUpdate);
}

void
LteEnbMac::DoCschedCellConfigUpdateInd (FfMacCschedSapUser::CschedCellConfigUpdateIndParameters params)
{
  NS_LOG_FUNCTION (this);
}

void
LteEnbMac::DoUlInfoListElementHarqFeeback (UlInfoListElement_s params)
{
  NS_LOG_FUNCTION (this);
  //send UL info to scheduler only if UE is connected
  std::map<uint16_t, std::map<uint8_t, LteMacSapUser*> >::iterator it = m_rlcAttached.find (params.m_rnti);
  if (it != m_rlcAttached.end ())
    {
      m_ulInfoListReceived.push_back (params);
    }
}

void
LteEnbMac::DoDlInfoListElementHarqFeeback (DlInfoListElement_s params)
{
  NS_LOG_FUNCTION (this);
  // Update HARQ buffer
  std::map <uint16_t, DlHarqProcessesBuffer_t>::iterator it =  m_miDlHarqProcessesPackets.find (params.m_rnti);
  NS_ASSERT (it != m_miDlHarqProcessesPackets.end ());
  for (uint8_t layer = 0; layer < params.m_harqStatus.size (); layer++)
    {
      if (params.m_harqStatus.at (layer) == DlInfoListElement_s::ACK)
        {
          // discard buffer
          Ptr<PacketBurst> emptyBuf = CreateObject <PacketBurst> ();
          (*it).second.at (layer).at (params.m_harqProcessId) = emptyBuf;
          NS_LOG_DEBUG (this << " HARQ-ACK UE " << params.m_rnti << " harqId " << (uint16_t)params.m_harqProcessId << " layer " << (uint16_t)layer);

          //reset the flag when the ack for rrc connection setup is received
          std::map<uint16_t, UeInfo>::iterator uit = m_ueInfoMap.find (params.m_rnti);
          if (uit != m_ueInfoMap.end () && uit->second.isMsg4Ready == true)
            {
              uit->second.isMsg4Ready = false;
            }
        }
      else if (params.m_harqStatus.at (layer) == DlInfoListElement_s::NACK)
        {
          NS_LOG_DEBUG (this << " HARQ-NACK UE " << params.m_rnti << " harqId " << (uint16_t)params.m_harqProcessId << " layer " << (uint16_t)layer);
        }
      else
        {
          NS_FATAL_ERROR (" HARQ functionality not implemented");
        }
    }
  m_dlInfoListReceived.push_back (params);
}

bool
LteEnbMac::EvaluateCollisionProbability (std::list<Vector> pointList)
{
  NS_LOG_FUNCTION (this);
  // compute the distances and look for max and min
  // note: there are at least 2 entries in the list since more than one identical preamble was recvd
  double min_dist = std::numeric_limits<double>::max (); // distance is always > 0
  double max_dist = 0;

  for (std::list<Vector>::iterator it = pointList.begin (); it != pointList.end (); ++it)
    {
      double dist = std::sqrt ((*it).x * (*it).x + (*it).y * (*it).y + (*it).z * (*it).z);
      if (dist < min_dist)
        {
          min_dist = dist;
        }
      if (dist > max_dist)
        {
          max_dist = dist;
        }
    }

  bool collisionProbability = (max_dist - min_dist) > cpConstant;
  // the eNB can distinguish 2 sequences if the distance of the UEs from the eNB d1 and d2 is such that
  // |d1-d2|/c > T_chip with T_chip = 1/2B and B the band of the preamble sequence
  NS_LOG_INFO ("collisionProbability " << collisionProbability << " min_dist " << min_dist << " max_dist " << max_dist);
  return collisionProbability;
}

void
LteEnbMac::SendTimingAdvanceCommand (uint16_t rnti)
{
  NS_LOG_FUNCTION (this);
  if (!m_lteUeMobilityModelCallback.IsNull () && m_componentCarrierId == 0) //calculate timing advance only for primary carrier
    {
      std::map<uint16_t, uint64_t>::iterator it = m_rntiImsiMap.find (rnti);
      NS_ASSERT_MSG (it != m_rntiImsiMap.end (), "could not find any UE with rnti " << rnti);
      Ptr<MobilityModel> mm = m_lteUeMobilityModelCallback (it->second);
      NS_LOG_LOGIC ("Position of UE with rnti " << rnti << ", imsi " << it->second << ", is " << mm->GetPosition ());
      Ptr<MobilityModel> enbmm = m_enbPhySapProvider->GetEnbMobility ();
      NS_LOG_LOGIC ("Position of Enb is " << enbmm->GetPosition ());

      std::map<uint16_t, UeInfo>::iterator uit = m_ueInfoMap.find (rnti);
      NS_ASSERT_MSG (uit != m_ueInfoMap.end (), "could not find any UE with rnti " << rnti);

      uit->second.currentDistance = mm->GetDistanceFrom (enbmm);
      NS_LOG_LOGIC ("Distance of UE with imsi " << it->second << ", from enb is " << uit->second.currentDistance);

      if (int (uit->second.currentDistance / 78) > 1282)
        {
          NS_LOG_WARN ("Distance between eNodeB & UE is greater than 100km, timing advance not calculated");
          return;
        }

      uit->second.currentTimingAdvance = (uit->second.currentDistance / 78) + 1;
      NS_LOG_LOGIC ("current TA: " << uit->second.currentTimingAdvance << "last TA: " << uit->second.lastTimingAdvance);
      if (uit->second.isRandomAccessResponse == true || (uit->second.txRxInactivityTimeout.IsRunning ()
                                                         && ((uit->second.currentTimingAdvance != uit->second.lastTimingAdvance) || uit->second.isTimeAlignmentTimerMargin)))
        {
          uit->second.lastTimingAdvance = uit->second.currentTimingAdvance;
          if (uit->second.isRandomAccessResponse == true)
            {
              uit->second.isRandomAccessResponse = false;
              NS_LOG_INFO ("RAR Timing advance command sent");
            }
          else
            {
              MacCeListElement_s ce;
              ce.m_macCeType = MacCeListElement_s::TAC;
              ce.m_rnti = rnti;
              Ptr<TacLteControlMessage> msg = Create<TacLteControlMessage> ();
              msg->SetTac (ce);
              if (m_dynamicTimerConfigured)
                {
                  double ueRelativeSpeed = (uit->second.currentDistance - uit->second.previousDistance) / (Simulator::Now ().GetSeconds () - uit->second.timestampOfLastTimingAdvance.GetSeconds ());
                  NS_LOG_LOGIC ("relative speed of UE with rnti " << rnti << ", imsi " << it->second << ", is " << ueRelativeSpeed);
                  uint16_t timer = GetDynamicTimerValue (std::abs (ueRelativeSpeed));

                  msg->SetTimeAlignmentTimer (timer);
                  NS_LOG_LOGIC ("Dynamic timer Value: " << MilliSeconds (timer));
                }

              NS_LOG_INFO ("Timing advance command sent");
              m_enbPhySapProvider->SendLteControlMessage (msg);
            }
          uit->second.previousDistance = uit->second.currentDistance;
          uit->second.timestampOfLastTimingAdvance = Simulator::Now ();
          NS_LOG_LOGIC ("sent time of last TA: " << uit->second.timestampOfLastTimingAdvance.GetSeconds ());
        }
    }
}

void
LteEnbMac::SetLteUeMobilityModelCallback (LteUeMobilityModelCallback c)
{
  NS_LOG_FUNCTION (this);
  m_lteUeMobilityModelCallback = c;
}

void
LteEnbMac::DoSetRntiImsiMap (uint16_t rnti, uint64_t imsi)
{
  NS_LOG_FUNCTION (this << rnti << imsi);
  m_rntiImsiMap[rnti] = imsi;
  if (m_componentCarrierId == 0) //send timing advance only for primary carrier
    {
      SendTimingAdvanceCommand (rnti);
    }
}

void
LteEnbMac::SetUeInfoMap (uint16_t rnti)
{
  NS_LOG_FUNCTION (this << rnti);
  UeInfo ueInfo;
  ueInfo.currentTimingAdvance = 0;
  ueInfo.lastTimingAdvance = 0;
  ueInfo.isRandomAccessResponse = true;
  ueInfo.isUplinkOutOfSync = false;
  ueInfo.isNonContentionBasedRandomAccess = false;
  ueInfo.isMsg4Ready = false;
  ueInfo.isTimeAlignmentTimerMargin = false;
  ueInfo.rachStarted = false;
  m_ueInfoMap[rnti] = ueInfo;

}

void
LteEnbMac::DoStartTimeAlignmentTimer (Time timeAlignmentTimer, uint16_t rnti)
{
  NS_LOG_FUNCTION (this << rnti);
  std::map<uint16_t, UeInfo>::iterator uit = m_ueInfoMap.find (rnti);
  if (uit != m_ueInfoMap.end ())
    {
      uit->second.timeAlignmentTimeout.Cancel ();
      uit->second.txRxInactivityTimeout.Cancel ();
      uit->second.enterTimeAlignmentTimerMargin.Cancel ();
      uit->second.timeAlignmentTimeout = Simulator::Schedule (timeAlignmentTimer, &LteEnbMac::TimeAlignmentTimeout, this, rnti);
      NS_LOG_INFO ("t_AlignmentTimer started/restarted " << timeAlignmentTimer);
      if (uit->second.txRxInactivityTimeout.IsExpired () && uit->second.isRandomAccessResponse)
        {
          uit->second.txRxInactivityTimeout = Simulator::Schedule (m_txRxInactivityTimeoutDuration, &LteEnbMac::TxRxInactivityTimeout, this, rnti);
        }
      Time timeAlignmentTimerMargin = MilliSeconds (timeAlignmentTimer.GetMilliSeconds () * 0.2);
      uit->second.enterTimeAlignmentTimerMargin = Simulator::Schedule (timeAlignmentTimer - timeAlignmentTimerMargin, &LteEnbMac::EnterTimeAlignmentTimerMargin, this, rnti);
      NS_LOG_INFO ("timeAlignmentTimerMargin " << timeAlignmentTimerMargin);
      if (uit->second.isNonContentionBasedRandomAccess)
        {
          uit->second.isUplinkOutOfSync = false;
          uit->second.rachStarted = false;
          uit->second.isNonContentionBasedRandomAccess = false;
          m_cmacSapUser->SendConnectionReconfigurationMsg (rnti); //send RRC connection reconfiguration
        }
    }
}

void
LteEnbMac::TimeAlignmentTimeout (uint16_t rnti)
{
  NS_LOG_FUNCTION (this << rnti);
  if (!m_cmacSapUser->NotifyUplinkOutOfSync (rnti))
    {
      return; //don't set isUplinkOutOfSync when handover is taking place otherwise it triggers PDCCH order
    }
  std::map<uint16_t, UeInfo>::iterator uit = m_ueInfoMap.find (rnti);
  if (uit != m_ueInfoMap.end ())
    {
      uit->second.isUplinkOutOfSync = true;
      uit->second.isNonContentionBasedRandomAccess = false;
    }

  m_miDlHarqProcessesPackets.erase (rnti);

  // Create DL trasmission HARQ buffers
  std::vector<Ptr<PacketBurst> > dlHarqLayer0pkt;
  dlHarqLayer0pkt.resize (8);
  for (uint8_t i = 0; i < 8; i++)
    {
      Ptr<PacketBurst> pb = CreateObject<PacketBurst> ();
      dlHarqLayer0pkt.at (i) = pb;
    }
  std::vector<Ptr<PacketBurst> > dlHarqLayer1pkt;
  dlHarqLayer1pkt.resize (8);
  for (uint8_t i = 0; i < 8; i++)
    {
      Ptr<PacketBurst> pb = CreateObject<PacketBurst> ();
      dlHarqLayer1pkt.at (i) = pb;
    }
  DlHarqProcessesBuffer_t buf;
  buf.push_back (dlHarqLayer0pkt);
  buf.push_back (dlHarqLayer1pkt);
  m_miDlHarqProcessesPackets.insert (std::pair<uint16_t, DlHarqProcessesBuffer_t> (rnti, buf));
}

void
LteEnbMac::TxRxInactivityTimeout (uint16_t rnti)
{
  NS_LOG_FUNCTION (this << rnti);
  std::map<uint16_t, UeInfo>::iterator uit = m_ueInfoMap.find (rnti);
  NS_ASSERT_MSG (uit != m_ueInfoMap.end (), "could not find any UE with rnti " << rnti);

}

void
LteEnbMac::EnterTimeAlignmentTimerMargin (uint16_t rnti)
{
  NS_LOG_FUNCTION (this << rnti);
  std::map<uint16_t, UeInfo>::iterator uit = m_ueInfoMap.find (rnti);
  if (uit != m_ueInfoMap.end () && uit->second.txRxInactivityTimeout.IsRunning ())
    {
      if (m_rntiImsiMap.find (rnti) != m_rntiImsiMap.end ())
        {
          uit->second.isTimeAlignmentTimerMargin = true;
          SendTimingAdvanceCommand (rnti);
          uit->second.isTimeAlignmentTimerMargin = false;
        }
    }
}

uint16_t
LteEnbMac::GetDynamicTimerValue (double UeSpeed)
{
  NS_LOG_FUNCTION (this << UeSpeed);
  uint16_t TimerValue = 500; //default value

  if (UeSpeed > 46.3 && UeSpeed <= 69.4)
    {
      TimerValue = 500;
    }
  else if (UeSpeed > 27.1 && UeSpeed <= 46.3)
    {
      TimerValue = 750;
    }
  else if (UeSpeed > 18.1 && UeSpeed <= 27.1)
    {
      TimerValue = 1280;
    }
  else if (UeSpeed > 13.6 && UeSpeed <= 18.1)
    {
      TimerValue = 1920;
    }
  else if (UeSpeed > 6.8 && UeSpeed <= 13.6)
    {
      TimerValue = 2560;
    }
  else if (UeSpeed > 3.4 && UeSpeed <= 6.8)
    {
      TimerValue = 5120;
    }
  else if (UeSpeed >= 0 && UeSpeed <= 3.4)
    {
      TimerValue = 10240;
    }
  return TimerValue;
}

void
LteEnbMac::SendPdcchOrder (uint16_t rnti)
{
  NS_LOG_FUNCTION (this << rnti);
  /*
   * DCI Format 1A is used for random access procedure initiated by a PDCCH order
   * only if format 1A CRC is scrambled with C-RNTI and all the remaining fields are set as follows
   */
  DlDciListElement_s dci;
  dci.m_rnti = rnti;
  dci.m_format = DlDciListElement_s::ONE_A;
  dci.m_pdcchOrder = true;
  LteEnbCmacSapProvider::AllocateNcRaPreambleReturnValue anrcrv = DoAllocateNcRaPreamble (rnti);
  if (anrcrv.valid == false)
    {
      NS_LOG_INFO (" failed to allocate a preamble for non-contention based RA => cannot perform PDCCH Order");
      NS_LOG_INFO (" declare radio link failure");
      m_cmacSapUser->NotifyRandomAccessFailure (rnti);
      return;
    }
  dci.m_preambleIndex = anrcrv.raPreambleId;
  dci.m_prachMaskIndex = anrcrv.raPrachMaskIndex;
  dci.m_rbBitmap = 0;
  Ptr<DlDciLteControlMessage> msg = Create<DlDciLteControlMessage> ();
  msg->SetDci (dci);
  m_enbPhySapProvider->SendLteControlMessage (msg);
}

void
LteEnbMac::RestartInactivityTimers (uint16_t rnti)
{
  NS_LOG_FUNCTION (this << rnti);
  std::map<uint16_t, UeInfo>::iterator uit = m_ueInfoMap.find (rnti);
  if (uit != m_ueInfoMap.end ())
    {
      uit->second.txRxInactivityTimeout.Cancel ();
      uit->second.txRxInactivityTimeout = Simulator::Schedule (m_txRxInactivityTimeoutDuration, &LteEnbMac::TxRxInactivityTimeout, this, rnti);
      m_cmacSapUser->NotifyRrcToRestartInactivityTimer (rnti);
    }
}

void
LteEnbMac::DoSetMsg4Ready (uint16_t rnti)
{
  std::map<uint16_t, UeInfo>::iterator uit = m_ueInfoMap.find (rnti);
  if (uit != m_ueInfoMap.end () && m_realPrach)
    {
      uit->second.isMsg4Ready = true;
    }
}

} // namespace ns3
