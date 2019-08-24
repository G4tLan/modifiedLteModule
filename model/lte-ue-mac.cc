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
 * Author: Nicola Baldo  <nbaldo@cttc.es>
 * Author: Marco Miozzo <mmiozzo@cttc.es>
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
#include <ns3/packet-burst.h>
#include <ns3/random-variable-stream.h>
#include "lte-prach-info.h"
#include "lte-ue-mac.h"
#include "lte-ue-phy.h"
#include "lte-ue-net-device.h"
#include "lte-radio-bearer-tag.h"
#include <ns3/ff-mac-common.h>
#include <ns3/lte-control-messages.h>
#include <ns3/simulator.h>
#include <ns3/lte-common.h>



namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("LteUeMac");

NS_OBJECT_ENSURE_REGISTERED (LteUeMac);


///////////////////////////////////////////////////////////
// SAP forwarders
///////////////////////////////////////////////////////////

/// UeMemberLteUeCmacSapProvider class
class UeMemberLteUeCmacSapProvider : public LteUeCmacSapProvider
{
public:
  /**
   * Constructor
   *
   * \param mac the UE MAC
   */
  UeMemberLteUeCmacSapProvider (LteUeMac* mac);

  // inherited from LteUeCmacSapProvider
  virtual void ConfigureRach (RachConfig rc);
  virtual void StartContentionBasedRandomAccessProcedure ();
  virtual void StartNonContentionBasedRandomAccessProcedure (uint16_t rnti, uint8_t preambleId, uint8_t prachMask);
  virtual void SetRnti (uint16_t rnti);
  virtual void AddLc (uint8_t lcId, LteUeCmacSapProvider::LogicalChannelConfig lcConfig, LteMacSapUser* msu);
  virtual void RemoveLc (uint8_t lcId);
  virtual void Reset ();
  virtual void NotifyConnectionExpired ();
  virtual void NotifyConnectionSuccessful ();
  virtual void SetTimeAlignmentTimer (uint16_t timeAlignmentTimerCommon);

private:
  LteUeMac* m_mac; ///< the UE MAC
};


UeMemberLteUeCmacSapProvider::UeMemberLteUeCmacSapProvider (LteUeMac* mac)
  : m_mac (mac)
{
}

void
UeMemberLteUeCmacSapProvider::ConfigureRach (RachConfig rc)
{
  m_mac->DoConfigureRach (rc);
}

void
UeMemberLteUeCmacSapProvider::StartContentionBasedRandomAccessProcedure ()
{
  m_mac->DoStartContentionBasedRandomAccessProcedure ();
}

void
UeMemberLteUeCmacSapProvider::StartNonContentionBasedRandomAccessProcedure (uint16_t rnti, uint8_t preambleId, uint8_t prachMask)
{
  m_mac->DoStartNonContentionBasedRandomAccessProcedure (rnti, preambleId, prachMask);
}

void
UeMemberLteUeCmacSapProvider::SetRnti (uint16_t rnti)
{
  m_mac->DoSetRnti (rnti);
}

void
UeMemberLteUeCmacSapProvider::AddLc (uint8_t lcId, LogicalChannelConfig lcConfig, LteMacSapUser* msu)
{
  m_mac->DoAddLc (lcId, lcConfig, msu);
}

void
UeMemberLteUeCmacSapProvider::RemoveLc (uint8_t lcid)
{
  m_mac->DoRemoveLc (lcid);
}

void
UeMemberLteUeCmacSapProvider::Reset ()
{
  m_mac->DoReset ();
}

void
UeMemberLteUeCmacSapProvider::NotifyConnectionExpired ()
{
  m_mac->DoNotifyConnectionExpired ();
}

void
UeMemberLteUeCmacSapProvider::NotifyConnectionSuccessful ()
{
  m_mac->DoNotifyConnectionSuccessful ();
}

void UeMemberLteUeCmacSapProvider::SetTimeAlignmentTimer (uint16_t timeAlignmentTimerCommon)
{
  m_mac->DoSetTimeAlignmentTimer (timeAlignmentTimerCommon);
}

/// UeMemberLteMacSapProvider class
class UeMemberLteMacSapProvider : public LteMacSapProvider
{
public:
  /**
   * Constructor
   *
   * \param mac the UE MAC
   */
  UeMemberLteMacSapProvider (LteUeMac* mac);

  // inherited from LteMacSapProvider
  virtual void TransmitPdu (TransmitPduParameters params);
  virtual void ReportBufferStatus (ReportBufferStatusParameters params);

private:
  LteUeMac* m_mac; ///< the UE MAC
};


UeMemberLteMacSapProvider::UeMemberLteMacSapProvider (LteUeMac* mac)
  : m_mac (mac)
{
}

void
UeMemberLteMacSapProvider::TransmitPdu (TransmitPduParameters params)
{
  m_mac->DoTransmitPdu (params);
}


void
UeMemberLteMacSapProvider::ReportBufferStatus (ReportBufferStatusParameters params)
{
  m_mac->DoReportBufferStatus (params);
}



/**
 * UeMemberLteUePhySapUser
 */
class UeMemberLteUePhySapUser : public LteUePhySapUser
{
public:
  /**
   * Constructor
   *
   * \param mac the UE MAC
   */
  UeMemberLteUePhySapUser (LteUeMac* mac);

  // inherited from LtePhySapUser
  virtual void ReceivePhyPdu (Ptr<Packet> p);
  virtual void SubframeIndication (uint32_t frameNo, uint32_t subframeNo);
  virtual void ReceiveLteControlMessage (Ptr<LteControlMessage> msg);
  virtual bool Msg3Ready ();
  virtual void UpdateRaRnti (uint32_t raRnti);

private:
  LteUeMac* m_mac; ///< the UE MAC
};

UeMemberLteUePhySapUser::UeMemberLteUePhySapUser (LteUeMac* mac) : m_mac (mac)
{

}

void
UeMemberLteUePhySapUser::ReceivePhyPdu (Ptr<Packet> p)
{
  m_mac->DoReceivePhyPdu (p);
}


void
UeMemberLteUePhySapUser::SubframeIndication (uint32_t frameNo, uint32_t subframeNo)
{
  m_mac->DoSubframeIndication (frameNo, subframeNo);
}

void
UeMemberLteUePhySapUser::ReceiveLteControlMessage (Ptr<LteControlMessage> msg)
{
  m_mac->DoReceiveLteControlMessage (msg);
}

bool
UeMemberLteUePhySapUser::Msg3Ready ()
{
  return m_mac->DoMsg3Ready ();
}

void
UeMemberLteUePhySapUser::UpdateRaRnti (uint32_t raRnti)
{
  m_mac->DoUpdateRaRnti (raRnti);
}



//////////////////////////////////////////////////////////
// LteUeMac methods
///////////////////////////////////////////////////////////


static const int backoffParameterValue[16] = {
  0, 10, 20, 30, 40, 60, 80, 120, 160, 240, 320, 480, 960, 960, 960, 960
}; // 3GPP TS 36321 Tabel 7.2-1

TypeId
LteUeMac::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::LteUeMac")
    .SetParent<Object> ()
    .SetGroupName ("Lte")
    .AddConstructor<LteUeMac> ()
    .AddTraceSource ("TxPrachPreambleStart",
                     "Trace fired when a new preamble transmission is started",
                     MakeTraceSourceAccessor (&LteUeMac::m_preambleTxStartTrace),
                     "ns3::LteUeMac::RachPreambleTxTracedCallback")
    .AddTraceSource ("TxMsg3Start",
                     "Trace fired when a new msg3 is queued for transmission",
                     MakeTraceSourceAccessor (&LteUeMac::m_msg3TxStartTrace),
                     "ns3::LteUeMac::Msg3TxTracedCallback")
    .AddTraceSource ("TimeAlignmentTimerUpdate",
                     "Trace fired when timing advance is received and time alignment timer is restarted",
                     MakeTraceSourceAccessor (&LteUeMac::m_timeAlignmentTimerUpdateTrace),
                     "ns3::LteUeMac::TimeAlignmentTimerTracedCallback")
    .AddTraceSource ("ConnectedOutOfSnycRach",
                     "Trace fired when random access procedure is triggered in connected out of sync state",
                     MakeTraceSourceAccessor (&LteUeMac::m_ConnectedRachStartTrace),
                     "ns3::LteUeMac::ConnectedRachStartTracedCallback")
  ;
  return tid;
}


LteUeMac::LteUeMac ()
  :  m_bsrPeriodicity (MilliSeconds (1)),
    // ideal behavior
    m_bsrLast (MilliSeconds (0)),
    m_freshUlBsr (false),
    m_harqProcessId (0),
    m_rnti (0),
    m_rachConfigured (false),
    m_message3Ready (false),
    m_waitingForRaResponse (false)

{
  NS_LOG_FUNCTION (this);
  m_miUlHarqProcessesPacket.resize (HARQ_PERIOD);
  for (uint8_t i = 0; i < m_miUlHarqProcessesPacket.size (); i++)
    {
      Ptr<PacketBurst> pb = CreateObject <PacketBurst> ();
      m_miUlHarqProcessesPacket.at (i) = pb;
    }
  m_miUlHarqProcessesPacketTimer.resize (HARQ_PERIOD, 0);

  m_macSapProvider = new UeMemberLteMacSapProvider (this);
  m_cmacSapProvider = new UeMemberLteUeCmacSapProvider (this);
  m_uePhySapUser = new UeMemberLteUePhySapUser (this);
  m_raPreambleUniformVariable = CreateObject<UniformRandomVariable> ();
  m_componentCarrierId = 0;
  m_backoffTime = CreateObject<UniformRandomVariable> ();
  m_rachPreambleReady = false;
  m_contention = true; //true for contention based RA and non-contention based RA(only handover case)
  m_uplinkOutOfSync = false; //by default UE is considered to be in sync since it has to accept the 1st RAR
  m_rachStarted = false;
}


LteUeMac::~LteUeMac ()
{
  NS_LOG_FUNCTION (this);
}

void
LteUeMac::DoDispose ()
{
  NS_LOG_FUNCTION (this);
  m_miUlHarqProcessesPacket.clear ();
  delete m_macSapProvider;
  delete m_cmacSapProvider;
  delete m_uePhySapUser;
  Object::DoDispose ();
}


LteUePhySapUser*
LteUeMac::GetLteUePhySapUser (void)
{
  return m_uePhySapUser;
}

void
LteUeMac::SetLteUePhySapProvider (LteUePhySapProvider* s)
{
  m_uePhySapProvider = s;
}


LteMacSapProvider*
LteUeMac::GetLteMacSapProvider (void)
{
  return m_macSapProvider;
}

void
LteUeMac::SetLteUeCmacSapUser (LteUeCmacSapUser* s)
{
  m_cmacSapUser = s;
}

LteUeCmacSapProvider*
LteUeMac::GetLteUeCmacSapProvider (void)
{
  return m_cmacSapProvider;
}

void
LteUeMac::SetComponentCarrierId (uint8_t index)
{
  m_componentCarrierId = index;
}

void
LteUeMac::SetImsi (uint64_t imsi)
{
  m_imsi = imsi;
}

uint64_t
LteUeMac::GetImsi () const
{
  return m_imsi;
}

bool
LteUeMac::DoMsg3Ready (void)
{
  return m_message3Ready;
}

void
LteUeMac::DoUpdateRaRnti (uint32_t raRnti)
{
  NS_LOG_FUNCTION (this);
  m_raRnti = raRnti;
}


void
LteUeMac::DoTransmitPdu (LteMacSapProvider::TransmitPduParameters params)
{
  NS_LOG_FUNCTION (this);
  if (m_uplinkOutOfSync)//stop UL transmission after ue goes out of sync to avoid errors
    {
      NS_LOG_ERROR ("invalid transmission when UE with RNTI " << params.rnti << " is out of sync");
      return;
    }
  NS_ASSERT_MSG (m_rnti == params.rnti, "RNTI mismatch between RLC and MAC");
  LteRadioBearerTag tag (params.rnti, params.lcid, 0 /* UE works in SISO mode*/);
  params.pdu->AddPacketTag (tag);
  // store pdu in HARQ buffer
  if (m_message3Ready) // tx msg3 and start contention resolution timer
    {
      m_miUlHarqProcessesPacket.at ((m_harqProcessId + 1) % HARQ_PERIOD)->AddPacket (params.pdu);
      m_miUlHarqProcessesPacketTimer.at ((m_harqProcessId + 1) % HARQ_PERIOD) = HARQ_PERIOD;
      Time contentionResolutionTimer = MilliSeconds (m_rachConfig.contentionResolutionTimer);
      m_contentionResolutionTimeout = Simulator::Schedule (contentionResolutionTimer, &LteUeMac::ContentionResolutionTimeout, this);
    }
  else
    {
      m_miUlHarqProcessesPacket.at (m_harqProcessId)->AddPacket (params.pdu);
      m_miUlHarqProcessesPacketTimer.at (m_harqProcessId) = HARQ_PERIOD;
    }
  m_uePhySapProvider->SendMacPdu (params.pdu);
}

void
LteUeMac::DoReportBufferStatus (LteMacSapProvider::ReportBufferStatusParameters params)
{
  NS_LOG_FUNCTION (this << (uint32_t) params.lcid);
  if (m_uplinkOutOfSync && !m_rachStarted) //trigger CB RA upon UL data arrival
    {
      NS_LOG_INFO ("UE out of sync, contention based random access started upon UL data arrival");
      m_ConnectedRachStartTrace (m_imsi, m_rnti, "Contention Based Random Access started upon UL data arrival");
      DoStartContentionBasedRandomAccessProcedure ();
      return;
    }

  std::map <uint8_t, LteMacSapProvider::ReportBufferStatusParameters>::iterator it;


  it = m_ulBsrReceived.find (params.lcid);
  if (it != m_ulBsrReceived.end ())
    {
      // update entry
      (*it).second = params;
    }
  else
    {
      m_ulBsrReceived.insert (std::pair<uint8_t, LteMacSapProvider::ReportBufferStatusParameters> (params.lcid, params));
    }
  m_freshUlBsr = true;
}


void
LteUeMac::SendReportBufferStatus (void)
{
  NS_LOG_FUNCTION (this);

  if (m_rnti == 0)
    {
      NS_LOG_INFO ("MAC not initialized, BSR deferred");
      return;
    }

  if (m_ulBsrReceived.size () == 0)
    {
      NS_LOG_INFO ("No BSR report to transmit");
      return;
    }
  MacCeListElement_s bsr;
  bsr.m_rnti = m_rnti;
  bsr.m_macCeType = MacCeListElement_s::BSR;

  // BSR is reported for each LCG
  std::map <uint8_t, LteMacSapProvider::ReportBufferStatusParameters>::iterator it;
  std::vector<uint32_t> queue (4, 0); // one value per each of the 4 LCGs, initialized to 0
  for (it = m_ulBsrReceived.begin (); it != m_ulBsrReceived.end (); it++)
    {
      uint8_t lcid = it->first;
      std::map <uint8_t, LcInfo>::iterator lcInfoMapIt;
      lcInfoMapIt = m_lcInfoMap.find (lcid);
      NS_ASSERT_MSG (lcInfoMapIt !=  m_lcInfoMap.end (), "lcid " << (uint32_t)lcid);
      NS_ASSERT_MSG ((lcid != 0) || (((*it).second.txQueueSize == 0)
                                     && ((*it).second.retxQueueSize == 0)
                                     && ((*it).second.statusPduSize == 0)),
                     "BSR should not be used for LCID 0");
      uint8_t lcg = lcInfoMapIt->second.lcConfig.logicalChannelGroup;
      queue.at (lcg) += ((*it).second.txQueueSize + (*it).second.retxQueueSize + (*it).second.statusPduSize);
    }

  // FF API says that all 4 LCGs are always present
  bsr.m_macCeValue.m_bufferStatus.push_back (BufferSizeLevelBsr::BufferSize2BsrId (queue.at (0)));
  bsr.m_macCeValue.m_bufferStatus.push_back (BufferSizeLevelBsr::BufferSize2BsrId (queue.at (1)));
  bsr.m_macCeValue.m_bufferStatus.push_back (BufferSizeLevelBsr::BufferSize2BsrId (queue.at (2)));
  bsr.m_macCeValue.m_bufferStatus.push_back (BufferSizeLevelBsr::BufferSize2BsrId (queue.at (3)));

  // create the feedback to eNB
  Ptr<BsrLteControlMessage> msg = Create<BsrLteControlMessage> ();
  msg->SetBsr (bsr);
  m_uePhySapProvider->SendLteControlMessage (msg);

}

void
LteUeMac::RandomlySelectAndSendRaPreamble ()
{
  NS_LOG_FUNCTION (this);
  // 3GPP 36.321 5.1.1
  NS_ASSERT_MSG (m_rachConfigured, "RACH not configured");
  // assume that there is no Random Access Preambles group B
  m_raPreambleId = m_raPreambleUniformVariable->GetInteger (0, m_rachConfig.numberOfRaPreambles - 1);
  m_raPreambleStartTime = Simulator::Now ();
  m_rachPreambleReady = true;
}

void
LteUeMac::SendRaPreamble (bool contention)
{
  NS_LOG_FUNCTION (this << (uint32_t) m_raPreambleId << contention);
  // Since regular UL LteControlMessages need m_ulConfigured = true in
  // order to be sent by the UE, the rach preamble needs to be sent
  // with a dedicated primitive (not
  // m_uePhySapProvider->SendLteControlMessage (msg)) so that it can
  // bypass the m_ulConfigured flag. This is reasonable, since In fact
  // the RACH preamble is sent on 6RB bandwidth so the uplink
  // bandwidth does not need to be configured.
  NS_ASSERT (m_subframeNo > 0); // sanity check for subframe starting at 1
  m_raRnti = m_subframeNo - 1;


  Time current = Simulator::Now ();
  NS_LOG_INFO (this << " sent preamble id " << (uint32_t) m_raPreambleId << ", RA-RNTI " << (uint32_t) m_raRnti << " at time " << current.GetSeconds ());

  Ptr<RachPreambleLteControlMessage> msg = Create<RachPreambleLteControlMessage> ();
  msg->SetRapId (m_raPreambleId);
  msg->SetImsi (m_imsi);
  msg->SetStartTime (m_raPreambleStartTime);

  m_uePhySapProvider->SendRachPreamble (msg);

  // fire trace
  m_preambleTxStartTrace (m_imsi);

  // 3GPP 36.321 5.1.4
  Time raWindowBegin = MilliSeconds (3);
  Time raWindowEnd = MilliSeconds (3 + m_rachConfig.raResponseWindowSize);
  //m_waitforRarEvent condition added to cancel this event during reset
  m_waitforRarEvent = Simulator::Schedule (raWindowBegin, &LteUeMac::StartWaitingForRaResponse, this);
  m_noRaResponseReceivedEvent = Simulator::Schedule (raWindowEnd, &LteUeMac::RaResponseTimeout, this, contention);
  m_rachPreambleReady = false;
}

void
LteUeMac::StartWaitingForRaResponse ()
{
  NS_LOG_FUNCTION (this);
  m_waitingForRaResponse = true;
}

void
LteUeMac::RecvRaResponse (BuildRarListElement_s raResponse)
{
  NS_LOG_FUNCTION (this);
  m_waitingForRaResponse = false;
  m_noRaResponseReceivedEvent.Cancel ();
  m_uePhySapProvider->NotifyRarReceived ();
  NS_LOG_INFO ("Got RAR for RAPID " << (uint32_t) m_raPreambleId << ", setting T-C-RNTI = " << raResponse.m_rnti);
  if (m_contention && !m_uplinkOutOfSync) //Set RNTI only when UE is uplink synchronized
    {
      m_rnti = raResponse.m_rnti;
      m_cmacSapUser->SetTemporaryCellRnti (m_rnti);
      m_cmacSapUser->NotifyRarReceived ();
    }

  //Assigning time alignment timer value from RAR msg and starting the timer
  if (m_timeAlignmentTimeoutEvent.IsExpired ())
    {
      m_timeAlignmentTimeoutEvent = Simulator::Schedule (m_timeAlignmentTimer, &LteUeMac::TimeAlignmentTimeout, this);
      NS_LOG_INFO ("TimeAlignmentTimer started/restarted " << m_timeAlignmentTimer);
      m_cmacSapUser->NotifyEnbTimeAlignmentTimerToStart (m_timeAlignmentTimer, m_rnti);
      m_timeAlignmentTimerUpdateTrace (m_imsi, m_rnti, m_timeAlignmentTimer, "Timing advance received in RAR");
    }

  //if RA triggered in UE CONNECTED: out-of-sync state, then ignore T-C-RNTI since UE already has RNTI
  if (m_uplinkOutOfSync)
    {
      if (!m_contention)//if PDCCH order triggered RA in UE CONNECTED state for DL data arrival
        {
          m_contention = true; //set it back to default
          NS_LOG_INFO ("UE back in-sync due to DL data arrival (PDCCH order triggered random access)");
          DoNotifyConnectionSuccessful ();
          return; //RA is considered successful, no need of further messages
        }
      //if UE triggered RA in UE CONNECTED state for UL data arrival
      NS_LOG_INFO ("UE back in-sync due to UL data arrival (UE triggered random access)");
      SendCRntiMacCe (raResponse.m_rnti); //Now, sent on ideal uplink channel;later can be sent on real uplink ctrl channel using RAR UL grant
      Time contentionResolutionTimer = MilliSeconds (m_rachConfig.contentionResolutionTimer);
      m_contentionResolutionTimeout = Simulator::Schedule (contentionResolutionTimer, &LteUeMac::ContentionResolutionTimeout, this);
      m_cmacSapUser->NotifyRarReceived ();//notify RRC that RAR is received for contention based random access triggered in UE CONNECTED state
    }

  // trigger tx opportunity for Message 3 over LC 0
  // this is needed since Message 3's UL GRANT is in the RAR, not in UL-DCIs
  const uint8_t lc0Lcid = 0;
  std::map<uint8_t, LcInfo>::iterator lc0InfoIt = m_lcInfoMap.find (lc0Lcid);
  NS_ASSERT (lc0InfoIt != m_lcInfoMap.end ());
  std::map<uint8_t, LteMacSapProvider::ReportBufferStatusParameters>::iterator lc0BsrIt = m_ulBsrReceived.find (lc0Lcid);
  if ((lc0BsrIt != m_ulBsrReceived.end ()) && (lc0BsrIt->second.txQueueSize > 0))
    {
      NS_ASSERT_MSG (raResponse.m_grant.m_tbSize > lc0BsrIt->second.txQueueSize, "segmentation of Message 3 is not allowed");
      m_message3Ready = true;
      // fire trace to notify that msg 3 is ready!
      m_msg3TxStartTrace (m_imsi);
      // this function can be called only from primary carrier
      if (m_componentCarrierId > 0)
        {
          NS_FATAL_ERROR ("Function called on wrong componentCarrier");
        }
      lc0InfoIt->second.macSapUser->NotifyTxOpportunity (raResponse.m_grant.m_tbSize, 0, 0, m_componentCarrierId, m_rnti, lc0Lcid);
      lc0BsrIt->second.txQueueSize = 0;
    }
}

void
LteUeMac::RaResponseTimeout (bool contention)
{
  NS_LOG_FUNCTION (this << contention);
  m_waitingForRaResponse = false;
  // 3GPP 36.321 5.1.4
  ++m_preambleTransmissionCounter;
  if (m_preambleTransmissionCounter == m_rachConfig.preambleTransMax + 1)
    {
      NS_LOG_INFO ("RAR timeout, preambleTransMax reached => giving up - rnti " << m_rnti);
      m_cmacSapUser->NotifyRandomAccessFailed ();
    }
  else
    {

      double backoffInterval = m_backoffTime->GetValue (0, backoffParameterValue[m_backoffParameter]);
      NS_LOG_INFO ("RAR timeout, re-send preamble after backoff " << backoffInterval);
      // the previous RAR msg need to be purged
      m_uePhySapProvider->DeletePrachPreamble ();
      if (contention)
        {
          m_reSendRaPreambleEvent = Simulator::Schedule (MilliSeconds (backoffInterval), &LteUeMac::RandomlySelectAndSendRaPreamble, this);
        }
      else
        {
          SendRaPreamble (contention);
        }
    }
}

void
LteUeMac::DoConfigureRach (LteUeCmacSapProvider::RachConfig rc)
{
  NS_LOG_FUNCTION (this);
  m_rachConfig = rc;
  m_rachConfigured = true;
  m_uePhySapProvider->ConfigurePrach (rc);
}

void
LteUeMac::DoStartContentionBasedRandomAccessProcedure ()
{
  NS_LOG_FUNCTION (this);

  // 3GPP 36.321 5.1.1
  NS_ASSERT_MSG (m_rachConfigured, "RACH not configured");
  m_preambleTransmissionCounter = 0;
  m_backoffParameter = 0;
  m_rachStarted = true;
  RandomlySelectAndSendRaPreamble ();
}

void
LteUeMac::DoSetRnti (uint16_t rnti)
{
  NS_LOG_FUNCTION (this);
  m_rnti = rnti;
}

void
LteUeMac::DoStartNonContentionBasedRandomAccessProcedure (uint16_t rnti, uint8_t preambleId, uint8_t prachMask)
{
  NS_LOG_FUNCTION (this << " rnti" << rnti);
  NS_ASSERT_MSG (prachMask == 0, "requested PRACH MASK = " << (uint32_t) prachMask << ", but only PRACH MASK = 0 is supported");
  m_rnti = rnti;
  m_raPreambleId = preambleId;
  bool contention = false;
  m_preambleTransmissionCounter = 0;
  m_backoffParameter = 0;
  m_raPreambleStartTime = Simulator::Now (); //start time for ra preamble during NCB RACH
  m_rachStarted = true;
  SendRaPreamble (contention);
}

void
LteUeMac::DoAddLc (uint8_t lcId,  LteUeCmacSapProvider::LogicalChannelConfig lcConfig, LteMacSapUser* msu)
{
  NS_LOG_FUNCTION (this << " lcId" << (uint32_t) lcId);
  NS_ASSERT_MSG (m_lcInfoMap.find (lcId) == m_lcInfoMap.end (), "cannot add channel because LCID " << lcId << " is already present");

  LcInfo lcInfo;
  lcInfo.lcConfig = lcConfig;
  lcInfo.macSapUser = msu;
  m_lcInfoMap[lcId] = lcInfo;
}

void
LteUeMac::DoRemoveLc (uint8_t lcId)
{
  NS_LOG_FUNCTION (this << " lcId" << lcId);
  NS_ASSERT_MSG (m_lcInfoMap.find (lcId) != m_lcInfoMap.end (), "could not find LCID " << lcId);
  m_lcInfoMap.erase (lcId);
}

void
LteUeMac::DoReset ()
{
  NS_LOG_FUNCTION (this);
  std::map<uint8_t, LcInfo>::iterator it = m_lcInfoMap.begin ();
  while (it != m_lcInfoMap.end ())
    {
      // don't delete CCCH)
      if (it->first == 0)
        {
          ++it;
        }
      else
        {
          // note: use of postfix operator preserves validity of iterator
          m_lcInfoMap.erase (it++);
        }
    }

  m_noRaResponseReceivedEvent.Cancel ();
  m_reSendRaPreambleEvent.Cancel ();
  m_contentionResolutionTimeout.Cancel ();
  m_rachConfigured = false;
  m_freshUlBsr = false;
  m_ulBsrReceived.clear ();
  m_message3Ready = false;

  //additional data to be cleared
  m_rachPreambleReady = false;
  m_waitforRarEvent.Cancel ();
  m_rnti = 0;
  m_timeAlignmentTimeoutEvent.Cancel ();
  m_waitingForRaResponse = false;
  m_harqProcessId = 0;
  m_miUlHarqProcessesPacket.clear ();
  m_miUlHarqProcessesPacket.resize (HARQ_PERIOD);
  for (uint8_t i = 0; i < m_miUlHarqProcessesPacket.size (); i++)
    {
      Ptr<PacketBurst> pb = CreateObject<PacketBurst> ();
      m_miUlHarqProcessesPacket.at (i) = pb;
    }
  m_miUlHarqProcessesPacketTimer.resize (HARQ_PERIOD, 0);
  m_contention = true;
  m_uplinkOutOfSync = false;
  m_rachStarted = false;
}

void
LteUeMac::DoNotifyConnectionExpired ()
{
  DoReset ();
}

void
LteUeMac::DoNotifyConnectionSuccessful ()
{
  NS_LOG_FUNCTION (this);
  NS_LOG_INFO ("Random Access successful");
  m_message3Ready = false;
  m_contentionResolutionTimeout.Cancel ();
  m_uplinkOutOfSync = false;
  m_rachStarted = false;
  m_uePhySapProvider->NotifyConnectionSuccessful (); //notify the PHY that RRC connection was successfully established
}

void
LteUeMac::DoReceivePhyPdu (Ptr<Packet> p)
{
  NS_LOG_FUNCTION (this);
  LteRadioBearerTag tag;
  p->RemovePacketTag (tag);
  if (tag.GetRnti () == m_rnti)
    {
      // packet is for the current user
      std::map <uint8_t, LcInfo>::const_iterator it = m_lcInfoMap.find (tag.GetLcid ());
      if (it != m_lcInfoMap.end ())
        {
          it->second.macSapUser->ReceivePdu (p, m_rnti, tag.GetLcid ());
        }
      else
        {
          NS_LOG_WARN ("received packet with unknown lcid " << (uint32_t) tag.GetLcid ());
        }
    }
}


void
LteUeMac::DoReceiveLteControlMessage (Ptr<LteControlMessage> msg)
{
  NS_LOG_FUNCTION (this);
  if (msg->GetMessageType () == LteControlMessage::UL_DCI)
    {
      Ptr<UlDciLteControlMessage> msg2 = DynamicCast<UlDciLteControlMessage> (msg);
      UlDciListElement_s dci = msg2->GetDci ();
      if (dci.m_ndi == 1)
        {
          // New transmission -> empty pkt buffer queue (for deleting eventual pkts not acked )
          Ptr<PacketBurst> pb = CreateObject <PacketBurst> ();
          m_miUlHarqProcessesPacket.at (m_harqProcessId) = pb;
          // Retrieve data from RLC
          std::map <uint8_t, LteMacSapProvider::ReportBufferStatusParameters>::iterator itBsr;
          uint16_t activeLcs = 0;
          uint32_t statusPduMinSize = 0;
          for (itBsr = m_ulBsrReceived.begin (); itBsr != m_ulBsrReceived.end (); itBsr++)
            {
              if (((*itBsr).second.statusPduSize > 0) || ((*itBsr).second.retxQueueSize > 0) || ((*itBsr).second.txQueueSize > 0))
                {
                  activeLcs++;
                  if (((*itBsr).second.statusPduSize != 0)&&((*itBsr).second.statusPduSize < statusPduMinSize))
                    {
                      statusPduMinSize = (*itBsr).second.statusPduSize;
                    }
                  if (((*itBsr).second.statusPduSize != 0)&&(statusPduMinSize == 0))
                    {
                      statusPduMinSize = (*itBsr).second.statusPduSize;
                    }
                }
            }
          if (activeLcs == 0)
            {
              NS_LOG_ERROR (this << " No active flows for this UL-DCI");
              return;
            }
          std::map <uint8_t, LcInfo>::iterator it;
          uint32_t bytesPerActiveLc = dci.m_tbSize / activeLcs;
          bool statusPduPriority = false;
          if ((statusPduMinSize != 0)&&(bytesPerActiveLc < statusPduMinSize))
            {
              // send only the status PDU which has highest priority
              statusPduPriority = true;
              NS_LOG_DEBUG (this << " Reduced resource -> send only Status, bytes " << statusPduMinSize);
              if (dci.m_tbSize < statusPduMinSize)
                {
                  NS_FATAL_ERROR ("Insufficient Tx Opportunity for sending a status message");
                }
            }
          NS_LOG_LOGIC (this << " UE " << m_rnti << ": UL-CQI notified TxOpportunity of " << dci.m_tbSize << " => " << bytesPerActiveLc << " bytes per active LC" << " statusPduMinSize " << statusPduMinSize);
          for (it = m_lcInfoMap.begin (); it != m_lcInfoMap.end (); it++)
            {
              itBsr = m_ulBsrReceived.find ((*it).first);
              NS_LOG_DEBUG (this << " Processing LC " << (uint32_t)(*it).first << " bytesPerActiveLc " << bytesPerActiveLc);
              if ( (itBsr != m_ulBsrReceived.end ())
                   && ( ((*itBsr).second.statusPduSize > 0)
                        || ((*itBsr).second.retxQueueSize > 0)
                        || ((*itBsr).second.txQueueSize > 0)) )
                {
                  if ((statusPduPriority) && ((*itBsr).second.statusPduSize == statusPduMinSize))
                    {
                      (*it).second.macSapUser->NotifyTxOpportunity ((*itBsr).second.statusPduSize, 0, 0, m_componentCarrierId, m_rnti, (*it).first);
                      NS_LOG_LOGIC (this << "\t" << bytesPerActiveLc << " send  " << (*itBsr).second.statusPduSize << " status bytes to LC " << (uint32_t)(*it).first << " statusQueue " << (*itBsr).second.statusPduSize << " retxQueue" << (*itBsr).second.retxQueueSize << " txQueue" <<  (*itBsr).second.txQueueSize);
                      (*itBsr).second.statusPduSize = 0;
                      break;
                    }
                  else
                    {
                      uint32_t bytesForThisLc = bytesPerActiveLc;
                      NS_LOG_LOGIC (this << "\t" << bytesPerActiveLc << " bytes to LC " << (uint32_t)(*it).first << " statusQueue " << (*itBsr).second.statusPduSize << " retxQueue" << (*itBsr).second.retxQueueSize << " txQueue" <<  (*itBsr).second.txQueueSize);
                      if (((*itBsr).second.statusPduSize > 0) && (bytesForThisLc > (*itBsr).second.statusPduSize))
                        {
                          (*it).second.macSapUser->NotifyTxOpportunity ((*itBsr).second.statusPduSize, 0, 0, m_componentCarrierId, m_rnti, (*it).first);
                          bytesForThisLc -= (*itBsr).second.statusPduSize;
                          NS_LOG_DEBUG (this << " serve STATUS " << (*itBsr).second.statusPduSize);
                          (*itBsr).second.statusPduSize = 0;
                        }
                      else
                        {
                          if ((*itBsr).second.statusPduSize > bytesForThisLc)
                            {
                              NS_FATAL_ERROR ("Insufficient Tx Opportunity for sending a status message");
                            }
                        }

                      if ((bytesForThisLc > 7)    // 7 is the min TxOpportunity useful for Rlc
                          && (((*itBsr).second.retxQueueSize > 0)
                              || ((*itBsr).second.txQueueSize > 0)))
                        {
                          if ((*itBsr).second.retxQueueSize > 0)
                            {
                              NS_LOG_DEBUG (this << " serve retx DATA, bytes " << bytesForThisLc);
                              (*it).second.macSapUser->NotifyTxOpportunity (bytesForThisLc, 0, 0, m_componentCarrierId, m_rnti, (*it).first);
                              if ((*itBsr).second.retxQueueSize >= bytesForThisLc)
                                {
                                  (*itBsr).second.retxQueueSize -= bytesForThisLc;
                                }
                              else
                                {
                                  (*itBsr).second.retxQueueSize = 0;
                                }
                            }
                          else if ((*itBsr).second.txQueueSize > 0)
                            {
                              uint16_t lcid = (*it).first;
                              uint32_t rlcOverhead;
                              if (lcid == 1)
                                {
                                  // for SRB1 (using RLC AM) it's better to
                                  // overestimate RLC overhead rather than
                                  // underestimate it and risk unneeded
                                  // segmentation which increases delay
                                  rlcOverhead = 4;
                                }
                              else
                                {
                                  // minimum RLC overhead due to header
                                  rlcOverhead = 2;
                                }
                              NS_LOG_DEBUG (this << " serve tx DATA, bytes " << bytesForThisLc << ", RLC overhead " << rlcOverhead);
                              (*it).second.macSapUser->NotifyTxOpportunity (bytesForThisLc, 0, 0, m_componentCarrierId, m_rnti, (*it).first);
                              if ((*itBsr).second.txQueueSize >= bytesForThisLc - rlcOverhead)
                                {
                                  (*itBsr).second.txQueueSize -= bytesForThisLc - rlcOverhead;
                                }
                              else
                                {
                                  (*itBsr).second.txQueueSize = 0;
                                }
                            }
                        }
                      else
                        {
                          if ( ((*itBsr).second.retxQueueSize > 0) || ((*itBsr).second.txQueueSize > 0))
                            {
                              // resend BSR info for updating eNB peer MAC
                              m_freshUlBsr = true;
                            }
                        }
                      NS_LOG_LOGIC (this << "\t" << bytesPerActiveLc << "\t new queues " << (uint32_t)(*it).first << " statusQueue " << (*itBsr).second.statusPduSize << " retxQueue" << (*itBsr).second.retxQueueSize << " txQueue" <<  (*itBsr).second.txQueueSize);
                    }

                }
            }
        }
      else
        {
          // HARQ retransmission -> retrieve data from HARQ buffer
          NS_LOG_DEBUG (this << " UE MAC RETX HARQ " << (uint16_t)m_harqProcessId);
          Ptr<PacketBurst> pb = m_miUlHarqProcessesPacket.at (m_harqProcessId);
          for (std::list<Ptr<Packet> >::const_iterator j = pb->Begin (); j != pb->End (); ++j)
            {
              Ptr<Packet> pkt = (*j)->Copy ();
              m_uePhySapProvider->SendMacPdu (pkt);
            }
          if (m_message3Ready) // start contention resolution timer if this is an harq of msg3
            {
              Time contentionResolutionTimer = MilliSeconds (m_rachConfig.contentionResolutionTimer);
              m_contentionResolutionTimeout.Cancel ();
              m_contentionResolutionTimeout = Simulator::Schedule (contentionResolutionTimer, &LteUeMac::ContentionResolutionTimeout, this);
            }
          m_miUlHarqProcessesPacketTimer.at (m_harqProcessId) = HARQ_PERIOD;
        }

    }
  else if (msg->GetMessageType () == LteControlMessage::RAR)
    {
      Ptr<RarLteControlMessage> rarMsg = DynamicCast<RarLteControlMessage> (msg);
      uint16_t raRnti = rarMsg->GetRaRnti ();
      m_backoffParameter = rarMsg->GetBackoffIndicator ();
      NS_LOG_INFO (this << "got RAR with RA-RNTI " << (uint32_t) raRnti << ", expecting " << (uint32_t) m_raRnti <<
                   " updated m_backoffParameter to " << m_backoffParameter);

      if (m_waitingForRaResponse)
        {
          if (raRnti == m_raRnti) // RAR corresponds to TX subframe of preamble
            {
              for (std::list<RarLteControlMessage::Rar>::const_iterator it = rarMsg->RarListBegin ();
                   it != rarMsg->RarListEnd ();
                   ++it)
                {
                  if (it->rapId == m_raPreambleId) // RAR is for me
                    {
                      RecvRaResponse (it->rarPayload);
                      /// \todo RRC generates the RecvRaResponse messaged
                      /// for avoiding holes in transmission at PHY layer
                      /// (which produce erroneous UL CQI evaluation)
                    }
                }
            }
        }
    }
  //Restart time alignment timer as time advance command is received
  else if (msg->GetMessageType () == LteControlMessage::TAC)
    {
      NS_LOG_INFO ("Timing advance command received");
      Ptr<TacLteControlMessage> tacMsg = DynamicCast<TacLteControlMessage> (msg);
      m_timeAlignmentTimeoutEvent.Cancel ();
      uint16_t DynamicTimer = tacMsg->GetTimeAlignmentTimer ();
      if (DynamicTimer != 0)
        {
          m_timeAlignmentTimer = MilliSeconds (DynamicTimer);
        }
      m_timeAlignmentTimeoutEvent = Simulator::Schedule (m_timeAlignmentTimer, &LteUeMac::TimeAlignmentTimeout, this);
      NS_LOG_INFO ("TimeAlignmentTimer started/restarted by UE: " << m_imsi << " timer value: " << m_timeAlignmentTimer);
      m_timeAlignmentTimerUpdateTrace (m_imsi, m_rnti, m_timeAlignmentTimer, "Timing advance received in TAC MAC CE");
      m_cmacSapUser->NotifyEnbTimeAlignmentTimerToStart (m_timeAlignmentTimer, m_rnti);
    }
  else if (msg->GetMessageType () == LteControlMessage::DL_DCI) //PDCCH order reception
    {
      NS_LOG_INFO ("PDCCH ORDER DL_DCI received");
      Ptr<DlDciLteControlMessage> msg2 = DynamicCast<DlDciLteControlMessage> (msg);
      DlDciListElement_s dci = msg2->GetDci ();
      if (m_uplinkOutOfSync && !m_rachStarted)
        {
          m_ConnectedRachStartTrace (m_imsi, dci.m_rnti, "Non Contention Based Random Access started upon DL data arrival");
          m_contention = false; //indicates non Contention Based RandomAccessProcedure triggered by enodeb with PDCCH order
          DoStartNonContentionBasedRandomAccessProcedure (dci.m_rnti, dci.m_preambleIndex, dci.m_prachMaskIndex);
        }

    }
  else if (msg->GetMessageType () == LteControlMessage::CRI && m_contentionResolutionTimeout.IsRunning ()) //contention resolution identity
    {
      NS_LOG_INFO ("contention resolution msg received");
      // this function can be called only from primary carrier
      if (m_componentCarrierId > 0)
        {
          NS_FATAL_ERROR ("Contention Resolution message received on wrong componentCarrier");
        }
      Ptr<CriLteControlMessage> crMsg = DynamicCast<CriLteControlMessage> (msg);
      if (crMsg->GetUeContentionResolutionIdentity () == m_imsi)
        {
          m_cmacSapUser->NotifyContentionResolutionMsgReceived ();
        }
    }
  else
    {
      NS_LOG_WARN (this << " LteControlMessage not recognized");
    }
}

void
LteUeMac::ContentionResolutionTimeout (void)
{
  NS_LOG_FUNCTION (this);
  // the maximum number of retx for msg3 has been reached and
  // as the standard asks the ra procedure has to begin once again
  // however in this implementation msg3 piggybacks RrcConnectionRequest
  // therefore the new ra procedure must be triggered by rrc
  m_noRaResponseReceivedEvent.Cancel ();
  m_contentionResolutionTimeout.Cancel ();
  //notify rrc about contention resolution timeout
  m_cmacSapUser->NotifyContentionResolutionTimeout ();
  m_uePhySapProvider->NotifyConnectionExpired (); //reset phy

  if (!m_uplinkOutOfSync)
    {
      m_rnti = 0; //reset RNTI only if the UE is uplink in sync (initial random access case)

    }
  m_timeAlignmentTimeoutEvent.Cancel (); //cancel time alignment timer when contention resolution timer expires
  m_message3Ready = false;
  m_waitingForRaResponse = false;

  // 3GPP 36.321 5.1.5
  ++m_preambleTransmissionCounter;
  if (m_preambleTransmissionCounter == m_rachConfig.preambleTransMax + 1)
    {
      NS_LOG_INFO ("ContentionResolutionTimeout timeout, preambleTransMax reached => giving up - rnti " << m_rnti);
      m_cmacSapUser->NotifyRandomAccessFailed ();
    }
  else
    {
      double backoffInterval = m_backoffTime->GetValue (0, backoffParameterValue[m_backoffParameter]);
      NS_LOG_INFO ("ContentionResolutionTimeout timeout, re-send preamble after backoff " << backoffInterval);
      m_reSendRaPreambleEvent = Simulator::Schedule (MilliSeconds (backoffInterval), &LteUeMac::RandomlySelectAndSendRaPreamble, this);
    }

}

void
LteUeMac::RefreshHarqProcessesPacketBuffer (void)
{
  NS_LOG_FUNCTION (this);

  for (uint16_t i = 0; i < m_miUlHarqProcessesPacketTimer.size (); i++)
    {
      if (m_miUlHarqProcessesPacketTimer.at (i) == 0)
        {
          if (m_miUlHarqProcessesPacket.at (i)->GetSize () > 0)
            {
              // timer expired: drop packets in buffer for this process
              NS_LOG_INFO (this << " HARQ Proc Id " << i << " packets buffer expired");
              Ptr<PacketBurst> emptyPb = CreateObject <PacketBurst> ();
              m_miUlHarqProcessesPacket.at (i) = emptyPb;
            }
        }
      else
        {
          m_miUlHarqProcessesPacketTimer.at (i)--;
        }
    }
}


void
LteUeMac::DoSubframeIndication (uint32_t frameNo, uint32_t subframeNo)
{
  NS_LOG_FUNCTION (this);
  m_frameNo = frameNo;
  m_subframeNo = subframeNo;
  RefreshHarqProcessesPacketBuffer ();
  if (m_rachPreambleReady)
    {
      uint32_t next_subframeNo = subframeNo;
      uint32_t next_frameNo = frameNo;

      if (subframeNo + 1 > 10)
        {
          next_frameNo = frameNo + 1;
          next_subframeNo = 1;
        }
      else
        {
          next_subframeNo = subframeNo + 1;
        }

      if (LtePrachInfo::IsPrachSff (next_frameNo, next_subframeNo))
      // in the next frame send prach
        {
          NS_LOG_INFO ("At frame " << frameNo << " subframe " << subframeNo << " call SendRaPreamble");
          SendRaPreamble (true);
        }
    }
  if ((Simulator::Now () >= m_bsrLast + m_bsrPeriodicity) && (m_freshUlBsr == true))
    {
      SendReportBufferStatus ();
      m_bsrLast = Simulator::Now ();
      m_freshUlBsr = false;
    }
  m_harqProcessId = (m_harqProcessId + 1) % HARQ_PERIOD;

}

int64_t
LteUeMac::AssignStreams (int64_t stream)
{
  NS_LOG_FUNCTION (this << stream);
  m_raPreambleUniformVariable->SetStream (stream);
  m_backoffTime->SetStream (stream);
  return 1;
}


void
LteUeMac::TimeAlignmentTimeout ()
{
  NS_LOG_FUNCTION (this << m_rnti);
  m_timeAlignmentTimeoutEvent.Cancel ();
  if (!m_cmacSapUser->NotifyTimeAlignmentTimeout ())
    {
      return;
    }
  m_uplinkOutOfSync = true;

  //data to be cleared
  m_noRaResponseReceivedEvent.Cancel ();  //RAR timer
  m_waitforRarEvent.Cancel ();
  m_reSendRaPreambleEvent.Cancel ();  //resend preamble after backoff event
  m_contentionResolutionTimeout.Cancel ();
  m_freshUlBsr = false;
  m_ulBsrReceived.clear ();  //clear ul bsr reports
  m_message3Ready = false;
  m_rachPreambleReady = false;
  m_waitingForRaResponse = false;
  m_rachStarted = false;

  m_harqProcessId = 0;
  m_miUlHarqProcessesPacket.clear ();  //clear harq packets
  m_miUlHarqProcessesPacket.resize (HARQ_PERIOD);
  for (uint8_t i = 0; i < m_miUlHarqProcessesPacket.size (); i++)
    {
      Ptr<PacketBurst> pb = CreateObject<PacketBurst> ();
      m_miUlHarqProcessesPacket.at (i) = pb;
    }
  m_miUlHarqProcessesPacketTimer.resize (HARQ_PERIOD, 0);  //reset harq timer

}

void
LteUeMac::DoSetTimeAlignmentTimer (uint16_t timeAlignmentTimerCommon)
{
  m_timeAlignmentTimer = MilliSeconds (timeAlignmentTimerCommon);
}

void
LteUeMac::SendCRntiMacCe (uint16_t tempRnti)
{
  NS_LOG_FUNCTION (this << m_rnti);
  NS_LOG_INFO ("temp RNTI: " << tempRnti);
  MacCeListElement_s ce;
  ce.m_macCeType = MacCeListElement_s::CRNTI;
  ce.m_rnti = m_rnti;
  Ptr<CRntiLteControlMessage> msg = Create<CRntiLteControlMessage> ();
  msg->SetCRnti (ce);
  msg->SetTempRnti (tempRnti);
  m_uePhySapProvider->SendLteControlMessage (msg);
}

} // namespace ns3
