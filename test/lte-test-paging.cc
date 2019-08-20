/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
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
 * Author: Vignesh Babu <ns3-dev@esk.fraunhofer.de>
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/lte-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/config-store-module.h"
#include "ns3/config-store.h"
#include <iostream>
#include <vector>
#include <stdio.h>
#include <iomanip>
#include <algorithm>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("LtePagingTest");

/**
 * \ingroup lte
 *
 * \brief Test the paging procedure which
 * enables the UE to transition from idle to connected state
 * during downlink data transfer
 */
class LtePagingTestCase : public TestCase
{
public:

  /**
   * \brief Creates an instance of the paging test case.
   * \param nEnbs number of eNBs in the test
   * \param nUes number of UEs in the test
   * \param isIdealRrc if true, simulation uses Ideal RRC protocol, otherwise
   *                   simulation uses Real RRC protocol
   * \param uePositionList the list containing the initial position of the UEs
   * \param ueToBePagedList the list of UEs to be paged
   * \param pagingSuccessList the list indicates if paging will succeed or not for each UE
   */
  LtePagingTestCase (uint32_t nEnbs, uint32_t nUes, bool isIdealRrc,
                     std::vector<Vector> uePositionList, std::vector<uint32_t> ueToBePagedList,
                     std::vector<bool> pagingSuccessList);

private:

  /**
   * Build name string
   * \param nEnbs number of eNBs in the test
   * \param nUes number of UEs in the test
   * \param isIdealRrc true if the ideal RRC is used
   * \returns the name string
   */
  static std::string BuildNameString (uint32_t nEnbs, uint32_t nUes,
                                      bool isIdealRrc);

  /**
   * \brief Setup the simulation according to the configuration set by the
   *        class constructor, run it, and verify the result.
   */
  virtual void DoRun ();

  /**
   * Check connected function
   * \param ueDevice the UE device
   * \param enbDevices the list of ENB devices
   */
  void CheckConnected (Ptr<NetDevice> ueDevice, NetDeviceContainer enbDevices);

  /**
   * Check if the UE is in idle state
   * \param ueDevice the UE device
   * \param enbDevices the list of ENB devices
   */
  void CheckIdle (Ptr<NetDevice> ueDevice, NetDeviceContainer enbDevices);

  /**
   * \brief State transition callback function
   * \param context the context string
   * \param imsi the IMSI
   * \param cellId the cell ID
   * \param rnti the RNTI
   * \param oldState the old state
   * \param newState the new state
   */
  void UeStateTransitionCallback (std::string context, uint64_t imsi,
                                uint16_t cellId, uint16_t rnti,
                                LteUeRrc::State oldState, LteUeRrc::State newState);

  /**
   * \brief Connection established at UE callback function
   * \param context the context string
   * \param imsi the IMSI
   * \param cellId the cell ID
   * \param rnti the RNTI
   */
  void ConnectionEstablishedUeCallback (std::string context, uint64_t imsi,
                                      uint16_t cellId, uint16_t rnti);

  /**
   * \brief Connection established at eNodeB callback function
   * \param context the context string
   * \param imsi the IMSI
   * \param cellId the cell ID
   * \param rnti the RNTI
   */
  void ConnectionEstablishedEnbCallback (std::string context, uint64_t imsi,
                                      uint16_t cellId, uint16_t rnti);

  /**
   * \brief SIB 2 received by UE  callback function
   * \param context the context string
   * \param imsi the IMSI
   * \param cellId the cell ID
   * \param rnti the RNTI
   */
  void Sib2ReceivedCallback (std::string context, uint64_t imsi,
                             uint16_t cellId, uint16_t rnti);

  /**
   * \brief Paging message sent by eNodeB to a particular UE callback function
   * \param context the context string
   * \param imsi the IMSI of the UE which is paged
   * \param pagingFrame the frame in which paging message is sent
   * \param pagingOccasion the subframe in which paging message is sent
   */
  void PagingMessageSentCallback (std::string context, uint64_t imsi,
                                  uint16_t pagingFrame, uint16_t pagingOccasion);

  uint32_t m_nEnbs; ///< number of eNBs in the test
  uint32_t m_nUes; ///< number of UEs in the test
  bool m_isIdealRrc; ///< whether the LTE is configured to use ideal RRC
  std::vector<Vector> m_uePositionList;///< the list containing the initial position of the UEs
  std::vector<uint32_t> m_ueToBePagedList; ///< the list of UEs to be paged
  std::vector<bool> m_pagingSuccessList; ///< the list indicates if paging will succeed or not for each UE
  std::list<uint64_t> m_sib2ReceivedList; ///< list of UEs which have received SIB 2

 /// Paging Info structure
  struct PagingInfo
  {
    uint16_t pagingFrame;
    uint16_t pagingOccasion;
  };

  /// the list of paging frames and paging occasions for each UE
  std::map<uint64_t, PagingInfo> m_pagingInfoList;
  uint16_t m_pagingCycle; ///< the default paging cycle

}; // end of class LtePagingTestCase

std::string
LtePagingTestCase::BuildNameString (uint32_t nEnbs, uint32_t nUes,
                                                bool isIdealRrc)
{
  std::ostringstream oss;
  oss << "nEnbs=" << nEnbs
      << " nUes=" << nUes;
  if (isIdealRrc)
    {
      oss << ", ideal RRC";
    }
  else
    {
      oss << ", real RRC";
    }
  return oss.str ();
}

LtePagingTestCase::LtePagingTestCase (uint32_t nEnbs, uint32_t nUes, bool isIdealRrc,
                     std::vector<Vector> uePositionList, std::vector<uint32_t> ueToBePagedList,
                     std::vector<bool> pagingSuccessList)
  : TestCase (BuildNameString (nEnbs, nUes, isIdealRrc)),
    m_nEnbs (nEnbs),
    m_nUes (nUes),
    m_isIdealRrc (isIdealRrc),
    m_uePositionList (uePositionList),
    m_ueToBePagedList(ueToBePagedList),
    m_pagingSuccessList(pagingSuccessList)
{
  m_pagingCycle=0;
}

void
LtePagingTestCase::DoRun ()
{
  NS_LOG_FUNCTION (this << GetName ());

  uint16_t numBearersPerUe = 1;
  double simTime = 5;
  double eNodeB_txPower = 43;
  double interSiteDistance = 500;

  Config::SetDefault ("ns3::LteHelper::UseIdealRrc", BooleanValue (m_isIdealRrc));
   if (!m_isIdealRrc)
     {
       Config::SetDefault ("ns3::LteHelper::UseIdealPrach", BooleanValue (false));
     }
   else
     {
       Config::SetDefault ("ns3::LteHelper::UseIdealPrach", BooleanValue (true));
     }
  //Paging parameters
  //T3413 timer (Note: value should be equal to or greater than paging cycle)
  Config::SetDefault ("ns3::EpcMme::T3413", TimeValue (Seconds (3))); //3s
  Config::SetDefault ("ns3::EpcSgwPgwApplication::MaxDlBufferSize", UintegerValue (10240)); //10*1024
  Config::SetDefault ("ns3::LteEnbRrc::DefaultPagingCycle", EnumValue (LteRrcSap::PcchConfig::RF_32)); //320ms
  Config::SetDefault ("ns3::LteEnbRrc::NB", EnumValue (LteRrcSap::PcchConfig::FOUR_T)); //4*T, T: paging cycle

  m_pagingCycle=32;//32 frames, value should be equal to LteEnbRrc::DefaultPagingCycle

  Ptr<LteHelper> lteHelper = CreateObject<LteHelper> ();
  Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper> ();
  lteHelper->SetEpcHelper (epcHelper);

  lteHelper->SetPathlossModelType (TypeId::LookupByName ("ns3::LogDistancePropagationLossModel"));
  lteHelper->SetPathlossModelAttribute ("Exponent", DoubleValue (3.9));
  lteHelper->SetPathlossModelAttribute ("ReferenceLoss", DoubleValue (38.57)); //ref. loss in dB at 1m for 2.025GHz
  lteHelper->SetPathlossModelAttribute ("ReferenceDistance", DoubleValue (1));

  //----power related (equal for all base stations)----
  Config::SetDefault ("ns3::LteEnbPhy::TxPower", DoubleValue (eNodeB_txPower));
  Config::SetDefault ("ns3::LteUePhy::TxPower", DoubleValue (23));
  Config::SetDefault ("ns3::LteUePhy::NoiseFigure", DoubleValue (7));
  Config::SetDefault ("ns3::LteEnbPhy::NoiseFigure", DoubleValue (2));
  Config::SetDefault ("ns3::LteUePhy::EnableUplinkPowerControl", BooleanValue (true));
  Config::SetDefault ("ns3::LteUePowerControl::ClosedLoop", BooleanValue (true));
  Config::SetDefault ("ns3::LteUePowerControl::AccumulationEnabled", BooleanValue (true));

  lteHelper->SetHandoverAlgorithmType ("ns3::A3RsrpHandoverAlgorithm");
  lteHelper->SetHandoverAlgorithmAttribute ("Hysteresis", DoubleValue (3.0)); //Handover margin (0-15 dB range).MAX=15 dB
  lteHelper->SetHandoverAlgorithmAttribute ("TimeToTrigger", TimeValue (MilliSeconds (256)));

  //----frequency related----
  lteHelper->SetEnbDeviceAttribute ("DlEarfcn", UintegerValue (100)); //2120MHz
  lteHelper->SetEnbDeviceAttribute ("UlEarfcn", UintegerValue (18100)); //1930MHz
  lteHelper->SetEnbDeviceAttribute ("DlBandwidth", UintegerValue (25)); //5MHz
  lteHelper->SetEnbDeviceAttribute ("UlBandwidth", UintegerValue (25)); //5MHz

  //----others----
  lteHelper->SetSchedulerType ("ns3::PfFfMacScheduler");
  Config::SetDefault ("ns3::LteAmc::AmcModel", EnumValue (LteAmc::PiroEW2010));
  Config::SetDefault ("ns3::LteAmc::Ber", DoubleValue (0.01));
  Config::SetDefault ("ns3::PfFfMacScheduler::HarqEnabled", BooleanValue (true));

  Config::SetDefault ("ns3::LteEnbMac::DynamicTimerConfiguration", BooleanValue (false));
  Config::SetDefault ("ns3::LteEnbRrc::TimeAlignmentTimer", UintegerValue (10240));


   // Create the internet
  Ptr<Node> pgw = epcHelper->GetPgwNode ();
  // Create a single RemoteHost0x18ab460
  NodeContainer remoteHostContainer;
  remoteHostContainer.Create (1);
  Ptr<Node> remoteHost = remoteHostContainer.Get (0);
  InternetStackHelper internet;
  internet.Install (remoteHostContainer);
  PointToPointHelper p2ph;
  p2ph.SetDeviceAttribute ("DataRate", DataRateValue (DataRate ("100Gb/s")));
  p2ph.SetDeviceAttribute ("Mtu", UintegerValue (1500));
  p2ph.SetChannelAttribute ("Delay", TimeValue (Seconds (0.010)));
  NetDeviceContainer internetDevices = p2ph.Install (pgw, remoteHost);
  Ipv4AddressHelper ipv4h;
  ipv4h.SetBase ("1.0.0.0", "255.0.0.0");
  Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign (internetDevices);
//  Ipv4Address remoteHostAddr = internetIpIfaces.GetAddress (1);
  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  Ptr<Ipv4StaticRouting> remoteHostStaticRouting = ipv4RoutingHelper.GetStaticRouting (remoteHost->GetObject<
      Ipv4> ());
  remoteHostStaticRouting->AddNetworkRouteTo (Ipv4Address ("7.0.0.0"), Ipv4Mask ("255.0.0.0"), 1);

  // Create Nodes: eNodeB and UE
  NodeContainer enbNodes;
  NodeContainer ueNodes;
  enbNodes.Create (m_nEnbs);
  ueNodes.Create (m_nUes);

  //Mobility
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();

  for (uint16_t i = 0; i < m_nEnbs; i++)
    {
      positionAlloc->Add (Vector (interSiteDistance * i, 0, 0));
    }
  MobilityHelper mobility;
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.SetPositionAllocator (positionAlloc);
  mobility.Install (enbNodes);

  Ptr<ListPositionAllocator> positionAlloc1 = CreateObject<ListPositionAllocator> ();

  for (std::vector<Vector>::iterator posIt=m_uePositionList.begin();
      posIt!=m_uePositionList.end();++posIt)
    {
  positionAlloc1->Add (*posIt);
    }

  mobility.SetPositionAllocator (positionAlloc1);
  mobility.SetMobilityModel ("ns3::ConstantVelocityMobilityModel");
  mobility.Install (ueNodes);

  // Install LTE Devices in eNB and UEs
  NetDeviceContainer enbDevs;
  NetDeviceContainer ueDevs;

  enbDevs = lteHelper->InstallEnbDevice (enbNodes);
  ueDevs = lteHelper->InstallUeDevice (ueNodes);

  // Install the IP stack on the UEs
  internet.Install (ueNodes);
  Ipv4InterfaceContainer ueIpIfaces;
  ueIpIfaces = epcHelper->AssignUeIpv4Address (NetDeviceContainer (ueDevs));

  // Attach a UE to a eNB
  lteHelper->Attach (ueDevs);

  // Install and start applications on UEs and remote host
  uint16_t dlPort = 10000;
  uint16_t ulPort = 20000;
  Ptr<UniformRandomVariable> startTimeSeconds = CreateObject<
      UniformRandomVariable> ();
  startTimeSeconds->SetAttribute ("Min", DoubleValue (0));
  startTimeSeconds->SetAttribute ("Max", DoubleValue (0.010));
  double interPacketInterval = 1000;

  for (uint32_t u = 0; u < m_nUes; ++u)
    {
      Ptr<Node> ue = ueNodes.Get (u);
      // Set the default gateway for the UE
      Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (ue->GetObject<
          Ipv4> ());
      ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);
    }

  //install applications on only the UEs which have to be paged
  for (std::vector<uint32_t>::iterator ueIndexIt=m_ueToBePagedList.begin();
      ueIndexIt!=m_ueToBePagedList.end(); ++ueIndexIt)
      {
        Ptr<Node> ue = ueNodes.Get (*ueIndexIt);

      for (uint32_t b = 0; b < numBearersPerUe; ++b)
        {
          ApplicationContainer dlClientApps;
          ApplicationContainer dlServerApps;

          ++dlPort;
          ++ulPort;

          NS_LOG_LOGIC("installing UDP DL app for UE " << (*ueIndexIt)+1);
          UdpClientHelper dlClientHelper (ueIpIfaces.GetAddress (*ueIndexIt), dlPort);
          dlClientHelper.SetAttribute ("Interval", TimeValue (MilliSeconds (interPacketInterval)));
          dlClientHelper.SetAttribute ("MaxPackets", UintegerValue (1000000));
          dlClientApps.Add (dlClientHelper.Install (remoteHost));

          PacketSinkHelper dlPacketSinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), dlPort));
          dlServerApps.Add (dlPacketSinkHelper.Install (ue));

          Ptr<EpcTft> tft = Create<EpcTft> ();
          EpcTft::PacketFilter dlpf;
          dlpf.localPortStart = dlPort;
          dlpf.localPortEnd = dlPort;
          tft->Add (dlpf);
          EpcTft::PacketFilter ulpf;
          ulpf.remotePortStart = ulPort;
          ulpf.remotePortEnd = ulPort;
          tft->Add (ulpf);
          EpsBearer bearer (EpsBearer::NGBR_IMS);
          lteHelper->ActivateDedicatedEpsBearer (ueDevs.Get (*ueIndexIt), bearer, tft);

          dlServerApps.Start (Seconds (1));
          dlClientApps.Start (Seconds (1));
        } // end for b
    }

  // Add X2 inteface
  if (m_nEnbs > 1)
    lteHelper->AddX2Interface (enbNodes);

  lteHelper->EnableTraces();

  for(uint32_t u = 0; u < m_nUes; ++u)
    {
  Simulator::Schedule (Seconds(0.9), &LtePagingTestCase::CheckIdle, this, ueDevs.Get(u), enbDevs);
    }

  // connect custom trace sinks for RRC connection establishment and paging notification
  Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/ConnectionEstablished",
                    MakeCallback (&LtePagingTestCase::ConnectionEstablishedEnbCallback, this));
   Config::Connect ("/NodeList/*/DeviceList/*/LteUeRrc/ConnectionEstablished",
                    MakeCallback (&LtePagingTestCase::ConnectionEstablishedUeCallback, this));
   Config::Connect ("/NodeList/*/DeviceList/*/LteUeRrc/StateTransition",
                                     MakeCallback (&LtePagingTestCase::UeStateTransitionCallback, this));
   Config::Connect ("/NodeList/*/DeviceList/*/LteUeRrc/Sib2Received",
                                        MakeCallback (&LtePagingTestCase::Sib2ReceivedCallback, this));
   Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/PagingMessageSent",
                                          MakeCallback (&LtePagingTestCase::PagingMessageSentCallback, this));

  Simulator::Stop (Seconds (simTime));

  Simulator::Run ();

  std::vector<Ptr<NetDevice> > ueConnectedList; //list of UEs which transition to connected after being paged
  std::vector<Ptr<NetDevice>> ueIdleList;//list of UEs which remain at idle since they are not paged
  uint16_t pagingSuccessListCount=0;
  for (std::vector<uint32_t>::iterator ueIndexIt=m_ueToBePagedList.begin();
      ueIndexIt!=m_ueToBePagedList.end(); ++ueIndexIt, ++pagingSuccessListCount)
      {
      if(m_pagingSuccessList.at(pagingSuccessListCount))
        {
      CheckConnected (ueDevs.Get(*ueIndexIt), enbDevs);
      ueConnectedList.push_back(ueDevs.Get(*ueIndexIt));
        }
      }

  //no need to sort since it already sorted
  std::set_difference(ueDevs.Begin(), ueDevs.End(), ueConnectedList.begin(), ueConnectedList.end(),
      std::inserter(ueIdleList, ueIdleList.begin()));
  for (std::vector<Ptr<NetDevice>>::iterator ueIndexIt=ueIdleList.begin();
       ueIndexIt!=ueIdleList.end(); ++ueIndexIt)
       {
      CheckIdle(*ueIndexIt, enbDevs);
       }

  Simulator::Destroy ();
}

void
LtePagingTestCase::CheckConnected (Ptr<NetDevice> ueDevice, NetDeviceContainer enbDevices)
{
  NS_LOG_FUNCTION (ueDevice);

  Ptr<LteUeNetDevice> ueLteDevice = ueDevice->GetObject<LteUeNetDevice> ();
  Ptr<LteUeRrc> ueRrc = ueLteDevice->GetRrc ();
  NS_TEST_ASSERT_MSG_EQ (ueRrc->GetState (), LteUeRrc::CONNECTED_NORMALLY, "Wrong LteUeRrc state!");
  uint16_t cellId = ueRrc->GetCellId();

  Ptr<LteEnbNetDevice> enbLteDevice;

  for( std::vector<Ptr<NetDevice> >::const_iterator enbDevIt=enbDevices.Begin();
      enbDevIt!=enbDevices.End();++enbDevIt)
    {
      if(((*enbDevIt)->GetObject<LteEnbNetDevice> ())->HasCellId(cellId))
        {
      enbLteDevice=(*enbDevIt)->GetObject<LteEnbNetDevice> ();
        }
    }

  NS_TEST_ASSERT_MSG_NE(enbLteDevice, 0, "LTE eNB device not found");

  Ptr<LteEnbRrc> enbRrc = enbLteDevice->GetRrc ();
  uint16_t rnti = ueRrc->GetRnti ();
  Ptr<UeManager> ueManager = enbRrc->GetUeManager (rnti);
  NS_TEST_ASSERT_MSG_NE (ueManager, 0, "RNTI " << rnti << " not found in eNB");

  UeManager::State ueManagerState = ueManager->GetState ();
  NS_TEST_ASSERT_MSG_EQ (ueManagerState, UeManager::CONNECTED_NORMALLY, "Wrong UeManager state!");

  uint16_t ueCellId = ueRrc->GetCellId ();
  uint16_t enbCellId = enbLteDevice->GetCellId ();
  uint8_t ueDlBandwidth = ueRrc->GetDlBandwidth ();
  uint8_t enbDlBandwidth = enbLteDevice->GetDlBandwidth ();
  uint8_t ueUlBandwidth = ueRrc->GetUlBandwidth ();
  uint8_t enbUlBandwidth = enbLteDevice->GetUlBandwidth ();
  uint8_t ueDlEarfcn = ueRrc->GetDlEarfcn ();
  uint8_t enbDlEarfcn = enbLteDevice->GetDlEarfcn ();
  uint8_t ueUlEarfcn = ueRrc->GetUlEarfcn ();
  uint8_t enbUlEarfcn = enbLteDevice->GetUlEarfcn ();
  uint64_t ueImsi = ueLteDevice->GetImsi ();
  uint64_t enbImsi = ueManager->GetImsi ();

  NS_TEST_ASSERT_MSG_EQ (ueImsi, enbImsi, "inconsistent IMSI");
  NS_TEST_ASSERT_MSG_EQ (ueCellId, enbCellId, "inconsistent CellId");
  NS_TEST_ASSERT_MSG_EQ (ueDlBandwidth, enbDlBandwidth, "inconsistent DlBandwidth");
  NS_TEST_ASSERT_MSG_EQ (ueUlBandwidth, enbUlBandwidth, "inconsistent UlBandwidth");
  NS_TEST_ASSERT_MSG_EQ (ueDlEarfcn, enbDlEarfcn, "inconsistent DlEarfcn");
  NS_TEST_ASSERT_MSG_EQ (ueUlEarfcn, enbUlEarfcn, "inconsistent UlEarfcn");

  ObjectMapValue enbDataRadioBearerMapValue;
  ueManager->GetAttribute ("DataRadioBearerMap", enbDataRadioBearerMapValue);
  NS_TEST_ASSERT_MSG_EQ (enbDataRadioBearerMapValue.GetN (), 2, "wrong num bearers at eNB");

  ObjectMapValue ueDataRadioBearerMapValue;
  ueRrc->GetAttribute ("DataRadioBearerMap", ueDataRadioBearerMapValue);
  NS_TEST_ASSERT_MSG_EQ (ueDataRadioBearerMapValue.GetN (), 2, "wrong num bearers at UE");

  ObjectMapValue::Iterator enbBearerIt = enbDataRadioBearerMapValue.Begin ();
  ObjectMapValue::Iterator ueBearerIt = ueDataRadioBearerMapValue.Begin ();
  while (enbBearerIt != enbDataRadioBearerMapValue.End ()
         && ueBearerIt != ueDataRadioBearerMapValue.End ())
    {
      Ptr<LteDataRadioBearerInfo> enbDrbInfo = enbBearerIt->second->GetObject<LteDataRadioBearerInfo> ();
      Ptr<LteDataRadioBearerInfo> ueDrbInfo = ueBearerIt->second->GetObject<LteDataRadioBearerInfo> ();
      NS_TEST_ASSERT_MSG_EQ ((uint32_t) enbDrbInfo->m_epsBearerIdentity, (uint32_t) ueDrbInfo->m_epsBearerIdentity, "epsBearerIdentity differs");
      NS_TEST_ASSERT_MSG_EQ ((uint32_t) enbDrbInfo->m_drbIdentity, (uint32_t) ueDrbInfo->m_drbIdentity, "drbIdentity differs");
      NS_TEST_ASSERT_MSG_EQ ((uint32_t) enbDrbInfo->m_logicalChannelIdentity, (uint32_t) ueDrbInfo->m_logicalChannelIdentity, "logicalChannelIdentity differs");

      ++enbBearerIt;
      ++ueBearerIt;
    }
  NS_ASSERT_MSG (enbBearerIt == enbDataRadioBearerMapValue.End (), "too many bearers at eNB");
  NS_ASSERT_MSG (ueBearerIt == ueDataRadioBearerMapValue.End (), "too many bearers at UE");
}

void
LtePagingTestCase::CheckIdle (Ptr<NetDevice> ueDevice, NetDeviceContainer enbDevices)
{
  NS_LOG_FUNCTION (ueDevice);

  Ptr<LteUeNetDevice> ueLteDevice = ueDevice->GetObject<LteUeNetDevice> ();
  Ptr<LteUeRrc> ueRrc = ueLteDevice->GetRrc ();
  NS_TEST_ASSERT_MSG_LT_OR_EQ(ueRrc->GetState (), LteUeRrc::IDLE_CAMPED_NORMALLY, "Wrong LteUeRrc state!");
  uint16_t cellId = ueRrc->GetCellId();

  Ptr<LteEnbNetDevice> enbLteDevice;
  bool hasCellId = false;

  for( std::vector<Ptr<NetDevice> >::const_iterator enbDevIt=enbDevices.Begin();
      enbDevIt!=enbDevices.End();++enbDevIt)
    {
      if(((*enbDevIt)->GetObject<LteEnbNetDevice> ())->HasCellId(cellId))
        {
      enbLteDevice=(*enbDevIt)->GetObject<LteEnbNetDevice> ();
      hasCellId = true;//set to true as this cell ID is served by this eNodeB
        }
    }
  if(!hasCellId)
    {
      return;//do not check if UE instance exists at an eNodeB since the UE is not synchronized to any enodeB
    }
  NS_TEST_ASSERT_MSG_NE(enbLteDevice, 0, "LTE eNB device not found");

  Ptr<LteEnbRrc> enbRrc = enbLteDevice->GetRrc ();
  uint16_t rnti = ueRrc->GetRnti ();
  bool ueManagerFound = enbRrc->HasUeManager(rnti);
  NS_TEST_ASSERT_MSG_EQ (ueManagerFound, false, "Unexpected RNTI with value " << rnti << " found in eNB");
}

void
LtePagingTestCase::UeStateTransitionCallback (
  std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti,
  LteUeRrc::State oldState, LteUeRrc::State newState)
{
  NS_LOG_FUNCTION (this << imsi << cellId << rnti << oldState << newState);
  if (oldState == LteUeRrc::IDLE_CAMPED_NORMALLY)
     {
      bool sib2Received = (std::find(m_sib2ReceivedList.begin(), m_sib2ReceivedList.end(), imsi)
      != m_sib2ReceivedList.end());
      NS_TEST_ASSERT_MSG_EQ (sib2Received, true, "connection establishment started before paging message is received for UE "<<imsi);
      uint16_t currentFrame=(uint16_t)(Simulator::Now().GetSeconds()*100)%m_pagingCycle;
      uint16_t currentSubframe=(uint16_t)(Simulator::Now().GetSeconds()*1000)%10;
      std::map<uint64_t, PagingInfo>::iterator it=m_pagingInfoList.find(imsi);
      NS_ASSERT_MSG (it!=m_pagingInfoList.end(), "paging info not found for UE "<<imsi);
      NS_TEST_ASSERT_MSG_EQ (it->second.pagingFrame, currentFrame, "UE receives paging message in frame!=paging frame");
      NS_TEST_ASSERT_MSG_EQ (it->second.pagingOccasion, currentSubframe, "UE receives paging message in sub frame!=paging occasion");
     }
}


void
LtePagingTestCase::ConnectionEstablishedUeCallback (
  std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
  NS_LOG_FUNCTION (this << imsi << cellId << rnti);
}

void
LtePagingTestCase::ConnectionEstablishedEnbCallback (
  std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
  NS_LOG_FUNCTION (this << imsi << cellId << rnti);
}

void
LtePagingTestCase::Sib2ReceivedCallback (
  std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
  NS_LOG_FUNCTION (this << imsi << cellId << rnti);
  m_sib2ReceivedList.push_back(imsi);
}

void
LtePagingTestCase::PagingMessageSentCallback (
  std::string context, uint64_t imsi, uint16_t pagingFrame, uint16_t pagingOccasion)
{
  NS_LOG_FUNCTION (this << imsi);
  PagingInfo info;
  info.pagingFrame=pagingFrame;
  info.pagingOccasion=pagingOccasion;
  m_pagingInfoList[imsi]=info;
}

/**
 * \brief Test suite for executing the paging test cases
 *
 * \sa ns3::LtePagingTestCase
 */
class LtePagingTestSuite : public TestSuite
{
public:
  LtePagingTestSuite ();
};

LtePagingTestSuite::LtePagingTestSuite ()
  : TestSuite ("lte-paging", SYSTEM)
{
  std::vector<Vector> uePositionList;
  uePositionList.push_back(Vector(200, 0, 0));
  uePositionList.push_back(Vector(400, 0, 0));
  uePositionList.push_back(Vector(600, 0, 0));
  uePositionList.push_back(Vector(900, 0, 0));
  uePositionList.push_back(Vector(1200, 0, 0));
  uePositionList.push_back(Vector(2500, 0, 0));

  std::vector<uint32_t> ueToBePagedList;
  ueToBePagedList.push_back(0);
  ueToBePagedList.push_back(2);
  ueToBePagedList.push_back(3);
  ueToBePagedList.push_back(5);

  std::vector<bool> pagingSuccessList;
  pagingSuccessList.push_back(true);
  pagingSuccessList.push_back(true);
  pagingSuccessList.push_back(true);
  pagingSuccessList.push_back(false);//paging fails for the last UE as it is outside the coverage of the 3 cells

  //                              nEnbs, nUes, isIdealRrc, uePositionList, ueToBePagedList, pagingSuccessList
  AddTestCase (new LtePagingTestCase (3, 6, false, uePositionList, ueToBePagedList, pagingSuccessList), TestCase::QUICK);

}// end of LtePagingTestSuite ()

static LtePagingTestSuite g_ltePagingTestSuite;


