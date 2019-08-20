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

#include "lte-test-handover-failure.h"

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

NS_LOG_COMPONENT_DEFINE("LteHandoverFailureTest");

/*
 * Test Suite
 */
LteHandoverFailureTestSuite::LteHandoverFailureTestSuite ()
  : TestSuite ("lte-handover-failure", SYSTEM)
{
  // REAL RRC PROTOCOL
  std::vector<uint32_t> ueHandoverFailureList;
  ueHandoverFailureList.push_back(11);
  AddTestCase (new LteHandoverFailureTestCase("Max preambles reached test", 14, false,
                                              MilliSeconds(200), MilliSeconds(500),
                                                    ueHandoverFailureList, Seconds(7)),  TestCase::QUICK);
  ueHandoverFailureList.clear();
  ueHandoverFailureList.push_back(0);
  //handoverJoiningTimeoutDuration is reduced to 20ms to test the handover failure when Handover Joining timer expires
  AddTestCase (new LteHandoverFailureTestCase("Handover Joining Timeout test", 1, false,
                                              MilliSeconds(20), MilliSeconds(500),
                                                    ueHandoverFailureList, Seconds(6)),  TestCase::QUICK);

} // end of LteHandoverFailureTestSuite::LteHandoverFailureTestSuite ()


static LteHandoverFailureTestSuite g_lteHandoverFailureTestSuite;

/*
 * Test Case
 */
LteHandoverFailureTestCase::LteHandoverFailureTestCase (
  std::string name, uint32_t nUes, bool isIdealRrc,
  Time handoverJoiningTimeoutDuration, Time handoverLeavingTimeoutDuration,
  std::vector<uint32_t> ueHandoverFailureList, Time simulationTime
  )
  : TestCase (name),
    m_nUes(nUes),
    m_isIdealRrc (isIdealRrc),
    m_handoverJoiningTimeoutDuration(handoverJoiningTimeoutDuration),
    m_handoverLeavingTimeoutDuration(handoverLeavingTimeoutDuration),
    m_ueHandoverFailureList(ueHandoverFailureList),
    m_simulationTime(simulationTime)
{
  NS_LOG_FUNCTION (this << GetName ());
  m_lastState=LteUeRrc::NUM_STATES;
}


LteHandoverFailureTestCase::~LteHandoverFailureTestCase ()
{
  NS_LOG_FUNCTION (this << GetName ());
}


void
LteHandoverFailureTestCase::DoRun ()
{
  NS_LOG_FUNCTION (this << GetName ());

  uint16_t numberOfEnbs = 2;
  uint16_t numBearersPerUe = 1;
  double eNodeB_txPower = 43;
  double interSiteDistance = 600;

  Config::SetDefault ("ns3::LteHelper::UseIdealRrc", BooleanValue (m_isIdealRrc));
   if (!m_isIdealRrc)
     {
       Config::SetDefault ("ns3::LteHelper::UseIdealPrach", BooleanValue (false));
     }
   else
     {
       Config::SetDefault ("ns3::LteHelper::UseIdealPrach", BooleanValue (true));
     }

  Ptr<LteHelper> lteHelper = CreateObject<LteHelper> ();
  Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper> ();
  lteHelper->SetEpcHelper (epcHelper);

  lteHelper->SetPathlossModelType (TypeId::LookupByName ("ns3::LogDistancePropagationLossModel"));
  lteHelper->SetPathlossModelAttribute ("Exponent", DoubleValue (3.5));
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

  //Set the TimeAlignmentTimer to max value so that only handover failure can be tested
  Config::SetDefault ("ns3::LteEnbRrc::TimeAlignmentTimer", UintegerValue (10240));//1920ms

  Config::SetDefault ("ns3::LteEnbRrc::HandoverJoiningTimeoutDuration", TimeValue (m_handoverJoiningTimeoutDuration)); //200
  Config::SetDefault ("ns3::LteEnbRrc::HandoverLeavingTimeoutDuration", TimeValue (m_handoverLeavingTimeoutDuration)); //500

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
  Ipv4Address remoteHostAddr = internetIpIfaces.GetAddress (1);
  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  Ptr<Ipv4StaticRouting> remoteHostStaticRouting = ipv4RoutingHelper.GetStaticRouting (remoteHost->GetObject<
      Ipv4> ());
  remoteHostStaticRouting->AddNetworkRouteTo (Ipv4Address ("7.0.0.0"), Ipv4Mask ("255.0.0.0"), 1);

  // Create Nodes: eNodeB and UE
  NodeContainer enbNodes;
  NodeContainer ueNodes;
  enbNodes.Create (numberOfEnbs);
  ueNodes.Create (m_nUes);

  //Mobility
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();

  for (uint16_t i = 0; i < numberOfEnbs; i++)
    {
      positionAlloc->Add (Vector (interSiteDistance * i, 0, 0));
    }
  MobilityHelper mobility;
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.SetPositionAllocator (positionAlloc);
  mobility.Install (enbNodes);

  Ptr<ListPositionAllocator> positionAlloc1 = CreateObject<ListPositionAllocator> ();

   for (uint32_t i = 0; i < m_nUes; i++)
      {
  positionAlloc1->Add (Vector (200, 0, 0));
      }

  mobility.SetPositionAllocator (positionAlloc1);
  mobility.SetMobilityModel ("ns3::ConstantVelocityMobilityModel");
  mobility.Install (ueNodes);

  for (uint32_t i = 0; i < m_nUes; i++)
    {
      ueNodes.Get (i)->GetObject<ConstantVelocityMobilityModel> ()->SetVelocity (Vector (25, 0.0, 0.0));
    }

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

      for (uint32_t b = 0; b < numBearersPerUe; ++b)
        {
          ApplicationContainer ulClientApps;
          ApplicationContainer ulServerApps;
          ApplicationContainer dlClientApps;
          ApplicationContainer dlServerApps;

          ++dlPort;
          ++ulPort;

          NS_LOG_LOGIC("installing UDP DL app for UE " << u+1);
          UdpClientHelper dlClientHelper (ueIpIfaces.GetAddress (u), dlPort);
          dlClientHelper.SetAttribute ("Interval", TimeValue (MilliSeconds (interPacketInterval)));
          dlClientHelper.SetAttribute ("MaxPackets", UintegerValue (1000000));
          dlClientApps.Add (dlClientHelper.Install (remoteHost));

          PacketSinkHelper dlPacketSinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), dlPort));
          dlServerApps.Add (dlPacketSinkHelper.Install (ue));

          NS_LOG_LOGIC("installing UDP UL app for UE " << u+1);
          UdpClientHelper ulClientHelper (remoteHostAddr, ulPort);
          ulClientHelper.SetAttribute ("Interval", TimeValue (MilliSeconds (interPacketInterval)));
          ulClientHelper.SetAttribute ("MaxPackets", UintegerValue (1000000));
          ulClientApps.Add (ulClientHelper.Install (ue));

          PacketSinkHelper ulPacketSinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), ulPort));
          ulServerApps.Add (ulPacketSinkHelper.Install (remoteHost));

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
          lteHelper->ActivateDedicatedEpsBearer (ueDevs.Get (u), bearer, tft);

          dlServerApps.Start (Seconds (0.01));
          dlClientApps.Start (Seconds (0.01));
          ulServerApps.Start (Seconds (0.01));
          ulClientApps.Start (Seconds (0.01));

        } // end for b
    }

  // Add X2 inteface
  if (numberOfEnbs > 1)
    lteHelper->AddX2Interface (enbNodes);

  lteHelper->EnableTraces();

  for(uint32_t u = 0; u < m_nUes; ++u)
    {
  Simulator::Schedule (Seconds(1.5), &LteHandoverFailureTestCase::CheckConnected, this, ueDevs.Get(u), enbDevs.Get(0));
    }

  // connect custom trace sinks for handover and RACH procedure
  Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/ConnectionEstablished",
                   MakeCallback (&LteHandoverFailureTestCase::ConnectionEstablishedEnbCallback, this));
  Config::Connect ("/NodeList/*/DeviceList/*/LteUeRrc/ConnectionEstablished",
                   MakeCallback (&LteHandoverFailureTestCase::ConnectionEstablishedUeCallback, this));
  Config::Connect ("/NodeList/*/DeviceList/*/LteUeRrc/StateTransition",
                   MakeCallback (&LteHandoverFailureTestCase::UeStateTransitionCallback, this));
  Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/TimerExpiry",
                   MakeCallback (&LteHandoverFailureTestCase::EnbTimerExpiryCallback, this));
  Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/NotifyConnectionRelease",
                   MakeCallback (&LteHandoverFailureTestCase::ConnectionReleaseAtEnbCallback, this));


  Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverStart",
                   MakeCallback (&LteHandoverFailureTestCase::HandoverStartEnbCallback, this));
  Config::Connect ("/NodeList/*/DeviceList/*/LteUeRrc/HandoverStart",
                   MakeCallback (&LteHandoverFailureTestCase::HandoverStartUeCallback, this));
  Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverEndOk",
                   MakeCallback (&LteHandoverFailureTestCase::HandoverEndOkEnbCallback, this));
  Config::Connect ("/NodeList/*/DeviceList/*/LteUeRrc/HandoverEndOk",
                   MakeCallback (&LteHandoverFailureTestCase::HandoverEndOkUeCallback, this));
  Config::Connect ("/NodeList/*/DeviceList/*/LteUeRrc/HandoverEndError",
                                     MakeCallback (&LteHandoverFailureTestCase::HandoverEndErrorUeCallback, this));
              Config::Connect ("/NodeList/*/DeviceList/*/LteUeRrc/RandomAccessError",
                                     MakeCallback (&LteHandoverFailureTestCase::RandomAccessErrorUeCallback, this));
              Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverFailure",
                                                  MakeCallback (&LteHandoverFailureTestCase::HandoverFailureEnbCallback, this));
  Simulator::Stop (m_simulationTime);

  Simulator::Run ();

  std::vector<Ptr<NetDevice> > ueHandoverSuccessfulList; //list of UEs which transition to connected after being paged
  std::vector<Ptr<NetDevice> > ueHandoverFailureList;
  for (std::vector<uint32_t>::iterator ueIndexIt = m_ueHandoverFailureList.begin ();
      ueIndexIt != m_ueHandoverFailureList.end (); ++ueIndexIt)
    {
      ueHandoverFailureList.push_back (ueDevs.Get (*ueIndexIt));
      if (GetName () == "Max preambles reached test") //UE remains connected to source cell when handover fails
        {
          CheckConnected (ueDevs.Get (*ueIndexIt), enbDevs.Get (0));
        }
      else //UE transitions to idle state when handover fails
        {
          CheckIdle (ueDevs.Get (*ueIndexIt), enbDevs);
        }
    }
  std::sort(ueHandoverFailureList.begin (), ueHandoverFailureList.end ());
  std::set_difference (ueDevs.Begin (), ueDevs.End (), ueHandoverFailureList.begin (),
  ueHandoverFailureList.end (), std::inserter (ueHandoverSuccessfulList, ueHandoverSuccessfulList.begin ()));
  for (std::vector<Ptr<NetDevice>>::iterator ueIndexIt = ueHandoverSuccessfulList.begin ();
      ueIndexIt != ueHandoverSuccessfulList.end (); ++ueIndexIt)
    {
      CheckConnected (*ueIndexIt, enbDevs.Get (1));
    }
  Simulator::Destroy ();

} // end of void LteHandoverFailureTestCase::DoRun ()

void
LteHandoverFailureTestCase::CheckConnected (Ptr<NetDevice> ueDevice, Ptr<NetDevice> enbDevice)
{
  NS_LOG_FUNCTION (ueDevice << enbDevice);

  Ptr<LteUeNetDevice> ueLteDevice = ueDevice->GetObject<LteUeNetDevice> ();
  Ptr<LteUeRrc> ueRrc = ueLteDevice->GetRrc ();
  NS_TEST_ASSERT_MSG_EQ (ueRrc->GetState (), LteUeRrc::CONNECTED_NORMALLY, "Wrong LteUeRrc state!");

  Ptr<LteEnbNetDevice> enbLteDevice = enbDevice->GetObject<LteEnbNetDevice> ();
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
  NS_TEST_ASSERT_MSG_EQ (enbDataRadioBearerMapValue.GetN (), 1 + 1, "wrong num bearers at eNB");

  ObjectMapValue ueDataRadioBearerMapValue;
  ueRrc->GetAttribute ("DataRadioBearerMap", ueDataRadioBearerMapValue);
  NS_TEST_ASSERT_MSG_EQ (ueDataRadioBearerMapValue.GetN (), 1 + 1, "wrong num bearers at UE");

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
LteHandoverFailureTestCase::CheckIdle (Ptr<NetDevice> ueDevice, NetDeviceContainer enbDevices)
{
  NS_LOG_FUNCTION (ueDevice);

  Ptr<LteUeNetDevice> ueLteDevice = ueDevice->GetObject<LteUeNetDevice> ();
  Ptr<LteUeRrc> ueRrc = ueLteDevice->GetRrc ();
  NS_TEST_ASSERT_MSG_LT_OR_EQ(ueRrc->GetState (), LteUeRrc::IDLE_CAMPED_NORMALLY, "Wrong LteUeRrc state!");
  uint16_t cellId = ueRrc->GetCellId();
  if(ueRrc->GetState ()==LteUeRrc::IDLE_CELL_SEARCH)
  {
    NS_TEST_ASSERT_MSG_EQ (cellId, 0, "Cell ID of the UE should be zero in the IDLE_CELL_SEARCH state");
    return;
  }
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
  bool ueManagerFound = enbRrc->HasUeManager(rnti);
  NS_TEST_ASSERT_MSG_EQ (ueManagerFound, false, "Unexpected RNTI with value " << rnti << " found in eNB");
}

void
LteHandoverFailureTestCase::UeStateTransitionCallback (std::string context,
                                                             uint64_t imsi, uint16_t cellId,
                                                             uint16_t rnti, LteUeRrc::State oldState,
                                                             LteUeRrc::State newState)
{
  NS_LOG_FUNCTION(this << imsi << cellId << rnti << oldState << newState);
  m_lastState = newState;
}


void
LteHandoverFailureTestCase::ConnectionEstablishedEnbCallback (
  std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
  NS_LOG_FUNCTION (this << imsi << cellId << rnti);
}

void
LteHandoverFailureTestCase::ConnectionEstablishedUeCallback (
  std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
  NS_LOG_FUNCTION (this << imsi << cellId << rnti);
}

void
LteHandoverFailureTestCase::EnbTimerExpiryCallback (std::string context, uint64_t imsi,
                                                          uint16_t rnti, uint16_t cellId, std::string cause)
{
  NS_LOG_FUNCTION (this << imsi << cellId << rnti<<cause);
}

void
LteHandoverFailureTestCase::ConnectionReleaseAtEnbCallback (std::string context, uint64_t imsi,
                                                                  uint16_t cellId, uint16_t rnti)
{
  NS_LOG_FUNCTION (this << imsi << cellId << rnti);
}

void
LteHandoverFailureTestCase::HandoverStartUeCallback (std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti, uint16_t targetCellId)
{
  NS_LOG_FUNCTION(this << imsi << cellId << rnti<<targetCellId);
}

void
LteHandoverFailureTestCase::HandoverEndOkUeCallback (std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
  NS_LOG_FUNCTION(this << imsi << cellId << rnti);
}

void
LteHandoverFailureTestCase::HandoverStartEnbCallback (std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti, uint16_t targetCellId)
{
  NS_LOG_FUNCTION(this << imsi << cellId << rnti<<targetCellId);
}

void
LteHandoverFailureTestCase::HandoverEndOkEnbCallback (std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
  NS_LOG_FUNCTION(this << imsi << cellId << rnti);
}

void
LteHandoverFailureTestCase::HandoverEndErrorUeCallback (std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
  NS_LOG_FUNCTION(this << imsi << cellId << rnti);
}

void
LteHandoverFailureTestCase::RandomAccessErrorUeCallback (std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
  NS_LOG_FUNCTION(this << imsi << cellId << rnti);
}

void
LteHandoverFailureTestCase::HandoverFailureEnbCallback (std::string context, uint64_t imsi, uint16_t rnti, uint16_t cellId, std::string cause)
{
  NS_LOG_FUNCTION(this << imsi << cellId << rnti<<cause);
}
