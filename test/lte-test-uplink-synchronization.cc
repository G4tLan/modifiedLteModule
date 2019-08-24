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

#include "lte-test-uplink-synchronization.h"

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

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("LteUplinkSynchronizationTest");

/*
 * Test Suite
 */
LteUplinkSynchronizationTestSuite::LteUplinkSynchronizationTestSuite ()
  : TestSuite ("lte-uplink-synchronization", SYSTEM)
{
  // REAL RRC PROTOCOL
  LteUplinkSynchronizationTestCase::UeStatus status;
   status.isConnected=false;
   status.isUplinkOutOfSync=true;
  AddTestCase (new LteUplinkSynchronizationTestCase("UE is stationary + data transfer stops after sometime",
                                                    false, false, true, Vector(0, 0, 0),
                                                    100, Seconds(2), status),  TestCase::QUICK);

  status.isConnected=true;
  status.isUplinkOutOfSync=false;
 AddTestCase (new LteUplinkSynchronizationTestCase("UE is stationary + data transfer stops at simulation end",
                                                   false, true, true, Vector(0, 0, 0),
                                                   100, Seconds(12), status),  TestCase::QUICK);

 status.isConnected=false;
 status.isUplinkOutOfSync=true;
AddTestCase (new LteUplinkSynchronizationTestCase("UE is moving + data transfer stops after sometime",
                                                  false, false, true, Vector(30, 0, 0),
                                                  100, Seconds(2), status),  TestCase::QUICK);

status.isConnected=true;
status.isUplinkOutOfSync=false;
AddTestCase (new LteUplinkSynchronizationTestCase("UE is moving + data transfer stops stops at simulation end",
                                                 false, true, true, Vector(30, 0, 0),
                                                 100, Seconds(12), status),  TestCase::QUICK);

status.isConnected=true;
status.isUplinkOutOfSync=true;
AddTestCase (new LteUplinkSynchronizationTestCase("UE is stationary + PDCCH order based RACH for DL data arrival",
                                                 false, false, true, Vector(0, 0, 0),
                                                 1000, Seconds(12), status),  TestCase::QUICK);

status.isConnected=true;
status.isUplinkOutOfSync=true;
AddTestCase (new LteUplinkSynchronizationTestCase("UE is moving + contention based RACH for UL data arrival",
                                                 false, false, false, Vector(10, 0, 0),
                                                 500, Seconds(12), status),  TestCase::QUICK);



} // end of LteUplinkSynchronizationTestSuite::LteUplinkSynchronizationTestSuite ()


static LteUplinkSynchronizationTestSuite g_lteUplinkSynchronizationTestSuite;

/*
 * Test Case
 */
LteUplinkSynchronizationTestCase::LteUplinkSynchronizationTestCase (
  std::string name, bool isIdealRrc,
  bool isDynamicTimerConfiguration, bool isDl, Vector velocity,
  double interPacketInterval, Time applicationStopTime,
  UeStatus status
  )
  : TestCase (name),
    m_isIdealRrc (isIdealRrc),
    m_isDynamicTimerConfiguration(isDynamicTimerConfiguration),
    m_isDl(isDl),
    m_velocity(velocity),
    m_interPacketInterval(interPacketInterval),
    m_applicationStopTime(applicationStopTime),
    m_status(status)
{
  NS_LOG_FUNCTION (this << GetName ());
  m_lastState=LteUeRrc::NUM_STATES;
  m_isUplinkOutOfSync = false;
}


LteUplinkSynchronizationTestCase::~LteUplinkSynchronizationTestCase ()
{
  NS_LOG_FUNCTION (this << GetName ());
}


void
LteUplinkSynchronizationTestCase::DoRun ()
{
  NS_LOG_FUNCTION (this << GetName ());

  uint16_t numberOfUes = 1;
  uint16_t numberOfEnbs = 1;
  uint16_t numBearersPerUe = 1;
  double simTime = 12;
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

  //Timing advance and uplink out of sync parameters
  Config::SetDefault ("ns3::LteEnbMac::DynamicTimerConfiguration", BooleanValue (m_isDynamicTimerConfiguration));
  //Note: if interPacketInterval > TxRxInactivityTimeoutDuration, then UE goes out of sync at some pint of time
  Config::SetDefault ("ns3::LteEnbMac::TxRxInactivityTimeoutDuration", TimeValue (MilliSeconds (100)));//100ms
  Config::SetDefault ("ns3::LteEnbRrc::TimeAlignmentTimer", UintegerValue (1920));//1920ms
  Config::SetDefault ("ns3::LteEnbRrc::InactivityTimeoutDuration",  TimeValue (Seconds (10)));//10s

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
  ueNodes.Create (numberOfUes);

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

  positionAlloc1->Add (Vector (100, 0, 0));

  mobility.SetPositionAllocator (positionAlloc1);
  mobility.SetMobilityModel ("ns3::ConstantVelocityMobilityModel");
  mobility.Install (ueNodes);

 ueNodes.Get (0)->GetObject<ConstantVelocityMobilityModel> ()->SetVelocity (m_velocity);


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
  double interPacketInterval = m_interPacketInterval;

  for (uint32_t u = 0; u < numberOfUes; ++u)
    {
      Ptr<Node> ue = ueNodes.Get (u);
      // Set the default gateway for the UE
      Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (ue->GetObject<
          Ipv4> ());
      ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);

      for (uint32_t b = 0; b < numBearersPerUe; ++b)
        {

          Ptr<EpcTft> tft = Create<EpcTft> ();
          if (m_isDl)
            {
              ApplicationContainer dlClientApps;
              ApplicationContainer dlServerApps;

              ++dlPort;

              NS_LOG_LOGIC("installing UDP DL app for UE " << u+1);
              UdpClientHelper dlClientHelper (ueIpIfaces.GetAddress (u), dlPort);
              dlClientHelper.SetAttribute ("Interval", TimeValue (MilliSeconds (interPacketInterval)));
              dlClientHelper.SetAttribute ("MaxPackets", UintegerValue (1000000));
              dlClientApps.Add (dlClientHelper.Install (remoteHost));

              PacketSinkHelper dlPacketSinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), dlPort));
              dlServerApps.Add (dlPacketSinkHelper.Install (ue));

              EpcTft::PacketFilter dlpf;
              dlpf.localPortStart = dlPort;
              dlpf.localPortEnd = dlPort;
              tft->Add (dlpf);
              EpsBearer bearer (EpsBearer::NGBR_IMS);
              lteHelper->ActivateDedicatedEpsBearer (ueDevs.Get (u), bearer, tft);

              dlServerApps.Start (Seconds (0.01));
              dlClientApps.Start (Seconds (0.01));
              dlServerApps.Stop (m_applicationStopTime);
              dlClientApps.Stop (m_applicationStopTime);
            }

          if (!m_isDl)
            {
              ApplicationContainer ulClientApps;
              ApplicationContainer ulServerApps;

              ++ulPort;

              NS_LOG_LOGIC("installing UDP UL app for UE " << u+1);
              UdpClientHelper ulClientHelper (remoteHostAddr, ulPort);
              ulClientHelper.SetAttribute ("Interval", TimeValue (MilliSeconds (interPacketInterval)));
              ulClientHelper.SetAttribute ("MaxPackets", UintegerValue (1000000));
              ulClientApps.Add (ulClientHelper.Install (ue));

              PacketSinkHelper ulPacketSinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), ulPort));
              ulServerApps.Add (ulPacketSinkHelper.Install (remoteHost));

              EpcTft::PacketFilter ulpf;
              ulpf.remotePortStart = ulPort;
              ulpf.remotePortEnd = ulPort;
              tft->Add (ulpf);
              EpsBearer bearer (EpsBearer::NGBR_IMS);
              lteHelper->ActivateDedicatedEpsBearer (ueDevs.Get (u), bearer, tft);

              ulServerApps.Start (Seconds (1));
              ulClientApps.Start (Seconds (1));
              ulServerApps.Stop (m_applicationStopTime);
              ulClientApps.Stop (m_applicationStopTime);
            }
        } // end for b
    }

  // Add X2 inteface
  if (numberOfEnbs > 1)
    lteHelper->AddX2Interface (enbNodes);

  lteHelper->EnableTraces ();

  // connect custom trace sinks for RRC connection establishment, timing advance updates and RACH procedure
  Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/ConnectionEstablished",
                   MakeCallback (&LteUplinkSynchronizationTestCase::ConnectionEstablishedEnbCallback, this));
  Config::Connect ("/NodeList/*/DeviceList/*/LteUeRrc/ConnectionEstablished",
                   MakeCallback (&LteUplinkSynchronizationTestCase::ConnectionEstablishedUeCallback, this));
  Config::Connect ("/NodeList/*/DeviceList/*/LteUeRrc/StateTransition",
                   MakeCallback (&LteUplinkSynchronizationTestCase::UeStateTransitionCallback, this));
  Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/ConnectionReconfiguration",
                   MakeCallback (&LteUplinkSynchronizationTestCase::ConnectionReconfigCompleteReceivedCallback, this));
  Config::Connect ("/NodeList/*/DeviceList/*/LteUeRrc/ConnectionReconfiguration",
                   MakeCallback (&LteUplinkSynchronizationTestCase::ConnectionReconfigReceivedCallback, this));
  Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/TimerExpiry",
                   MakeCallback (&LteUplinkSynchronizationTestCase::EnbTimerExpiryCallback, this));
  Config::Connect ("/NodeList/*/DeviceList/*/LteUeRrc/UplinkOutofSync",
                   MakeCallback (&LteUplinkSynchronizationTestCase::UplinkOutofSyncCallback, this));
  Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/NotifyConnectionRelease",
                   MakeCallback (&LteUplinkSynchronizationTestCase::ConnectionReleaseAtEnbCallback, this));
  Config::Connect ("/NodeList/*/DeviceList/*/ComponentCarrierMapUe/*/LteUeMac/ConnectedOutOfSnycRach",
                   MakeCallback (&LteUplinkSynchronizationTestCase::ConnectedOutOfSnycRachCallback, this));
  Config::Connect ("/NodeList/*/DeviceList/*/ComponentCarrierMapUe/*/LteUeMac/TimeAlignmentTimerUpdate",
                   MakeCallback (&LteUplinkSynchronizationTestCase::TimeAlignmentTimerUpdateCallback, this));

  Simulator::Stop (Seconds (simTime));

  Simulator::Run ();
  if(m_status.isConnected)
    {
      CheckConnected (ueDevs.Get(0), enbDevs.Get(0));
    }
  else
    {
      CheckIdle (ueDevs.Get(0), enbDevs.Get(0));
    }
  NS_TEST_ASSERT_MSG_EQ (m_status.isUplinkOutOfSync, m_isUplinkOutOfSync, "wrong synchronization state");
  Simulator::Destroy ();

} // end of void LteUplinkSynchronizationTestCase::DoRun ()

void
LteUplinkSynchronizationTestCase::CheckConnected (Ptr<NetDevice> ueDevice, Ptr<NetDevice> enbDevice)
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
  NS_ASSERT_MSG (ueManagerState == UeManager::CONNECTED_NORMALLY, "Wrong UeManager state!");

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
LteUplinkSynchronizationTestCase::CheckIdle (Ptr<NetDevice> ueDevice, Ptr<NetDevice> enbDevice)
{
  NS_LOG_FUNCTION (ueDevice);

  Ptr<LteUeNetDevice> ueLteDevice = ueDevice->GetObject<LteUeNetDevice> ();
  Ptr<LteUeRrc> ueRrc = ueLteDevice->GetRrc ();
  NS_TEST_ASSERT_MSG_EQ(ueRrc->GetState (), LteUeRrc::IDLE_CAMPED_NORMALLY, "Wrong LteUeRrc state!");

  Ptr<LteEnbNetDevice> enbLteDevice = enbDevice->GetObject<LteEnbNetDevice> ();
  Ptr<LteEnbRrc> enbRrc = enbLteDevice->GetRrc ();
  uint16_t rnti = ueRrc->GetRnti ();
  bool ueManagerFound = enbRrc->HasUeManager(rnti);
  NS_TEST_ASSERT_MSG_EQ (ueManagerFound, false, "Unexpected RNTI with value " << rnti << " found in eNB");
}

void
LteUplinkSynchronizationTestCase::UeStateTransitionCallback (std::string context,
                                                             uint64_t imsi, uint16_t cellId,
                                                             uint16_t rnti, LteUeRrc::State oldState,
                                                             LteUeRrc::State newState)
{
  NS_LOG_FUNCTION(this << imsi << cellId << rnti << oldState << newState);
  m_lastState = newState;
}


void
LteUplinkSynchronizationTestCase::ConnectionEstablishedEnbCallback (
  std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
  NS_LOG_FUNCTION (this << imsi << cellId << rnti);
}

void
LteUplinkSynchronizationTestCase::ConnectionEstablishedUeCallback (
  std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
  NS_LOG_FUNCTION (this << imsi << cellId << rnti);
}

void
LteUplinkSynchronizationTestCase::EnbTimerExpiryCallback (std::string context, uint64_t imsi,
                                                          uint16_t rnti, uint16_t cellId, std::string cause)
{
  NS_LOG_FUNCTION (this << imsi << cellId << rnti<<cause);
}

void
LteUplinkSynchronizationTestCase::UplinkOutofSyncCallback (std::string context, uint64_t imsi,
                                                           uint16_t cellId, uint16_t rnti, std::string result)
{
  NS_LOG_FUNCTION (this << imsi << cellId << rnti);
  if(result== "Uplink out of sync considered")
    {
       m_isUplinkOutOfSync=true;
    }
}

void
LteUplinkSynchronizationTestCase::ConnectionReleaseAtEnbCallback (std::string context, uint64_t imsi,
                                                                  uint16_t cellId, uint16_t rnti)
{
  NS_LOG_FUNCTION (this << imsi << cellId << rnti);
}

void
LteUplinkSynchronizationTestCase::ConnectedOutOfSnycRachCallback (std::string context, uint64_t imsi,
                                                                  uint16_t rnti, std::string type)
{
  NS_LOG_FUNCTION (this << imsi << rnti<<type);
  NS_TEST_ASSERT_MSG_EQ(m_isUplinkOutOfSync, true, "RACH in connected state occurs when UE is already synchronized");
}

void
LteUplinkSynchronizationTestCase::TimeAlignmentTimerUpdateCallback (std::string context, uint64_t imsi, uint16_t rnti,
                                                                    Time timer, std::string cause)
{
  NS_LOG_FUNCTION (this << imsi << rnti<<cause);
  if(!m_isDynamicTimerConfiguration)
    {
      //The value of timeAlignmentTimer is harcoded to the default value 1920ms
      NS_TEST_ASSERT_MSG_EQ(timer, MilliSeconds(1920), "Timer value is not set to default for static configuration");
    }
}

void
LteUplinkSynchronizationTestCase::ConnectionReconfigReceivedCallback (std::string context, uint64_t imsi,
                                                              uint16_t cellId, uint16_t rnti)
{
  NS_LOG_FUNCTION (this << imsi << cellId << rnti);
}

void
LteUplinkSynchronizationTestCase::ConnectionReconfigCompleteReceivedCallback (std::string context, uint64_t imsi,
                                                                      uint16_t cellId, uint16_t rnti)
{
  NS_LOG_FUNCTION (this << imsi << cellId << rnti);
}
